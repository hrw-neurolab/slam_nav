"""
Tour: send the robot through its world with Nav2, one goal after another.

    [tour] --action navigate_to_pose--> bt_navigator (Nav2) --> planner, controller --> /cmd_vel

The goals of each world are listed in worlds.py ("tour"):
    loop   once around the square corridor and 2 m further, like slide 8
    room   a round trip through both halves of the room

Use it while mapping, so that every run drives the same way:
    ros2 launch slam_nav slam.launch.py world:=loop
    ros2 run slam_nav tour --ros-args -p world:=loop

The goals are given in frame world. SLAM's map frame starts at the start
pose of the robot, so the node converts every goal with world_to_map()
before sending it in frame map. Nav2 plans through cells it has not seen
yet (allow_unknown: true), that is how it can explore while mapping.

ROS parameters
    world      loop or room (default loop)
    laps       how often to repeat the tour (default 1)
"""
import math
from typing import List

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.task import Future
from slam_nav.worlds import get_world, world_to_map


class Tour(Node):

    def __init__(self) -> None:
        super().__init__('tour')
        self.world = get_world(self.declare_parameter('world', 'loop').value)
        laps: int = self.declare_parameter('laps', 1).value
        self.goals: List[tuple] = list(self.world.tour) * laps
        self.client: ActionClient = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def make_goal(self, x: float, y: float, yaw: float) -> NavigateToPose.Goal:
        mx, my, myaw = world_to_map(self.world, x, y, yaw)
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = mx
        pose.pose.position.y = my
        pose.pose.orientation.z = math.sin(myaw / 2.0)
        pose.pose.orientation.w = math.cos(myaw / 2.0)
        goal = NavigateToPose.Goal()
        goal.pose = pose
        return goal

    def send(self, goal: NavigateToPose.Goal, tries: int = 30):
        """Send a goal. Nav2 rejects it while it is still starting up, so try again."""
        for _ in range(tries):
            future = self.client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future)
            handle = future.result()
            if handle is not None and handle.accepted:
                return handle
            self.get_logger().info('Nav2 is not active yet, trying again in 2 s ...')
            rclpy.spin_until_future_complete(self, Future(), timeout_sec=2.0)  # wait 2 s
        return None

    def run(self) -> bool:
        self.get_logger().info('Waiting for Nav2 (action navigate_to_pose) ...')
        self.client.wait_for_server()
        for i, (x, y, yaw) in enumerate(self.goals, start=1):
            self.get_logger().info(
                f'Goal {i}/{len(self.goals)}: world x={x:.2f}, y={y:.2f}, '
                f'yaw={math.degrees(yaw):.0f} deg')
            handle = self.send(self.make_goal(x, y, yaw))
            if handle is None:
                self.get_logger().error('Nav2 rejected the goal.')
                return False
            result = handle.get_result_async()
            rclpy.spin_until_future_complete(self, result)
            res = result.result()
            if res is None or res.status != GoalStatus.STATUS_SUCCEEDED:
                status = res.status if res is not None else 'none'
                self.get_logger().error(f'Goal {i} failed (status {status}), stopping the tour.')
                return False
            self.get_logger().info(f'Goal {i} reached.')
        self.get_logger().info('Tour finished.')
        return True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Tour()
    try:
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
