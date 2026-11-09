"""
Laser simulator: what a 2D lidar on the robot would measure.

Same laser as day 6 (package mcl_room), for the world chosen with "world".

    /sim/true_pose --> [laser_sim] --> /scan   (sensor_msgs/LaserScan, frame base_scan)

For every beam it casts a ray from the TRUE pose against the walls of the
world (worlds.py) and adds a little Gaussian noise. SLAM never sees the
true pose, only these distances and the odometry.

The scan keeps the time stamp of the true pose. The odometry tf of drive_sim
has exactly the same stamp, so SLAM Toolbox and AMCL can always look it up.

ROS parameters
    world       loop or room (default loop)
    beams       number of beams (default 360, one per degree like a TurtleBot lidar)
    fov_deg     field of view in degrees (default 360)
    rate        scans per second (default 10)
    range_max   m (default 3.5, the range of the TurtleBot 3 lidar)
    noise_std   m (default 0.02)

A shorter range makes the corridor of the loop world harder to match, and
the loop closure easier to see:
    ros2 launch slam_nav slam.launch.py range_max:=2.0
"""
import math
import random
from typing import Optional

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from slam_nav.drive_sim import quaternion_to_yaw
from slam_nav.worlds import get_world


class LaserSim(Node):

    def __init__(self) -> None:
        super().__init__('laser_sim')
        p = self.declare_parameter
        self.world = get_world(p('world', 'loop').value)
        self.beams: int = p('beams', 360).value
        fov = math.radians(p('fov_deg', 360.0).value)
        rate: float = p('rate', 10.0).value
        self.range_min: float = p('range_min', 0.12).value
        self.range_max: float = p('range_max', 3.5).value
        self.noise_std: float = p('noise_std', 0.02).value

        # beams spread evenly over the field of view, centred on the robot's x axis
        if fov >= 2.0 * math.pi - 1e-6:
            self.increment = 2.0 * math.pi / self.beams          # full circle, no double beam
            self.angle_min = -math.pi
        else:
            self.increment = fov / (self.beams - 1) if self.beams > 1 else 0.0
            self.angle_min = -fov / 2.0

        self.pose: Optional[PoseStamped] = None
        self.create_subscription(PoseStamped, '/sim/true_pose', self.on_pose, 10)
        self.pub = self.create_publisher(LaserScan, 'scan', 10)
        self.create_timer(1.0 / rate, self.publish_scan)
        self.get_logger().info(
            f'{self.beams} beams, {math.degrees(fov):.0f} deg field of view, '
            f'{self.range_max:.1f} m range, {rate:.0f} Hz')

    def on_pose(self, msg: PoseStamped) -> None:
        self.pose = msg

    def publish_scan(self) -> None:
        if self.pose is None:
            return
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        yaw = quaternion_to_yaw(self.pose.pose.orientation)

        scan = LaserScan()
        scan.header.stamp = self.pose.header.stamp
        scan.header.frame_id = 'base_scan'
        scan.angle_min = self.angle_min
        scan.angle_increment = self.increment
        scan.angle_max = self.angle_min + self.increment * (self.beams - 1)
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.scan_time = 0.0
        scan.time_increment = 0.0

        ranges = []
        for i in range(self.beams):
            angle = yaw + self.angle_min + i * self.increment
            r = self.world.raycast(x, y, angle, self.range_max)
            if r is None:
                ranges.append(math.inf)        # no echo
            else:
                ranges.append(max(self.range_min, r + random.gauss(0.0, self.noise_std)))
        scan.ranges = ranges
        self.pub.publish(scan)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LaserSim()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise  # a real error. Otherwise Ctrl+C stopped ROS while a timer was publishing
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
