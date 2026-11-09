"""
Drive simulator: the robot's real motion and its (wrong) odometry.

Same simulator as day 6 (package mcl_room), now for two worlds. It replaces
Gazebo and keeps two poses of the same robot:

    true pose  where the robot really is. Only the simulator knows it.
    odometry   what the wheel encoders believe. It drifts (day 6, slide 4).

    /cmd_vel --> [drive_sim] --> tf odom -> base_footprint   (odometry, for SLAM and AMCL)
                             --> /odom                       (same, as a message)
                             --> /sim/true_pose              (ground truth, frame world)
                             --> tf map -> world             (static, see below)

Every step the commanded motion is integrated twice:
  * true pose: exactly as commanded. A wall stops the robot (it bumps and
    the wheels stall, so the odometry does not count that step either).
  * odometry:  with a small scale error, a heading bias and random noise.
    SLAM has to correct this drift with scan matching and loop closure.

Why tf map -> world? SLAM starts its map frame where the robot is switched
on, like a real robot that knows nothing about the building. The walls of
the world are defined in their own frame "world". The simulator knows the
start pose, so it tells RViz where the world lies in the map frame. Only
the visualisation uses this, SLAM and Nav2 never look at it.

ROS parameters
    world              loop or room (default loop)
    rate               Hz (default 30)
    cmd_timeout        s, stop without new commands (default 0.6)
    max_linear         m/s (default 0.5)
    max_angular        rad/s (default 1.5)
    odom_scale_error   fraction, distances measured too long (default 0.02)
    odom_heading_bias  rad per metre driven (default 0.03)
    odom_noise_trans   std, fraction of the distance (default 0.05)
    odom_noise_rot     std, fraction of the turn (default 0.05)
    stamped            false: geometry_msgs/Twist on cmd_vel, true: TwistStamped
"""
import math
import random

from geometry_msgs.msg import PoseStamped, TransformStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from slam_nav.worlds import get_world, world_to_map, wrap
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def yaw_to_quaternion(yaw: float) -> tuple:
    """Rotation about z as quaternion (x, y, z, w)."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def quaternion_to_yaw(q) -> float:
    """Yaw angle of a quaternion message."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class DriveSim(Node):

    def __init__(self) -> None:
        super().__init__('drive_sim')
        p = self.declare_parameter
        self.world = get_world(p('world', 'loop').value)
        self.rate: float = p('rate', 30.0).value                       # Hz
        self.cmd_timeout: float = p('cmd_timeout', 0.6).value          # s, stop without commands
        self.max_linear: float = p('max_linear', 0.5).value            # m/s
        self.max_angular: float = p('max_angular', 1.5).value          # rad/s
        # odometry errors, see day 6, slide 4
        self.scale_error: float = p('odom_scale_error', 0.02).value    # 2 % too long
        self.heading_bias: float = p('odom_heading_bias', 0.03).value  # rad per metre driven
        self.noise_trans: float = p('odom_noise_trans', 0.05).value    # std, fraction of distance
        self.noise_rot: float = p('odom_noise_rot', 0.05).value        # std, fraction of turn
        stamped: bool = p('stamped', False).value                      # Twist or TwistStamped

        x, y, yaw = self.world.start
        self.true_x, self.true_y, self.true_yaw = x, y, yaw
        self.odom_x, self.odom_y, self.odom_yaw = 0.0, 0.0, 0.0   # odom starts at zero
        self.v = 0.0
        self.w = 0.0
        self.bumped = False
        self.last_cmd = self.get_clock().now()

        if stamped:
            self.create_subscription(TwistStamped, 'cmd_vel',
                                     lambda m: self.on_cmd(m.twist), 10)
        else:
            self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.true_pub = self.create_publisher(PoseStamped, '/sim/true_pose', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.publish_world_frame()
        self.create_timer(1.0 / self.rate, self.step)
        # allow changes while running, e.g. ros2 param set /drive_sim odom_heading_bias 0.1
        self.add_on_set_parameters_callback(self.on_params)
        kind = 'TwistStamped' if stamped else 'Twist'
        self.get_logger().info(
            f'World "{self.world.name}": robot starts at x={x:.2f}, y={y:.2f}, '
            f'yaw={math.degrees(yaw):.0f} deg (cmd_vel as {kind})')

    # -- inputs -------------------------------------------------------------

    def on_params(self, params) -> SetParametersResult:
        names = {
            'cmd_timeout': 'cmd_timeout', 'max_linear': 'max_linear',
            'max_angular': 'max_angular', 'odom_scale_error': 'scale_error',
            'odom_heading_bias': 'heading_bias', 'odom_noise_trans': 'noise_trans',
            'odom_noise_rot': 'noise_rot',
        }
        for param in params:
            if param.name in names:
                setattr(self, names[param.name], float(param.value))
                self.get_logger().info(f'{param.name} = {param.value}')
        return SetParametersResult(successful=True)

    def on_cmd(self, msg: Twist) -> None:
        self.v = max(-self.max_linear, min(self.max_linear, msg.linear.x))
        self.w = max(-self.max_angular, min(self.max_angular, msg.angular.z))
        self.last_cmd = self.get_clock().now()

    # -- simulation step ------------------------------------------------------

    def step(self) -> None:
        now = self.get_clock().now()
        dt = 1.0 / self.rate
        if (now - self.last_cmd).nanoseconds * 1e-9 > self.cmd_timeout:
            self.v, self.w = 0.0, 0.0

        d = self.v * dt   # commanded distance in this step
        a = self.w * dt   # commanded turn in this step

        # 1. True motion: drive along the arc, unless a wall is in the way
        yaw_mid = self.true_yaw + a / 2.0
        new_x = self.true_x + d * math.cos(yaw_mid)
        new_y = self.true_y + d * math.sin(yaw_mid)
        if self.world.is_free(new_x, new_y):
            self.true_x, self.true_y = new_x, new_y
        else:
            d = 0.0   # bumped into a wall: the wheels stall, nothing to count
            if not self.bumped:
                self.get_logger().warn('Bump! A wall is in the way.')
            self.bumped = True
        if d != 0.0:
            self.bumped = False
        self.true_yaw = wrap(self.true_yaw + a)

        # 2. Odometry: the encoders count the commanded wheel motion, with errors
        if d != 0.0 or a != 0.0:
            d_odom = d * (1.0 + self.scale_error) + random.gauss(0.0, self.noise_trans * abs(d))
            a_odom = (a + self.heading_bias * abs(d)
                      + random.gauss(0.0, self.noise_rot * abs(a) + 0.01 * abs(d)))
            yaw_mid = self.odom_yaw + a_odom / 2.0
            self.odom_x += d_odom * math.cos(yaw_mid)
            self.odom_y += d_odom * math.sin(yaw_mid)
            self.odom_yaw = wrap(self.odom_yaw + a_odom)

        self.publish(now.to_msg())

    def publish(self, stamp) -> None:
        q = yaw_to_quaternion(self.odom_yaw)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_footprint'
        tf.transform.translation.x = self.odom_x
        tf.transform.translation.y = self.odom_y
        tf.transform.rotation.x, tf.transform.rotation.y = q[0], q[1]
        tf.transform.rotation.z, tf.transform.rotation.w = q[2], q[3]
        self.tf_broadcaster.sendTransform(tf)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.odom_x
        odom.pose.pose.position.y = self.odom_y
        odom.pose.pose.orientation = tf.transform.rotation
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.w
        self.odom_pub.publish(odom)

        true = PoseStamped()
        true.header.stamp = stamp   # same stamp as the tf, the laser uses it
        true.header.frame_id = 'world'
        true.pose.position.x = self.true_x
        true.pose.position.y = self.true_y
        qt = yaw_to_quaternion(self.true_yaw)
        true.pose.orientation.z, true.pose.orientation.w = qt[2], qt[3]
        self.true_pub.publish(true)

    def publish_world_frame(self) -> None:
        """
        Publish the static tf map -> world: where the world origin lies in the map frame.

        The map frame is the start pose, so the world origin (0, 0, 0) seen
        from the start pose is world_to_map(origin).
        """
        x, y, yaw = world_to_map(self.world, 0.0, 0.0, 0.0)
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'map'
        tf.child_frame_id = 'world'
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        q = yaw_to_quaternion(yaw)
        tf.transform.rotation.z, tf.transform.rotation.w = q[2], q[3]
        self.static_broadcaster.sendTransform(tf)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DriveSim()
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
