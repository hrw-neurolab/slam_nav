"""
Visualisation only: draws the true walls and the robot like on day 6.

    /sim/true_pose --+
                     +--> [robot_viz] --> /viz/markers   (visualization_msgs/MarkerArray)
    /scan -----------+

What you see in RViz:
    thin dark lines    the TRUE walls of the world (frame world). The map that
                       SLAM draws should lie exactly under them.
    blue robot         where the robot BELIEVES it is: drawn in frame base_footprint,
                       so RViz places it with the tf chain map -> odom -> base_footprint,
                       that is the SLAM correction (or AMCL's) plus the odometry
    grey robot         where the robot REALLY is (ground truth from the simulator)
    blue beams         the laser scan, drawn from the believed pose

This node has no effect on SLAM or navigation. Delete it and both work the same.
"""
import math
from typing import List, Optional, Tuple

from geometry_msgs.msg import Point, PoseStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from slam_nav.worlds import get_world, ROBOT_RADIUS
from visualization_msgs.msg import Marker, MarkerArray

BLUE = (0.165, 0.392, 0.588, 1.0)     # #2A6496, the colours of the slides
GREY = (0.373, 0.400, 0.447, 0.8)     # #5F6672
WHITE = (0.957, 0.949, 0.925, 1.0)    # #F4F2EC
BLACK = (0.071, 0.102, 0.141, 0.6)    # #121A24, a bit transparent over the SLAM map


def make_marker(frame: str, ns: str, mid: int, kind: int,
                color: Tuple[float, float, float, float], width: float = 0.03) -> Marker:
    m = Marker()
    m.header.frame_id = frame
    m.ns = ns
    m.id = mid
    m.type = kind
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    m.scale.x = m.scale.y = m.scale.z = width
    m.color.r, m.color.g, m.color.b, m.color.a = color
    return m


def pt(x: float, y: float, z: float = 0.0) -> Point:
    return Point(x=x, y=y, z=z)


def robot_markers(frame: str, ns: str, color: Tuple[float, float, float, float],
                  label: str) -> List[Marker]:
    """Build a robot like on day 6: light disc, coloured outline and a heading line."""
    r = ROBOT_RADIUS
    disc = make_marker(frame, ns, 0, Marker.CYLINDER, WHITE)
    disc.scale.x = disc.scale.y = 2.0 * r
    disc.scale.z = 0.01
    disc.color.a = color[3]

    outline = make_marker(frame, ns, 1, Marker.LINE_STRIP, color, 0.035)
    outline.points = [pt(r * math.cos(a), r * math.sin(a), 0.01)
                      for a in [2.0 * math.pi * i / 36 for i in range(37)]]

    heading = make_marker(frame, ns, 2, Marker.LINE_LIST, color, 0.035)
    heading.points = [pt(0.0, 0.0, 0.01), pt(r + 0.12, 0.0, 0.01)]

    text = make_marker(frame, ns, 3, Marker.TEXT_VIEW_FACING, color, 0.14)
    text.pose.position.y = r + 0.15
    text.text = label
    return [disc, outline, heading, text]


class RobotViz(Node):

    def __init__(self) -> None:
        super().__init__('robot_viz')
        self.world = get_world(self.declare_parameter('world', 'loop').value)
        self.true_pose: Optional[PoseStamped] = None
        self.scan: Optional[LaserScan] = None
        self.create_subscription(PoseStamped, '/sim/true_pose', self.on_true, 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, 10)
        self.pub = self.create_publisher(MarkerArray, '/viz/markers', 10)
        self.create_timer(0.1, self.publish)

    def on_true(self, msg: PoseStamped) -> None:
        self.true_pose = msg

    def on_scan(self, msg: LaserScan) -> None:
        self.scan = msg

    def publish(self) -> None:
        markers = MarkerArray()

        walls = make_marker('world', 'world', 0, Marker.LINE_LIST, BLACK, 0.025)
        for (ax, ay), (bx, by) in self.world.walls:
            walls.points += [pt(ax, ay, 0.02), pt(bx, by, 0.02)]
        markers.markers.append(walls)

        # believed pose: frame base_footprint, RViz looks up map -> odom -> base_footprint
        markers.markers += robot_markers('base_footprint', 'estimate', BLUE, 'estimate')

        # true pose: frame world, placed at the simulator's ground truth
        if self.true_pose is not None:
            for m in robot_markers('world', 'true', GREY, 'true'):
                p, q = self.true_pose.pose.position, self.true_pose.pose.orientation
                m.pose.position.x += p.x
                m.pose.position.y += p.y
                m.pose.orientation = q
                if m.type == Marker.TEXT_VIEW_FACING:   # keep the label above the robot
                    m.pose.position.x = p.x
                    m.pose.position.y = p.y + ROBOT_RADIUS + 0.15
                markers.markers.append(m)

        # beams of the last scan, drawn from the believed pose
        if self.scan is not None:
            beams = make_marker('base_scan', 'beams', 0, Marker.LINE_LIST,
                                (BLUE[0], BLUE[1], BLUE[2], 0.25), 0.008)
            ends = make_marker('base_scan', 'beams', 1, Marker.SPHERE_LIST, BLUE, 0.04)
            beams.header.stamp = ends.header.stamp = self.scan.header.stamp
            for i, r in enumerate(self.scan.ranges):
                if not math.isfinite(r):
                    continue
                a = self.scan.angle_min + i * self.scan.angle_increment
                end = pt(r * math.cos(a), r * math.sin(a), 0.02)
                if i % 4 == 0:   # every 4th beam as a line, else the picture is all blue
                    beams.points += [pt(0.0, 0.0, 0.02), end]
                ends.points.append(end)
            markers.markers += [beams, ends]

        self.pub.publish(markers)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RobotViz()
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
