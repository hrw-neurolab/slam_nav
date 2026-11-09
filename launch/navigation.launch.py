"""
Part of slam.launch.py and nav.launch.py: the Nav2 nodes of slide 23.

    goal (RViz "2D Goal Pose" or node tour) --> bt_navigator
        bt_navigator --> planner_server    (NavFn, global costmap)  --> /plan
                     --> controller_server (MPPI, local costmap)    --> /cmd_vel --> drive_sim
                     --> behavior_server   (spin, wait, back up when something fails)

All four are lifecycle nodes. lifecycle_manager_navigation configures and
activates them in this order. Parameters: config/nav2.yaml.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile

# (package, executable) of the Nav2 nodes, in the order they are activated
NODES = [
    ('nav2_controller', 'controller_server'),
    ('nav2_planner', 'planner_server'),
    ('nav2_behaviors', 'behavior_server'),
    ('nav2_bt_navigator', 'bt_navigator'),
]


def generate_launch_description():
    share = get_package_share_directory('slam_nav')
    # allow_substs: resolves $(find-pkg-share ...) in the file, like nav2_bringup does
    params = ParameterFile(os.path.join(share, 'config', 'nav2.yaml'), allow_substs=True)

    nodes = [Node(package=package, executable=name, name=name, output='screen',
                  parameters=[params])
             for package, name in NODES]
    return LaunchDescription(nodes + [
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'autostart': True,
                          'node_names': [name for _, name in NODES]}]),
    ])
