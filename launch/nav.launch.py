"""
Navigate on a map that SLAM has drawn before: AMCL and Nav2, no SLAM (slide 23).

    /cmd_vel (Nav2) --> drive_sim --tf odom->base_footprint--> amcl --tf map->odom--> Nav2
                            |                                  ^   ^
                            +--/sim/true_pose--> laser_sim --/scan  |
                                                 map_server --/map--+

Nodes started here:
    drive_sim, laser_sim, robot_viz, base_scan_tf   the simulated robot (sim.launch.py)
    map_server, amcl, lifecycle_manager_localization        the map and day 6's AMCL
    controller_server, planner_server, behavior_server, bt_navigator,
    lifecycle_manager_navigation                            Nav2 (navigation.launch.py)
    rviz2                                                   with config/<world>.rviz

Run with the map shipped in maps/, recorded with slam.launch.py and the tour:
    ros2 launch slam_nav nav.launch.py world:=loop
or with your own map:
    ros2 launch slam_nav nav.launch.py world:=loop map:=$HOME/loop_map.yaml
Then click "2D Goal Pose" in RViz, or run the tour:
    ros2 run slam_nav tour --ros-args -p world:=loop

The map must come from the same world. The robot starts where it started
while mapping, which is the origin of the map, so AMCL starts there too.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('slam_nav')
    world = LaunchConfiguration('world')
    range_max = LaunchConfiguration('range_max')

    def launch_file(name):
        return PythonLaunchDescriptionSource(os.path.join(share, 'launch', name))

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='loop', choices=['loop', 'room'],
                              description='loop: corridor of slide 8, room: room of day 6'),
        DeclareLaunchArgument('map',
                              default_value=PathJoinSubstitution(
                                  [share, 'maps', [world, '.yaml']]),
                              description='map yaml written by map_saver_cli'),
        DeclareLaunchArgument('range_max', default_value='3.5', description='lidar range in m'),
        DeclareLaunchArgument('noise_std', default_value='0.02',
                              description='lidar noise (standard deviation) in m'),
        DeclareLaunchArgument('rviz', default_value='true', description='start RViz'),

        # --- the simulated robot ---------------------------------------------
        IncludeLaunchDescription(
            launch_file('sim.launch.py'),
            launch_arguments={'world': world, 'range_max': range_max,
                              'noise_std': LaunchConfiguration('noise_std')}.items()),

        # --- the saved map and AMCL, as on day 6 ----------------------------------
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen', parameters=[{'yaml_filename': LaunchConfiguration('map')}]),
        Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen',
             parameters=[os.path.join(share, 'config', 'amcl.yaml')]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', output='screen',
             parameters=[{'autostart': True, 'node_names': ['map_server', 'amcl']}]),

        # --- Nav2 ----------------------------------------------------------------
        IncludeLaunchDescription(launch_file('navigation.launch.py')),

        # --- RViz ----------------------------------------------------------------
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', PathJoinSubstitution([share, 'config', [world, '.rviz']])],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
