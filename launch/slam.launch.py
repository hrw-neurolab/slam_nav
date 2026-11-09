"""
Map a world with SLAM Toolbox, and navigate in it while it is being drawn.

    /cmd_vel (teleop or Nav2) --> drive_sim --tf odom->base_footprint--> slam_toolbox
                                      |                                   ^     |
                                      +--/sim/true_pose--> laser_sim --/scan    |
                                                                                v
                            Nav2 <--/map, tf map->odom-------------------- RViz

Nodes started here:
    drive_sim, laser_sim, robot_viz, base_scan_tf   the simulated robot (sim.launch.py)
    slam_toolbox                  SLAM, synchronous like Nav2's slam:=True    (slides 11, 12)
    lifecycle_manager_slam        configures and activates slam_toolbox
    controller_server, planner_server, behavior_server, bt_navigator,
    lifecycle_manager_navigation  Nav2, only with nav:=true          (navigation.launch.py)
    rviz2                         with config/<world>.rviz

Run:
    ros2 launch slam_nav slam.launch.py                      # loop of slide 8
    ros2 launch slam_nav slam.launch.py world:=room          # room of day 6
Loop closure of slide 8: a weaker lidar lets the robot drift, then compare
    ros2 launch slam_nav slam.launch.py range_max:=2.0 noise_std:=0.05 loop_closing:=false
    ros2 launch slam_nav slam.launch.py range_max:=2.0 noise_std:=0.05 loop_closing:=true
Drive in a second terminal, by hand or with the tour:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ros2 run slam_nav tour --ros-args -p world:=loop
Save the map in a third terminal:
    ros2 run nav2_map_server map_saver_cli -f ~/loop_map
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('slam_nav')
    world = LaunchConfiguration('world')
    range_max = LaunchConfiguration('range_max')

    def launch_file(name):
        return PythonLaunchDescriptionSource(os.path.join(share, 'launch', name))

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='loop', choices=['loop', 'room'],
                              description='loop: corridor of slide 8, room: room of day 6'),
        DeclareLaunchArgument('loop_closing', default_value='true',
                              description='false: SLAM Toolbox never closes a loop'),
        DeclareLaunchArgument('range_max', default_value='3.5',
                              description='lidar range in m, for laser_sim and slam_toolbox'),
        DeclareLaunchArgument('noise_std', default_value='0.02',
                              description='lidar noise (standard deviation) in m'),
        DeclareLaunchArgument('nav', default_value='true',
                              description='start Nav2, so goals can be sent while mapping'),
        DeclareLaunchArgument('rviz', default_value='true', description='start RViz'),

        # --- the simulated robot ---------------------------------------------
        IncludeLaunchDescription(
            launch_file('sim.launch.py'),
            launch_arguments={'world': world, 'range_max': range_max,
                              'noise_std': LaunchConfiguration('noise_std')}.items()),

        # --- SLAM Toolbox ------------------------------------------------------
        Node(package='slam_toolbox', executable='sync_slam_toolbox_node', name='slam_toolbox',
             output='screen',
             parameters=[os.path.join(share, 'config', 'slam_toolbox.yaml'), {
                 'do_loop_closing': ParameterValue(LaunchConfiguration('loop_closing'),
                                                   value_type=bool),
                 'max_laser_range': ParameterValue(range_max, value_type=float)}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_slam', output='screen',
             parameters=[{'autostart': True, 'node_names': ['slam_toolbox']}]),

        # --- Nav2 ----------------------------------------------------------------
        IncludeLaunchDescription(launch_file('navigation.launch.py'),
                                 condition=IfCondition(LaunchConfiguration('nav'))),

        # --- RViz ----------------------------------------------------------------
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', PathJoinSubstitution([share, 'config', [world, '.rviz']])],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
