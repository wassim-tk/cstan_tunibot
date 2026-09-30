"""Autonomous frontier exploration of Dar TuniBot (Step 4: build the map without driving by hand).

Starts Nav2 (no map_server, no AMCL: slam_toolbox provides /map and map->odom) and then the
frontier explorer, which sends NavigateToPose goals to the edge of the known map.

Usage:
  1. ros2 launch cstam_gazebo dar_sim.launch.py        (Gazebo + slam_toolbox)
  2. ros2 launch cstam_navigation explore.launch.py    (this file)
  3. When /exploration_complete is published and the robot is back at the start, save the map:
     ros2 run nav2_map_server map_saver_cli -f <path>/maps/dar_map
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    nav_share = get_package_share_directory('cstam_navigation')

    nav2_params = os.path.join(nav_share, 'config', 'nav2_explore_params.yaml')

    # With use_composition, navigation_launch.py only LOADS nodes into an existing container
    # (bringup_launch.py normally creates it), so we create 'nav2_container' ourselves.
    container = Node(
        package='rclcpp_components',
        executable='component_container_isolated',
        name='nav2_container',
        parameters=[nav2_params, {'autostart': True, 'use_sim_time': True}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        output='screen',
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'),
                         'launch', 'navigation_launch.py')),
        launch_arguments={
            'params_file': nav2_params,
            'use_sim_time': 'true',
            'autostart': 'true',
            'use_composition': 'True',     # one process for all Nav2 nodes: saves RAM
            'container_name': 'nav2_container',
        }.items(),
    )

    explorer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('frontier_exploration_ros2'),
                         'launch', 'frontier_explorer.launch.py')),
        launch_arguments={
            'params_file': os.path.join(nav_share, 'config', 'explore_params.yaml'),
            'use_sim_time': 'true',
        }.items(),
    )

    # Give Nav2 a few seconds to come up before the explorer sends its first goal.
    return LaunchDescription([container, nav2, TimerAction(period=5.0, actions=[explorer])])
