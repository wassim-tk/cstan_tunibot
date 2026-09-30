"""Phase 1 - Mapping: Gazebo + slam_toolbox + Nav2 (no AMCL / map_server).

    ros2 launch cstam_navigation slam.launch.py
    ros2 run cstam_core mapping_tour      # or drive with teleop / RViz goals
    ros2 run cstam_core save_map          # writes cstam_navigation/maps/cstam_map.*

The map frame of slam_toolbox starts at the robot's spawn pose (the dock). A
static world -> map transform is published so that goals / RViz can use world
coordinates while mapping; save_map applies the same offset to the saved map.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DOCK_X, DOCK_Y = '7.85', '-12.79'


def generate_launch_description():
    pkg_nav = get_package_share_directory('cstam_navigation')
    pkg_gazebo = get_package_share_directory('cstam_gazebo')
    pkg_nav2 = get_package_share_directory('nav2_bringup')
    pkg_slam = get_package_share_directory('slam_toolbox')

    headless = LaunchConfiguration('headless')
    rviz = LaunchConfiguration('rviz')
    params = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo, 'launch', 'spawn_cstam_robot.launch.py')),
        launch_arguments={'headless': headless, 'spawn_x': DOCK_X, 'spawn_y': DOCK_Y,
                          'spawn_yaw': '0.0'}.items(),
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_slam, 'launch', 'online_async_launch.py')),
        launch_arguments={'slam_params_file': os.path.join(pkg_nav, 'config', 'slam_params.yaml'),
                          'use_sim_time': 'true'}.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')),
        launch_arguments={'params_file': params, 'use_sim_time': 'true',
                          'autostart': 'true'}.items(),
    )

    world_to_map = Node(
        package='tf2_ros', executable='static_transform_publisher', name='world_to_map',
        arguments=['--x', DOCK_X, '--y', DOCK_Y, '--frame-id', 'world', '--child-frame-id', 'map'],
        parameters=[{'use_sim_time': True}],
    )

    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='log',
        arguments=['-d', os.path.join(pkg_nav2, 'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false', description='Gazebo without GUI'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz'),
        gazebo,
        world_to_map,
        # Give Gazebo time to start publishing /clock, /odom and /scan.
        TimerAction(period=5.0, actions=[slam, nav2]),
        rviz_node,
    ])
