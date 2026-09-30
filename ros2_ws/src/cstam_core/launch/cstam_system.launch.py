"""Phase 1 full system: simulation + localization/navigation on the saved
map + delivery task manager + auto-docking controller.

    ros2 launch cstam_core cstam_system.launch.py
    ros2 run cstam_core request_delivery 3 "Espresso"
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('cstam_gazebo')
    pkg_nav = get_package_share_directory('cstam_navigation')
    waypoints = os.path.join(pkg_nav, 'config', 'waypoints.yaml')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo, 'launch', 'spawn_cstam_robot.launch.py')),
        launch_arguments={'headless': LaunchConfiguration('headless')}.items(),
        condition=IfCondition(LaunchConfiguration('sim')),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav, 'launch', 'navigation.launch.py')),
        launch_arguments={'map': LaunchConfiguration('map'),
                          'rviz': LaunchConfiguration('rviz')}.items(),
    )

    task_manager = Node(
        package='cstam_core', executable='delivery_task_manager', name='delivery_task_manager',
        output='screen',
        parameters=[{'use_sim_time': True, 'waypoints_file': waypoints,
                     'load_time': LaunchConfiguration('load_time'),
                     'unload_time': LaunchConfiguration('unload_time')}],
    )

    docking_controller = Node(
        package='cstam_core', executable='docking_controller', name='docking_controller',
        output='screen',
        parameters=[{'use_sim_time': True, 'waypoints_file': waypoints,
                     'idle_timeout': LaunchConfiguration('idle_timeout')}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='true', description='Start Gazebo'),
        DeclareLaunchArgument('headless', default_value='false', description='Gazebo without GUI'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz'),
        DeclareLaunchArgument('map', default_value=os.path.join(pkg_nav, 'maps', 'cstam_map.yaml')),
        DeclareLaunchArgument('load_time', default_value='10.0',
                              description='Seconds waited at the kitchen if nobody confirms'),
        DeclareLaunchArgument('unload_time', default_value='10.0',
                              description='Seconds waited at the table if nobody confirms'),
        DeclareLaunchArgument('idle_timeout', default_value='30.0',
                              description='Idle seconds before returning to the dock'),
        sim,
        # Start Nav2 once Gazebo publishes /clock, /tf and /scan.
        TimerAction(period=5.0, actions=[navigation]),
        TimerAction(period=8.0, actions=[task_manager, docking_controller]),
    ])
