"""Nav2 localization (map_server + AMCL) and navigation on the saved map.

Included by `cstam_core/cstam_system.launch.py`; can also be used on its own
next to a running simulation:

    ros2 launch cstam_navigation navigation.launch.py [map:=/path/to/map.yaml]
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_nav = get_package_share_directory('cstam_navigation')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': 'true',
            'autostart': 'true',
            'slam': 'False',
            'use_composition': 'True',
            'use_respawn': 'False',
        }.items(),
    )

    # The saved map is anchored in world coordinates: world == map.
    world_to_map = Node(
        package='tf2_ros', executable='static_transform_publisher', name='world_to_map',
        arguments=['--frame-id', 'world', '--child-frame-id', 'map'],
        parameters=[{'use_sim_time': True}],
    )

    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='log',
        arguments=['-d', os.path.join(pkg_nav2, 'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=os.path.join(pkg_nav, 'maps', 'cstam_map.yaml'),
                              description='Map yaml produced by save_map'),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(pkg_nav, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz'),
        nav2,
        world_to_map,
        rviz_node,
    ])
