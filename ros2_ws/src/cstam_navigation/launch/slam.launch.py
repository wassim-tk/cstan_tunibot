"""Start slam_toolbox (online async, mapping mode) for the Dar TuniBot world.

slam_toolbox is a lifecycle node on Jazzy: it must be configured and activated before /map
appears. slam_toolbox's own online_async_launch.py does that for us (autostart:=true).

Usage:  ros2 launch cstam_navigation slam.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    slam_params_file = os.path.join(
        get_package_share_directory('cstam_navigation'), 'config', 'slam_params.yaml')

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'),
                         'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time': 'true',
            'autostart': 'true',
        }.items(),
    )

    return LaunchDescription([slam_launch])
