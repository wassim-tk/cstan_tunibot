"""Launch the Dar TuniBot restaurant with the robot at its start pose.

It reuses Wassim's spawn_cstam_robot.launch.py and only fills in our values.
Usage:  ros2 launch cstam_gazebo dar_sim.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_cstam_gazebo = get_package_share_directory('cstam_gazebo')

    spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_cstam_gazebo, 'launch', 'spawn_cstam_robot.launch.py')
        ),
        launch_arguments={
            'world': 'dar_tunibot.world',
            'spawn_x': '-9.0',     # robot start = future map origin
            'spawn_y': '-1.45',
            'spawn_yaw': '0.0',    # facing east, into the restaurant
        }.items(),
    )

    return LaunchDescription([spawn_launch])