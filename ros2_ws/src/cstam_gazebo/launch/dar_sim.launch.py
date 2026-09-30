"""Launch the Dar TuniBot restaurant with the robot at its start pose.

It reuses Wassim's spawn_cstam_robot.launch.py and only fills in our values.
Usage:  ros2 launch cstam_gazebo dar_sim.launch.py
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

    # Gazebo publishes the wheel joint angles on /joint_states (JointStatePublisher plugin in
    # the URDF). This bridge copies them into ROS, so robot_state_publisher can place the
    # wheels and RViz's RobotModel stops showing errors for the wheel links.
    # '[' means one direction only: Gazebo -> ROS.
    joint_state_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='joint_state_bridge',
        arguments=['/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model'],
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # SLAM (slam_toolbox, mapping mode) starts with the simulation. Turn it off with slam:=false.
    slam_arg = DeclareLaunchArgument('slam', default_value='true',
                                     description='Start slam_toolbox with the simulation')
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('cstam_navigation'),
                         'launch', 'slam.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('slam')),
    )

    return LaunchDescription([slam_arg, spawn_launch, joint_state_bridge, slam_launch])