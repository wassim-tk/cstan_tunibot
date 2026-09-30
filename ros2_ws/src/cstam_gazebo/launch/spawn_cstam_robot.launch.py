"""Gazebo Sim (Harmonic) bringup: restaurant world + CSTAM robot + ROS <-> GZ bridge.

The robot is spawned on the docking station by default. Its heading is 0 rad
(facing +X) so that the wheel-odometry frame is only *translated* from the
world frame: this lets `save_map` re-anchor a SLAM map in world coordinates.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import xacro

# Docking station pad centre in restaurant.world (robot faces the charging tower).
DOCK_X = '7.85'
DOCK_Y = '-12.79'


def generate_launch_description():
    pkg_cstam_gazebo = get_package_share_directory('cstam_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')

    declare_args = [
        DeclareLaunchArgument('world', default_value='restaurant.world',
                              description='World file name inside cstam_gazebo/worlds'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Run Gazebo server only (no GUI)'),
        DeclareLaunchArgument('spawn_x', default_value=DOCK_X),
        DeclareLaunchArgument('spawn_y', default_value=DOCK_Y),
        DeclareLaunchArgument('spawn_z', default_value='0.05'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
    ]

    # Let Gazebo resolve package://cstam_gazebo/... and model:// URIs
    set_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.pathsep.join([
            os.path.join(pkg_cstam_gazebo, 'models'),
            os.path.dirname(pkg_cstam_gazebo),
        ]),
    )

    xacro_file = os.path.join(pkg_cstam_gazebo, 'urdf', 'cstam_robot.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    world_path = PathJoinSubstitution([pkg_cstam_gazebo, 'worlds', world])
    gz_launch = os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')

    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={'gz_args': ['-r ', world_path], 'on_exit_shutdown': 'true'}.items(),
        condition=UnlessCondition(headless),
    )
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={'gz_args': ['-r -s --headless-rendering ', world_path],
                          'on_exit_shutdown': 'true'}.items(),
        condition=IfCondition(headless),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'cstam_robot',
            '-x', LaunchConfiguration('spawn_x'),
            '-y', LaunchConfiguration('spawn_y'),
            '-z', LaunchConfiguration('spawn_z'),
            '-Y', LaunchConfiguration('spawn_yaw'),
        ],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription(declare_args + [
        set_resource_path,
        gazebo_gui,
        gazebo_headless,
        robot_state_publisher,
        spawn_robot,
        bridge,
    ])
