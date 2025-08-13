#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Package Directories
    pkg_dir = FindPackageShare('ouster_gazebo').find('ouster_gazebo')
    pkg_gazebo_ros = FindPackageShare('gazebo_ros').find('gazebo_ros')

    # Paths to directories and files
    gazebo_models_path = os.path.join(pkg_dir, 'meshes')
    world_file_path = os.path.join(pkg_dir, 'worlds', 'empty.world')
    urdf_file_path = os.path.join(pkg_dir, 'urdf', 'skid_steer_robot.urdf.xacro')

    # Set environment variable for Gazebo model path
    env = {'GAZEBO_MODEL_PATH': gazebo_models_path}

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_file_path,
        description='Full path to world file to load'
    )

    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='skid_steer_robot',
        description='Name of the robot'
    )

    x_pos_arg = DeclareLaunchArgument(
        'x_pos',
        default_value='0.0',
        description='Initial x position of the robot'
    )

    y_pos_arg = DeclareLaunchArgument(
        'y_pos',
        default_value='0.0',
        description='Initial y position of the robot'
    )

    z_pos_arg = DeclareLaunchArgument(
        'z_pos',
        default_value='0.5',
        description='Initial z position of the robot'
    )

    # Start Gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    # Start Gazebo client
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
    )

    # Robot State Publisher
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': Command(['xacro ', urdf_file_path])
        }]
    )

    # Spawn robot in Gazebo
    spawn_robot_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', LaunchConfiguration('robot_name'),
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('x_pos'),
            '-y', LaunchConfiguration('y_pos'),
            '-z', LaunchConfiguration('z_pos')
        ],
        output='screen'
    )

    # Joint State Publisher
    joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare launch arguments
    ld.add_action(use_sim_time_arg)
    ld.add_action(world_arg)
    ld.add_action(robot_name_arg)
    ld.add_action(x_pos_arg)
    ld.add_action(y_pos_arg)
    ld.add_action(z_pos_arg)

    # Add launch actions
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_robot_cmd)
    ld.add_action(joint_state_publisher_cmd)

    return ld
