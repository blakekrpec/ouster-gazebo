from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_ouster_gazebo = FindPackageShare('ouster_gazebo').find('ouster_gazebo')

    urdf_path = os.path.join(pkg_ouster_gazebo, 'urdf', 'skid_steer_robot.xacro')
    config_path = os.path.join(pkg_ouster_gazebo, 'config', 'ouster_puck.yaml')

    robot_description = Command([
        'xacro',
        urdf_path,
        'ouster_config:=',
        config_path
    ])

    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', 'empty.world'],
        output='screen'
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            os.path.join(pkg_ouster_gazebo, 'config', 'skid_steer_ros2_control.yaml')
        ],
        output='screen'
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'skid_steer_robot'],
        output='screen'
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    load_skid_steer_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'skid_steer_controller'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        rsp,
        controller_manager,
        spawn_entity,
        load_joint_state_broadcaster,
        load_skid_steer_controller
    ])
