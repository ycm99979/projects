#!/usr/bin/env python3
"""
듀얼 모터 연결 테스트 런치파일

MyActuator RMD 모터 2개를 연결하여 기본 동작을 테스트합니다.
ros2_control을 사용하여 position 명령을 보내고 상태를 읽습니다.

사용법:
    ros2 launch myactuator_hardware dual_motor_test.launch.py
    ros2 launch myactuator_hardware dual_motor_test.launch.py motor1_id:=1 motor2_id:=2
    ros2 launch myactuator_hardware dual_motor_test.launch.py can_interface:=can0

테스트 명령:
    # 조인트 상태 확인
    ros2 topic echo /joint_states

    # 위치 명령 보내기 (라디안) - 2개 모터 동시에
    ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.5, -0.5]"
    
    # 모터 1만 움직이기 (모터 2는 0)
    ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [1.0, 0.0]"
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch arguments
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'can_interface',
            default_value='can0',
            description='CAN interface name'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'motor1_id',
            default_value='1',
            description='Motor 1 CAN ID (1-32)'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'motor2_id',
            default_value='2',
            description='Motor 2 CAN ID (1-32)'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'update_rate',
            default_value='100',
            description='Controller update rate in Hz'
        )
    )

    # Get launch configurations
    can_interface = LaunchConfiguration('can_interface')
    motor1_id = LaunchConfiguration('motor1_id')
    motor2_id = LaunchConfiguration('motor2_id')

    # Get package share directory
    pkg_share = get_package_share_directory('myactuator_hardware')
    
    # Robot description (URDF)
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([pkg_share, 'config', 'dual_motor_test.urdf.xacro']),
        ' can_interface:=', can_interface,
        ' motor1_id:=', motor1_id,
        ' motor2_id:=', motor2_id,
    ])
    
    robot_description = {'robot_description': robot_description_content}

    # Controller configuration
    controller_config = PathJoinSubstitution([
        pkg_share, 'config', 'dual_motor_controllers.yaml'
    ])

    # ros2_control node
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_config],
        output='screen',
    )

    # Robot state publisher
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # Joint state broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Joint trajectory controller spawner (for MoveIt and teleop)
    trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Delay controller spawners until joint_state_broadcaster is active
    delay_trajectory_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[trajectory_controller_spawner],
        )
    )

    return LaunchDescription(
        declared_arguments + [
            control_node,
            robot_state_pub_node,
            joint_state_broadcaster_spawner,
            delay_trajectory_controller,
        ]
    )
