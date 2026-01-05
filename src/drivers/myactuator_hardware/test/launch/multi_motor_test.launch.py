#!/usr/bin/env python3
"""
다중 모터 테스트 런치파일

MyActuator RMD 모터 4개를 연결하여 동작을 테스트합니다.
ros2_control을 사용하여 position 명령을 보내고 상태를 읽습니다.

사용법:
    ros2 launch myactuator_hardware multi_motor_test.launch.py
    ros2 launch myactuator_hardware multi_motor_test.launch.py can_interface:=can0

테스트 명령:
    # 조인트 상태 확인
    ros2 topic echo /joint_states

    # Forward Position Controller로 위치 명령 (4개 조인트)
    ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0, 0.0, 0.0, 0.0]"
    ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.5, 0.3, 0.2, 0.1]"

    # Joint Trajectory Controller로 위치 명령
    ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
        trajectory: {
            joint_names: [joint_1, joint_2, joint_3, joint_4],
            points: [
                {positions: [0.5, 0.3, 0.2, 0.1], time_from_start: {sec: 2}}
            ]
        }
    }"
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
            description='Motor 1 CAN ID'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'motor2_id',
            default_value='2',
            description='Motor 2 CAN ID'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'motor3_id',
            default_value='3',
            description='Motor 3 CAN ID'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'motor4_id',
            default_value='4',
            description='Motor 4 CAN ID'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_trajectory_controller',
            default_value='false',
            description='Use JointTrajectoryController instead of ForwardPositionController'
        )
    )

    # Get launch configurations
    can_interface = LaunchConfiguration('can_interface')
    motor1_id = LaunchConfiguration('motor1_id')
    motor2_id = LaunchConfiguration('motor2_id')
    motor3_id = LaunchConfiguration('motor3_id')
    motor4_id = LaunchConfiguration('motor4_id')
    use_trajectory_controller = LaunchConfiguration('use_trajectory_controller')

    # Get package share directory
    pkg_share = get_package_share_directory('myactuator_hardware')
    
    # Robot description (URDF)
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([pkg_share, 'config', 'multi_motor_test.urdf.xacro']),
        ' can_interface:=', can_interface,
        ' motor1_id:=', motor1_id,
        ' motor2_id:=', motor2_id,
        ' motor3_id:=', motor3_id,
        ' motor4_id:=', motor4_id,
    ])
    
    robot_description = {'robot_description': robot_description_content}

    # Controller configuration
    controller_config = PathJoinSubstitution([
        pkg_share, 'config', 'multi_motor_controllers.yaml'
    ])

    # ros2_control node
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_config],
        output='screen',
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # Joint state broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen',
    )

    # Forward position controller spawner
    forward_position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['forward_position_controller', '-c', '/controller_manager'],
        output='screen',
    )

    # Delay forward position controller after joint state broadcaster
    delay_forward_position_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[forward_position_controller_spawner],
        )
    )

    # RViz for visualization
    rviz_config_file = os.path.join(pkg_share, 'config', 'multi_motor_test.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
    )

    return LaunchDescription(
        declared_arguments + [
            control_node,
            robot_state_publisher,
            joint_state_broadcaster_spawner,
            delay_forward_position_controller,
            rviz_node,
        ]
    )
