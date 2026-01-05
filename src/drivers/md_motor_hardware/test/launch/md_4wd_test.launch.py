#!/usr/bin/env python3
"""
============================================================================
MD Motor 4WD Hardware Interface Test Launch File (Dual Serial Port)
============================================================================

2개의 USB-RS485 변환기를 사용하는 4WD 테스트용 런치 파일
- Front Driver (ttyUSB0): FL, FR 바퀴
- Rear Driver (ttyUSB1): RL, RR 바퀴

robot_description의 mobile_base.xacro 기반 파라미터:
- wheel_radius: 0.098m
- wheel_separation: 0.32m (좌우 휠 간격)
- wheelbase: 0.44m (전후 휠 간격)

[사용법]
ros2 launch md_motor_hardware md_4wd_test.launch.py
ros2 launch md_motor_hardware md_4wd_test.launch.py port_front:=/dev/ttyUSB0 port_rear:=/dev/ttyUSB1

[텔레옵]
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
    --remap cmd_vel:=/diff_drive_controller/cmd_vel_unstamped

[확인 방법]
ros2 control list_controllers
ros2 topic echo /diff_drive_controller/odom

============================================================================
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('md_motor_hardware')

    # Launch Arguments - 실제 로봇 파라미터 (robot_description/mobile_base.xacro 기준)
    declared_arguments = [
        # 시리얼 포트
        DeclareLaunchArgument('port_front', default_value='/dev/ttyUSB0',
                              description='Serial port for front driver (FL, FR)'),
        DeclareLaunchArgument('port_rear', default_value='/dev/ttyUSB1',
                              description='Serial port for rear driver (RL, RR)'),
        DeclareLaunchArgument('baudrate', default_value='19200'),
        
        # 모터 드라이버 ID
        DeclareLaunchArgument('front_driver_id', default_value='1'),
        DeclareLaunchArgument('rear_driver_id', default_value='2'),
        DeclareLaunchArgument('id_mdui', default_value='184'),
        DeclareLaunchArgument('id_mdt', default_value='183'),
        
        # 로봇 기구학 파라미터 (mobile_base.xacro 기준)
        DeclareLaunchArgument('wheel_radius', default_value='0.098',
                              description='Wheel radius in meters'),
        DeclareLaunchArgument('wheel_separation', default_value='0.32',
                              description='Distance between left and right wheels'),
        DeclareLaunchArgument('wheelbase', default_value='0.44',
                              description='Distance between front and rear wheels'),
        
        # 모터/엔코더 파라미터
        DeclareLaunchArgument('gear_ratio', default_value='15'),
        DeclareLaunchArgument('poles', default_value='10'),
        
        DeclareLaunchArgument('use_sim_time', default_value='false'),
    ]

    # URDF - md_4wd_robot.urdf.xacro 사용
    urdf_file = os.path.join(pkg_share, 'urdf', 'md_4wd_robot.urdf.xacro')
    robot_description_content = ParameterValue(
        Command([
            'xacro ', urdf_file,
            ' port_front:=', LaunchConfiguration('port_front'),
            ' port_rear:=', LaunchConfiguration('port_rear'),
            ' baudrate:=', LaunchConfiguration('baudrate'),
            ' front_driver_id:=', LaunchConfiguration('front_driver_id'),
            ' rear_driver_id:=', LaunchConfiguration('rear_driver_id'),
            ' id_mdui:=', LaunchConfiguration('id_mdui'),
            ' id_mdt:=', LaunchConfiguration('id_mdt'),
            ' wheel_radius:=', LaunchConfiguration('wheel_radius'),
            ' wheel_separation:=', LaunchConfiguration('wheel_separation'),
            ' wheelbase:=', LaunchConfiguration('wheelbase'),
            ' gear_ratio:=', LaunchConfiguration('gear_ratio'),
            ' poles:=', LaunchConfiguration('poles'),
        ]),
        value_type=str
    )

    # Controller config
    controller_config = os.path.join(pkg_share, 'config', 'md_4wd_controllers.yaml')

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description_content},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    # Controller Manager
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            controller_config,
            {'robot_description': robot_description_content},
        ],
        output='screen',
    )

    # Joint State Broadcaster Spawner (5초 후)
    joint_state_broadcaster_spawner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
                output='screen',
            )
        ]
    )

    # Diff Drive Controller Spawner (7초 후)
    diff_drive_controller_spawner = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['diff_drive_controller', '-c', '/controller_manager'],
                output='screen',
            )
        ]
    )

    return LaunchDescription(
        declared_arguments + [
            robot_state_publisher_node,
            controller_manager_node,
            joint_state_broadcaster_spawner,
            diff_drive_controller_spawner,
        ]
    )
