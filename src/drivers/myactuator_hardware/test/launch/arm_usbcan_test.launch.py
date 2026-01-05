#!/usr/bin/env python3
"""
USBCAN-UC12 기반 팔 테스트 런치파일

MyActuator RMD 모터 4개를 USBCAN-UC12 어댑터를 통해 제어합니다.
SocketCAN 없이 libusb를 직접 사용합니다.

사용법:
    ros2 launch myactuator_hardware arm_usbcan_test.launch.py
    ros2 launch myactuator_hardware arm_usbcan_test.launch.py can_channel:=0 baudrate:=1000000

테스트 명령:
    # 조인트 상태 확인
    ros2 topic echo /joint_states

    # Forward Position Controller로 위치 명령 (4개 조인트)
    ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0, 0.0, 0.0, 0.0]"
    ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.5, 0.3, 0.2, 0.1]"

    # Joint Trajectory Controller로 위치 명령
    ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
        trajectory: {
            joint_names: [link2_to_link1, link3_to_link2, link4_to_link3, gripper_to_link4],
            points: [
                {positions: [0.5, 0.3, 0.2, 0.1], time_from_start: {sec: 2}}
            ]
        }
    }"
"""

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'can_channel',
            default_value='0',
            description='USBCAN channel (0 or 1)'
        ),
        DeclareLaunchArgument(
            'baudrate',
            default_value='1000000',
            description='CAN baudrate (1000000, 500000, 250000, 125000)'
        ),
        DeclareLaunchArgument(
            'motor1_id',
            default_value='1',
            description='Motor 1 CAN ID'
        ),
        DeclareLaunchArgument(
            'motor2_id',
            default_value='2',
            description='Motor 2 CAN ID'
        ),
        DeclareLaunchArgument(
            'motor3_id',
            default_value='3',
            description='Motor 3 CAN ID'
        ),
        DeclareLaunchArgument(
            'motor4_id',
            default_value='4',
            description='Motor 4 CAN ID'
        ),
        DeclareLaunchArgument(
            'auto_home',
            default_value='true',
            description='Auto home on activation'
        ),
        DeclareLaunchArgument(
            'gripper_port',
            default_value='none',
            description='Gripper serial port (none to disable)'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz'
        ),
    ]

    # Get launch configurations
    can_channel = LaunchConfiguration('can_channel')
    baudrate = LaunchConfiguration('baudrate')
    motor1_id = LaunchConfiguration('motor1_id')
    motor2_id = LaunchConfiguration('motor2_id')
    motor3_id = LaunchConfiguration('motor3_id')
    motor4_id = LaunchConfiguration('motor4_id')
    auto_home = LaunchConfiguration('auto_home')
    gripper_port = LaunchConfiguration('gripper_port')
    rviz = LaunchConfiguration('rviz')

    # Package paths
    robot_description_pkg = FindPackageShare('robot_description')
    myactuator_hardware_pkg = FindPackageShare('myactuator_hardware')

    # Robot description (URDF)
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([robot_description_pkg, 'urdf', 'arm_usbcan_hardware.xacro']),
        ' can_channel:=', can_channel,
        ' baudrate:=', baudrate,
        ' motor1_id:=', motor1_id,
        ' motor2_id:=', motor2_id,
        ' motor3_id:=', motor3_id,
        ' motor4_id:=', motor4_id,
        ' auto_home:=', auto_home,
        ' gripper_port:=', gripper_port,
    ])

    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # Controller configuration
    controller_config = PathJoinSubstitution([
        myactuator_hardware_pkg, 'config', 'arm_usbcan_controllers.yaml'
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

    # RViz
    rviz_config = PathJoinSubstitution([robot_description_pkg, 'rviz', 'frbot.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config],
        condition=IfCondition(rviz),
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
