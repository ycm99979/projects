"""
============================================================================
MD Motor Hardware Interface 테스트 런치 파일
============================================================================

이 런치 파일은 다음을 실행합니다:
1. robot_state_publisher (URDF 발행)
2. controller_manager (ros2_control 매니저)
3. diff_drive_controller (차동 구동 컨트롤러)
4. joint_state_broadcaster (조인트 상태 발행)

[사용법]
ros2 launch md_motor_hardware md_hardware_test.launch.py

[확인 방법]
# 컨트롤러 상태 확인
ros2 control list_controllers

# 토픽 확인
ros2 topic list

# cmd_vel 테스트
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}"

============================================================================
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 패키지 경로
    md_motor_hardware_pkg = get_package_share_directory('md_motor_hardware')
    
    # 파라미터
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # URDF 파일 경로
    urdf_file = os.path.join(md_motor_hardware_pkg, 'urdf', 'md_robot.urdf.xacro')
    
    # Controller 설정 파일
    controller_config = os.path.join(md_motor_hardware_pkg, 'config', 'md_controllers.yaml')

    # Robot Description (URDF → XML)
    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )
    robot_description = {'robot_description': robot_description_content}

    # =========================================================================
    # Nodes
    # =========================================================================
    
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}, {'use_sim_time': use_sim_time}],
    )

    # Controller Manager (ros2_control)
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            controller_config,
            {'robot_description': robot_description_content},
        ],
        output='screen',
    )

    # Joint State Broadcaster Spawner (Controller Manager 시작 후 5초 대기)
    joint_state_broadcaster_spawner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'joint_state_broadcaster',
                    '--controller-manager', '/controller_manager',
                    '--unload-on-kill',
                ],
                output='screen',
            )
        ]
    )

    # Diff Drive Controller Spawner (Controller Manager 시작 후 7초 대기)
    # --inactive 없이 바로 활성화, 기존 컨트롤러가 있으면 unload 후 재로드
    diff_drive_controller_spawner = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'diff_drive_controller',
                    '--controller-manager', '/controller_manager',
                ],
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        
        # Nodes
        robot_state_publisher_node,
        controller_manager_node,
        # joint_state_broadcaster_spawner,  # diff_drive_controller가 joint_states를 자체 발행
        diff_drive_controller_spawner,
    ])
