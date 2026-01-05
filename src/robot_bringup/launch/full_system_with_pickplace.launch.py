#!/usr/bin/env python3
"""
============================================================================
Full System Launch File with Pick & Place Controller
============================================================================

모든 컴포넌트 + Pick & Place 컨트롤러를 자동으로 실행:
1. Mobile Manipulator MoveIt + Hardware
2. Robot Localization EKF (센서 융합)
3. Dual RealSense Cameras + YOLO
4. Web Interface (rosbridge + video server)
5. Pick & Place Controller (topic_pick_place.py)

[사용법]
ros2 launch robot_bringup full_system_with_pickplace.launch.py

[Pick & Place 사용법]
- YOLO가 객체를 감지하면 /target_point 토픽으로 좌표 발행
- Pick & Place 컨트롤러가 자동으로 동작 수행
- 또는 수동으로 좌표 발행:
  ros2 topic pub /target_point geometry_msgs/PointStamped "..."

[TF 구조]
map → odom (SLAM/AMCL)
odom → base_link (robot_localization EKF)
base_link → ... (robot_state_publisher)

============================================================================
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package paths
    robot_bringup_pkg = get_package_share_directory("robot_bringup")
    robot_web_interface_pkg = get_package_share_directory("robot_web_interface")

    # EKF config path
    ekf_config_path = os.path.join(robot_bringup_pkg, 'config', 'ekf.yaml')

    # ================================================================
    # Launch Arguments
    # ================================================================
    declared_arguments = [
        DeclareLaunchArgument("rviz", default_value="true",
                              description="Launch RViz"),
        DeclareLaunchArgument("can_channel", default_value="0",
                              description="USBCAN channel"),
        DeclareLaunchArgument("gripper_port", default_value="/dev/ttyACM0",
                              description="Gripper serial port"),
        DeclareLaunchArgument("port_front", default_value="/dev/ttyUSB0",
                              description="Front motor driver port"),
        DeclareLaunchArgument("port_rear", default_value="/dev/ttyUSB1",
                              description="Rear motor driver port"),
        DeclareLaunchArgument("enable_pickplace", default_value="true",
                              description="Enable pick and place controller"),
    ]

    # Get configurations
    rviz_arg = LaunchConfiguration("rviz")
    can_channel = LaunchConfiguration("can_channel")
    gripper_port = LaunchConfiguration("gripper_port")
    port_front = LaunchConfiguration("port_front")
    port_rear = LaunchConfiguration("port_rear")
    enable_pickplace = LaunchConfiguration("enable_pickplace")

    # ================================================================
    # 1. Mobile Manipulator MoveIt + Hardware (즉시 시작)
    # ================================================================
    mobile_manipulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_bringup_pkg, "launch", "mobile_manipulator_moveit.launch.py")
        ),
        launch_arguments={
            "rviz": rviz_arg,
            "can_channel": can_channel,
            "gripper_port": gripper_port,
            "port_front": port_front,
            "port_rear": port_rear,
        }.items()
    )

    # ================================================================
    # 2. Robot Localization EKF (3초 후 시작 - diff_drive 준비 후)
    # ================================================================
    ekf_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                parameters=[ekf_config_path],
            )
        ]
    )

    # ================================================================
    # 3. Dual RealSense + YOLO (5초 후 시작)
    # ================================================================
    cameras_and_yolo = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(robot_bringup_pkg, "launch", "dual_realsense.launch.py")
                )
            )
        ]
    )

    # ================================================================
    # 4. Web Interface (8초 후 시작)
    # ================================================================
    web_interface = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(robot_web_interface_pkg, "launch", "web_interface.launch.py")
                )
            )
        ]
    )

    # ================================================================
    # 5. Pick & Place Controller (12초 후 시작)
    # ================================================================
    pick_place_controller = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='robot_bringup',
                executable='topic_pick_place.py',
                name='topic_pick_place_controller',
                output='screen',
                parameters=[{
                    'approach_height': 0.08,
                    'retreat_height': 0.12,
                    'gripper_open_position': 0.019,
                    'gripper_close_position': -0.01,
                    'gripper_max_effort': 50.0,
                    'camera_frame': 'd405_optical_frame',
                    'base_frame': 'base_link',
                    'planning_group': 'arm',
                    'end_effector_link': 'gripper_base',
                    'place_x': 0.0,
                    'place_y': 0.25,
                    'place_z': 0.15,
                }],
                condition=IfCondition(enable_pickplace)
            )
        ]
    )

    # ================================================================
    # Launch Description
    # ================================================================
    return LaunchDescription(
        declared_arguments + [
            mobile_manipulator,    # 즉시 시작
            ekf_node,              # 3초 후 (diff_drive 준비 후)
            cameras_and_yolo,      # 5초 후
            web_interface,         # 8초 후
            pick_place_controller, # 12초 후 (모든 시스템 준비 후)
        ]
    )