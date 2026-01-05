#!/usr/bin/env python3
"""
============================================================================
Full System Launch File (Simple Version)
============================================================================

현재 5개 터미널에서 실행하던 모든 컴포넌트를 하나로 통합:
1. Mobile Manipulator MoveIt + Hardware
2. Dual RealSense Cameras + YOLO
3. Web Interface (rosbridge + video server)

[사용법]
ros2 launch robot_bringup full_system.launch.py

[주요 파라미터]
- rviz: RViz 실행 여부 (default: true)
- can_channel: USBCAN 채널 (default: 0)
- gripper_port: 그리퍼 포트 (default: /dev/ttyACM0)

============================================================================
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package paths
    robot_bringup_pkg = get_package_share_directory("robot_bringup")
    robot_web_interface_pkg = get_package_share_directory("robot_web_interface")

    # ================================================================
    # Launch Arguments (주요 파라미터만)
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
    ]

    # Get configurations
    rviz_arg = LaunchConfiguration("rviz")
    can_channel = LaunchConfiguration("can_channel")
    gripper_port = LaunchConfiguration("gripper_port")
    port_front = LaunchConfiguration("port_front")
    port_rear = LaunchConfiguration("port_rear")

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
    # 2. Dual RealSense + YOLO (5초 후 시작)
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
    # 3. Web Interface (8초 후 시작)
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
    # Launch Description
    # ================================================================
    return LaunchDescription(
        declared_arguments + [
            mobile_manipulator,    # 즉시 시작
            cameras_and_yolo,      # 5초 후
            web_interface,         # 8초 후
        ]
    )