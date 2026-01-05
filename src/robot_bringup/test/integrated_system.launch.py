#!/usr/bin/env python3
"""
============================================================================
Integrated System Launch File
============================================================================

모든 시스템 컴포넌트를 하나의 런치 파일로 통합:
- Mobile Manipulator MoveIt (하드웨어 제어)
- Dual RealSense 카메라 (D405 + D455)
- YOLO 객체 인식
- Web Interface (rosbridge + video server)
- Navigation (옵션)

[사용법]
ros2 launch robot_bringup integrated_system.launch.py

[파라미터]
- enable_cameras: 카메라 활성화 (default: true)
- enable_web: 웹 인터페이스 활성화 (default: true)
- enable_navigation: 네비게이션 활성화 (default: false)
- enable_yolo: YOLO 객체 인식 활성화 (default: true)
- rviz: RViz 실행 여부 (default: true)

============================================================================
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription, 
    TimerAction,
    GroupAction
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package paths
    robot_bringup_pkg = get_package_share_directory("robot_bringup")
    robot_web_interface_pkg = get_package_share_directory("robot_web_interface")

    # ================================================================
    # Launch Arguments
    # ================================================================
    declared_arguments = [
        # System components
        DeclareLaunchArgument("enable_cameras", default_value="true",
                              description="Enable dual RealSense cameras"),
        DeclareLaunchArgument("enable_web", default_value="true",
                              description="Enable web interface"),
        DeclareLaunchArgument("enable_navigation", default_value="false",
                              description="Enable navigation stack"),
        DeclareLaunchArgument("enable_yolo", default_value="true",
                              description="Enable YOLO object detection"),
        
        # Hardware parameters (passed to mobile_manipulator_moveit.launch.py)
        DeclareLaunchArgument("can_channel", default_value="0",
                              description="USBCAN channel (0 or 1)"),
        DeclareLaunchArgument("baudrate", default_value="1000000",
                              description="CAN baudrate"),
        DeclareLaunchArgument("gripper_port", default_value="/dev/ttyACM0",
                              description="Serial port for gripper"),
        DeclareLaunchArgument("port_front", default_value="/dev/ttyUSB0",
                              description="Serial port for front motor driver"),
        DeclareLaunchArgument("port_rear", default_value="/dev/ttyUSB1",
                              description="Serial port for rear motor driver"),
        
        # General
        DeclareLaunchArgument("rviz", default_value="true",
                              description="Launch RViz"),
        DeclareLaunchArgument("use_sim_time", default_value="false",
                              description="Use simulation time"),
    ]

    # Get launch configurations
    enable_cameras = LaunchConfiguration("enable_cameras")
    enable_web = LaunchConfiguration("enable_web")
    enable_navigation = LaunchConfiguration("enable_navigation")
    enable_yolo = LaunchConfiguration("enable_yolo")
    
    can_channel = LaunchConfiguration("can_channel")
    baudrate = LaunchConfiguration("baudrate")
    gripper_port = LaunchConfiguration("gripper_port")
    port_front = LaunchConfiguration("port_front")
    port_rear = LaunchConfiguration("port_rear")
    rviz_arg = LaunchConfiguration("rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # ================================================================
    # Core System: Mobile Manipulator MoveIt
    # ================================================================
    mobile_manipulator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_bringup_pkg, "launch", "mobile_manipulator_moveit.launch.py")
        ),
        launch_arguments={
            "can_channel": can_channel,
            "baudrate": baudrate,
            "gripper_port": gripper_port,
            "port_front": port_front,
            "port_rear": port_rear,
            "rviz": rviz_arg,
            "use_sim_time": use_sim_time,
        }.items()
    )

    # ================================================================
    # Camera System: Dual RealSense
    # ================================================================
    cameras_group = GroupAction(
        condition=IfCondition(enable_cameras),
        actions=[
            TimerAction(
                period=5.0,  # MoveIt 초기화 후 카메라 시작
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(robot_bringup_pkg, "launch", "dual_realsense.launch.py")
                        )
                    )
                ]
            )
        ]
    )

    # ================================================================
    # Web Interface
    # ================================================================
    web_interface_group = GroupAction(
        condition=IfCondition(enable_web),
        actions=[
            TimerAction(
                period=8.0,  # 카메라 초기화 후 웹 인터페이스 시작
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(robot_web_interface_pkg, "launch", "web_interface.launch.py")
                        )
                    )
                ]
            )
        ]
    )

    # ================================================================
    # Navigation (Optional)
    # ================================================================
    navigation_group = GroupAction(
        condition=IfCondition(enable_navigation),
        actions=[
            TimerAction(
                period=10.0,  # 모든 시스템 초기화 후 네비게이션 시작
                actions=[
                    # Navigation launch file would go here
                    # IncludeLaunchDescription(...)
                ]
            )
        ]
    )

    # ================================================================
    # Launch Description
    # ================================================================
    return LaunchDescription(
        declared_arguments + [
            # Core system first
            mobile_manipulator_launch,
            
            # Then cameras
            cameras_group,
            
            # Then web interface
            web_interface_group,
            
            # Finally navigation (if enabled)
            navigation_group,
        ]
    )