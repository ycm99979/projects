#!/usr/bin/env python3
"""
Web Interface Launch File

rosbridge_server와 web_video_server를 실행하고
웹 인터페이스를 제공합니다.

사용법:
    ros2 launch robot_web_interface web_interface.launch.py

웹 브라우저에서 접속:
    http://<robot_ip>:8000
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('robot_web_interface')
    www_dir = os.path.join(pkg_dir, 'www')

    # rosbridge_server (WebSocket on port 9090)
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{
            'port': 9090,
            'address': '',
            'retry_startup_delay': 5.0,
        }],
        output='screen',
    )

    # web_video_server (HTTP streaming on port 8080)
    web_video_node = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        parameters=[{
            'port': 8080,
            'address': '0.0.0.0',
        }],
        output='screen',
    )

    # Simple HTTP server for web interface (port 8000)
    http_server = ExecuteProcess(
        cmd=['python3', '-m', 'http.server', '8000', '--directory', www_dir],
        output='screen',
    )

    return LaunchDescription([
        rosbridge_node,
        web_video_node,
        TimerAction(
            period=2.0,
            actions=[http_server],
        ),
    ])
