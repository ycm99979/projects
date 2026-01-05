#!/usr/bin/env python3
"""
============================================================================
Motor Home Position Calibration Launch File (USBCAN Version)
============================================================================

USBCAN-II 모듈을 사용하여 모터의 현재 위치를 홈(0) 위치로 설정하는 런치파일

[사용법]
1. 전원 OFF 상태에서 로봇팔을 원하는 초기 위치로 수동으로 이동
2. 전원 ON
3. 이 런치파일 실행:
   ros2 launch robot_bringup calibrate_home.launch.py

4. 캘리브레이션 완료 후 mm_moveit_hardware.launch.py 실행

[파라미터]
- can_baudrate: CAN 보드레이트 (기본값: 1000000)
- motor_ids: 캘리브레이션할 모터 ID (기본값: 1,2,3,4)
- can_channel: CAN 채널 번호 (기본값: 0)

[주의]
- 이 런치파일은 현재 모터 위치를 0으로 설정합니다
- 캘리브레이션 후에는 모터를 재시작해야 적용됩니다

============================================================================
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
)
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # ========================================================================
    # Launch Arguments
    # ========================================================================
    declared_arguments = [
        DeclareLaunchArgument(
            "can_baudrate",
            default_value="1000000",
            description="CAN baudrate (1000000, 500000, 250000, 125000)"
        ),
        DeclareLaunchArgument(
            "motor_ids",
            default_value="1,2,3,4",
            description="Motor IDs to calibrate (comma-separated)"
        ),
        DeclareLaunchArgument(
            "can_channel",
            default_value="0",
            description="CAN channel (0 or 1)"
        ),
    ]

    can_baudrate = LaunchConfiguration("can_baudrate")
    motor_ids = LaunchConfiguration("motor_ids")
    can_channel = LaunchConfiguration("can_channel")

    # ========================================================================
    # Workspace path
    # ========================================================================
    import subprocess
    result = subprocess.run(['ros2', 'pkg', 'prefix', 'robot_bringup'], capture_output=True, text=True)
    pkg_prefix = result.stdout.strip() if result.returncode == 0 else ""

    if pkg_prefix:
        workspace_path = os.path.dirname(os.path.dirname(pkg_prefix))
    else:
        workspace_path = os.path.expanduser("~/frbot_ws")

    calibrate_script = os.path.join(workspace_path, "scripts", "calibrate_home_usbcan.py")

    # ========================================================================
    # Log messages
    # ========================================================================
    log_start = LogInfo(msg="""
================================================================================
  MOTOR HOME POSITION CALIBRATION (USBCAN)
================================================================================

이 스크립트는 USBCAN-II를 사용하여 현재 모터 위치를 홈(0) 위치로 설정합니다.

[주의사항]
1. 전원 OFF 상태에서 로봇팔을 원하는 초기 위치로 수동으로 이동하세요
2. 전원을 켜고 이 스크립트를 실행하세요
3. 캘리브레이션이 완료되면 모터의 현재 위치가 0으로 설정됩니다
4. 캘리브레이션 후 모터 전원을 껐다 켜야 적용됩니다!

================================================================================
""")

    # ========================================================================
    # Calibration Process (USBCAN)
    # ========================================================================
    calibrate_process = ExecuteProcess(
        cmd=[
            'python3', calibrate_script,
            '--baudrate', can_baudrate,
            '--motor-ids', motor_ids,
            '--channel', can_channel,
        ],
        name='calibrate_home_usbcan',
        output='screen',
    )

    # ========================================================================
    # Launch Description
    # ========================================================================
    return LaunchDescription(
        declared_arguments + [
            log_start,
            calibrate_process,
        ]
    )
