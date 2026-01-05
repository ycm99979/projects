#!/bin/bash
#
# 4WD Mobile Base Teleop Script
# MD Motor Hardware Interface 텔레옵 테스트용
#

echo "╔══════════════════════════════════════════════════════════════╗"
echo "║                4WD Mobile Base Teleop                        ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""

# 1. 컨트롤러 상태 확인
echo "[1/3] Checking controller status..."
if ! ros2 control list_controllers 2>/dev/null | grep -q "diff_drive_controller.*active"; then
    echo "  ⚠ diff_drive_controller is not active!"
    echo ""
    echo "  Please run first:"
    echo "    ros2 launch md_motor_hardware md_4wd_test.launch.py"
    echo ""
    exit 1
fi

echo "  ✓ diff_drive_controller is active"
echo ""

# 2. 토픽 확인
echo "[2/3] Checking topics..."
TOPIC="/diff_drive_controller/cmd_vel_unstamped"
if ! ros2 topic list 2>/dev/null | grep -q "$TOPIC"; then
    echo "  ✗ Topic $TOPIC not found!"
    echo ""
    echo "  Available topics:"
    ros2 topic list | grep cmd_vel || echo "    No cmd_vel topics found"
    echo ""
    exit 1
fi

echo "  ✓ Topic $TOPIC is available"
echo ""

# 3. 텔레옵 실행
echo "[3/3] Starting teleop..."
echo ""
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║                    TELEOP CONTROLS                           ║"
echo "╠══════════════════════════════════════════════════════════════╣"
echo "║  Moving around:                                              ║"
echo "║     u    i    o                                              ║"
echo "║     j    k    l                                              ║"
echo "║     m    ,    .                                              ║"
echo "║                                                              ║"
echo "║  q/z : increase/decrease max speeds by 10%                   ║"
echo "║  w/x : increase/decrease only linear speed by 10%            ║"
echo "║  e/c : increase/decrease only angular speed by 10%           ║"
echo "║  space key, k : force stop                                   ║"
echo "║  CTRL-C to quit                                              ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""
echo "Press any key to start teleop..."
read -n 1 -s
echo ""

# teleop_twist_keyboard 실행
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
    --remap cmd_vel:=$TOPIC

echo ""
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║                    Teleop Stopped                            ║"
echo "╚══════════════════════════════════════════════════════════════╝"