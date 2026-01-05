#!/bin/bash

# SLCAN 연결 및 CAN0 인터페이스 활성화 스크립트
# 사용법: ./setup_slcan.sh [device] [baudrate]

#ros2 launch realsense2_camera rs_multi_camera_launch.py \
#serial_no1:="_315122271488" \
#serial_no2:="_213622301251" \
#camera_name1:="d405" \
#camera_name2:="d455"

# 기본값 설정
DEVICE=${1:-/dev/ttyACM0}  # 기본 디바이스
BAUDRATE=${2:-1000000}     # 기본 보드레이트 (1Mbps)

echo "SLCAN 설정 시작..."
echo "디바이스: $DEVICE"
echo "보드레이트: $BAUDRATE"

# 기존 can0 인터페이스가 있다면 종료
if ip link show can0 &> /dev/null; then
    echo "기존 can0 인터페이스 종료 중..."
    sudo ip link set can0 down
    sudo slcand -k can0 2>/dev/null || true
fi

# slcand 프로세스 종료
echo "기존 slcand 프로세스 종료 중..."
sudo pkill slcand 2>/dev/null || true

# 잠시 대기
sleep 1

# SLCAN 연결 설정
echo "SLCAN 연결 설정 중..."
case $BAUDRATE in
    10000)   SLCAN_SPEED="0" ;;
    20000)   SLCAN_SPEED="1" ;;
    50000)   SLCAN_SPEED="2" ;;
    100000)  SLCAN_SPEED="3" ;;
    125000)  SLCAN_SPEED="4" ;;
    250000)  SLCAN_SPEED="5" ;;
    500000)  SLCAN_SPEED="6" ;;
    800000)  SLCAN_SPEED="7" ;;
    1000000) SLCAN_SPEED="8" ;;
    *)       SLCAN_SPEED="8"; echo "지원하지 않는 보드레이트, 1Mbps로 설정" ;;
esac

# slcand 시작
echo "slcand 시작 중..."
sudo slcand -o -c -f -s$SLCAN_SPEED $DEVICE can0

if [ $? -ne 0 ]; then
    echo "오류: slcand 시작 실패"
    echo "디바이스 $DEVICE 가 연결되어 있는지 확인하세요"
    exit 1
fi

# can0 인터페이스 활성화
echo "can0 인터페이스 활성화 중..."
sudo ip link set can0 up

if [ $? -ne 0 ]; then
    echo "오류: can0 인터페이스 활성화 실패"
    sudo slcand -k can0
    exit 1
fi

# 상태 확인
echo ""
echo "=== CAN 인터페이스 상태 ==="
ip link show can0

echo ""
echo "=== 설정 완료 ==="
echo "SLCAN이 성공적으로 연결되었습니다!"
echo "can0 인터페이스가 활성화되었습니다."
echo ""
echo "테스트 명령어:"
echo "  candump can0                    # CAN 메시지 모니터링"
echo "  cansend can0 123#DEADBEEF       # 테스트 메시지 전송"
echo ""
echo "종료하려면: sudo ./stop_slcan.sh"