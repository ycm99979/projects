#!/bin/bash
# ============================================================================
# Motor Home Position Calibration Script
# ============================================================================
#
# 현재 모터 위치를 홈(0) 위치로 설정하는 스크립트
#
# 사용법:
#   ./calibrate_home.sh [can_interface]
#
# MyActuator RMD 모터의 영점 설정 명령:
#   0x64: Write current multi-turn position to ROM as zero point
#         (현재 위치를 영점으로 ROM에 저장)
#
# 주의: 캘리브레이션 후 모터 전원을 껐다 켜야 적용됩니다!
#
# ============================================================================

CAN_IF=${1:-can0}

echo ""
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║       Motor Home Position Calibration                        ║"
echo "╠══════════════════════════════════════════════════════════════╣"
echo "║  현재 모터 위치를 영점(0)으로 ROM에 저장합니다               ║"
echo "║                                                              ║"
echo "║  ⚠️  주의: 캘리브레이션 후 모터 전원을 껐다 켜야 적용됩니다!  ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""
echo "CAN Interface: $CAN_IF"
echo ""

# CAN 인터페이스 확인
if ! ip link show $CAN_IF &> /dev/null; then
    echo "❌ 오류: $CAN_IF 인터페이스가 없습니다."
    echo "   먼저 SLCAN을 설정하세요: sudo ./scripts/setup_slcan.sh"
    exit 1
fi

# 모터 ID 배열 (1~4)
MOTOR_IDS=(1 2 3 4)

echo "========================================"
echo "  Step 1: 현재 모터 위치 확인"
echo "========================================"
echo ""

TMPFILE=$(mktemp)

# candump 시작
timeout 3s candump $CAN_IF > $TMPFILE &
CANDUMP_PID=$!
sleep 0.5

# 각 모터의 현재 위치 읽기 (0x92: Read multi-turn angle)
for ID in "${MOTOR_IDS[@]}"; do
    CAN_ID=$(printf "%03X" $((0x140 + ID)))
    cansend $CAN_IF ${CAN_ID}#9200000000000000 2>/dev/null
    sleep 0.2
done

wait $CANDUMP_PID 2>/dev/null

echo "현재 모터 위치 (캘리브레이션 전):"
for ID in "${MOTOR_IDS[@]}"; do
    CAN_ID=$(printf "%03X" $((0x140 + ID)))
    RESPONSE_ID=$(printf "%03X" $((0x240 + ID)))
    RESPONSE=$(grep -E "$CAN_IF[[:space:]]+$RESPONSE_ID" $TMPFILE | grep "92" | tail -1)
    
    if [ ! -z "$RESPONSE" ]; then
        # 응답 데이터에서 각도 추출 (바이트 1-7)
        # 형식: 92 XX XX XX XX XX XX XX (바이트 4-7이 각도, 0.01도 단위)
        DATA=$(echo "$RESPONSE" | awk '{for(i=4;i<=NF;i++) printf $i" "}')
        echo "  ✅ 모터 $ID (0x$CAN_ID): 응답 있음 - $DATA"
    else
        echo "  ❌ 모터 $ID (0x$CAN_ID): 응답 없음"
    fi
done

rm -f $TMPFILE

echo ""
echo "========================================"
echo "  Step 2: 영점 설정 중..."
echo "========================================"
echo ""
echo "  명령: 0x64 (Write current position to ROM as zero)"
echo ""

TMPFILE2=$(mktemp)

# candump 시작 (응답 확인용)
timeout 5s candump $CAN_IF > $TMPFILE2 &
CANDUMP_PID=$!
sleep 0.3

# 0x64: Write current multi-turn position to ROM as zero point
# 명령 형식: [0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
for ID in "${MOTOR_IDS[@]}"; do
    CAN_ID=$(printf "%03X" $((0x140 + ID)))
    RESPONSE_ID=$(printf "%03X" $((0x240 + ID)))
    
    echo "  모터 $ID (0x$CAN_ID): 영점 설정 명령 전송..."
    
    # 영점 설정 명령 전송 (0x64)
    cansend $CAN_IF ${CAN_ID}#6400000000000000 2>/dev/null
    
    if [ $? -eq 0 ]; then
        echo "    → 명령 전송 완료"
    else
        echo "    ❌ 명령 전송 실패"
    fi
    
    sleep 0.5
done

wait $CANDUMP_PID 2>/dev/null

# 응답 확인
echo ""
echo "  응답 확인:"
for ID in "${MOTOR_IDS[@]}"; do
    RESPONSE_ID=$(printf "%03X" $((0x240 + ID)))
    RESPONSE=$(grep -E "$CAN_IF[[:space:]]+$RESPONSE_ID" $TMPFILE2 | grep "64" | tail -1)
    
    if [ ! -z "$RESPONSE" ]; then
        DATA=$(echo "$RESPONSE" | awk '{for(i=4;i<=NF;i++) printf $i" "}')
        echo "    ✅ 모터 $ID: 응답 수신 - $DATA"
    else
        echo "    ⚠️  모터 $ID: 응답 없음 (명령은 전송됨)"
    fi
done

rm -f $TMPFILE2

echo ""
echo "========================================"
echo "  Step 3: 영점 설정 확인"
echo "========================================"
echo ""

# 잠시 대기 후 위치 다시 확인
sleep 1

TMPFILE3=$(mktemp)
timeout 3s candump $CAN_IF > $TMPFILE3 &
CANDUMP_PID=$!
sleep 0.5

for ID in "${MOTOR_IDS[@]}"; do
    CAN_ID=$(printf "%03X" $((0x140 + ID)))
    cansend $CAN_IF ${CAN_ID}#9200000000000000 2>/dev/null
    sleep 0.2
done

wait $CANDUMP_PID 2>/dev/null

echo "영점 설정 후 모터 위치:"
for ID in "${MOTOR_IDS[@]}"; do
    CAN_ID=$(printf "%03X" $((0x140 + ID)))
    RESPONSE_ID=$(printf "%03X" $((0x240 + ID)))
    RESPONSE=$(grep -E "$CAN_IF[[:space:]]+$RESPONSE_ID" $TMPFILE3 | grep "92" | tail -1)
    
    if [ ! -z "$RESPONSE" ]; then
        DATA=$(echo "$RESPONSE" | awk '{for(i=4;i<=NF;i++) printf $i" "}')
        echo "  ✅ 모터 $ID: $DATA"
    else
        echo "  ❌ 모터 $ID: 확인 실패"
    fi
done

rm -f $TMPFILE3

echo ""
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║  ✅ 캘리브레이션 명령 전송 완료!                              ║"
echo "╠══════════════════════════════════════════════════════════════╣"
echo "║                                                              ║"
echo "║  ⚠️  중요: 영점 설정을 적용하려면:                            ║"
echo "║                                                              ║"
echo "║     1. 모터 전원을 끄세요 (또는 E-STOP)                      ║"
echo "║     2. 2-3초 대기                                            ║"
echo "║     3. 모터 전원을 다시 켜세요                               ║"
echo "║                                                              ║"
echo "║  그 후 mm_moveit_hardware.launch.py를 실행하세요             ║"
echo "║                                                              ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""
