#!/bin/bash
# MyActuator RMD 모터 스캔 스크립트 (안정성 개선 버전)

CAN_IF=${1:-can0}

echo "========================================"
echo "  MyActuator RMD 모터 스캔 (Robust Mode)"
echo "========================================"
echo "CAN 인터페이스: $CAN_IF"

if ! ip link show $CAN_IF &> /dev/null; then
    echo "오류: $CAN_IF 인터페이스가 없습니다."
    exit 1
fi

MOTOR_IDS=(1 2 3 4)

echo "모터 스캔 중 (4초간 대기)..."
echo ""

TMPFILE=$(mktemp)

# 1. candump를 더 넉넉하게 4초간 실행 (백그라운드)
timeout 4s candump $CAN_IF > $TMPFILE &
CANDUMP_PID=$!

# 2. 버스가 안정화될 때까지 잠시 대기
sleep 0.5

# 3. 각 모터에 순차적으로 요청 전송 (간격을 0.15s로 약간 늘림)
for ID in "${MOTOR_IDS[@]}"; do
    CAN_ID=$(printf "%03X" $((0x140 + ID)))
    
    # 0x9C 명령 전송
    cansend $CAN_IF ${CAN_ID}#9C00000000000000 2>/dev/null
    sleep 0.15
done

# 4. candump가 종료될 때까지 대기
wait $CANDUMP_PID 2>/dev/null

echo "========================================"
echo "  스캔 결과"
echo "========================================"

FOUND=0
for ID in "${MOTOR_IDS[@]}"; do
    CAN_ID=$(printf "%03X" $((0x140 + ID)))
    RESPONSE_ID=$(printf "%03X" $((0x240 + ID)))
    
    # 5. [수정] 정규표현식으로 공백에 상관없이 검색
    # RESPONSE_ID 뒤에 데이터가 오는지 확인
    RESPONSE=$(grep -E "$CAN_IF[[:space:]]+$RESPONSE_ID" $TMPFILE | tail -1)
    
    if [ ! -z "$RESPONSE" ]; then
        echo "✅ 모터 $ID (CAN ID: 0x$CAN_ID) - 응답 있음"
        
        # 데이터 부분만 추출하여 출력
        DATA=$(echo "$RESPONSE" | sed -E "s/.*$RESPONSE_ID[[:space:]]+\[[0-9]\][[:space:]]+//")
        echo "   응답 데이터: $DATA"
        FOUND=$((FOUND + 1))
    else
        echo "❌ 모터 $ID (CAN ID: 0x$CAN_ID) - 응답 없음"
    fi
done

echo ""
echo "========================================"
echo "  총 $FOUND / ${#MOTOR_IDS[@]} 개 모터 발견"
echo "========================================"

rm -f $TMPFILE