#!/bin/bash

# SLCAN 연결 종료 스크립트

echo "SLCAN 연결 종료 중..."

# can0 인터페이스 비활성화
if ip link show can0 &> /dev/null; then
    echo "can0 인터페이스 비활성화 중..."
    sudo ip link set can0 down
fi

# slcand 종료
echo "slcand 프로세스 종료 중..."
sudo slcand -k can0 2>/dev/null || true
sudo pkill slcand 2>/dev/null || true

echo "SLCAN 연결이 종료되었습니다."