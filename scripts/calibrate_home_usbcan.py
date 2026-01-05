#!/usr/bin/env python3
"""
============================================================================
Motor Home Position Calibration Script (USBCAN Version)
============================================================================

USBCAN-II 모듈을 사용하여 모터의 현재 위치를 홈(0) 위치로 설정하는 스크립트

[사용법]
  python3 calibrate_home_usbcan.py [--baudrate 1000000] [--motor-ids 1,2,3,4]

[MyActuator RMD 모터 명령]
  0x64: Write current multi-turn position to ROM as zero point
  0x92: Read multi-turn angle

============================================================================
"""

import sys
import time
import argparse
from ctypes import *

# ============================================================================
# USBCAN Library Structures
# ============================================================================

class ZCAN_CAN_INIT_CONFIG(Structure):
    _fields_ = [
        ("AccCode", c_int),
        ("AccMask", c_int),
        ("Reserved", c_int),
        ("Filter", c_ubyte),
        ("Timing0", c_ubyte),
        ("Timing1", c_ubyte),
        ("Mode", c_ubyte)
    ]


class ZCAN_CAN_OBJ(Structure):
    _fields_ = [
        ("ID", c_uint32),
        ("TimeStamp", c_uint32),
        ("TimeFlag", c_uint8),
        ("SendType", c_byte),
        ("RemoteFlag", c_byte),
        ("ExternFlag", c_byte),
        ("DataLen", c_byte),
        ("Data", c_ubyte * 8),
        ("Reserved", c_ubyte * 3)
    ]


# ============================================================================
# Baudrate Codes
# ============================================================================
BAUDRATE_CODES = {
    1000000: 0x1400,  # 1Mbps
    500000: 0x1c00,   # 500kbps
    250000: 0x1c01,   # 250kbps
    125000: 0x1c03,   # 125kbps
}


# ============================================================================
# USBCAN Calibration Class
# ============================================================================

class USBCANCalibrator:
    USBCAN_II = c_uint32(4)  # USBCAN-II/II+ device type

    def __init__(self, baudrate=1000000, channel=0):
        self.lib = None
        self.device_opened = False
        self.channel = channel
        self.baudrate = baudrate
        self.baud_code = BAUDRATE_CODES.get(baudrate, 0x1400)

    def open_device(self):
        """USBCAN 장치 열기"""
        try:
            self.lib = cdll.LoadLibrary("/lib/libusbcan.so")
        except OSError as e:
            print(f"  libusbcan.so 로드 실패: {e}")
            print("  /lib/libusbcan.so 파일이 있는지 확인하세요.")
            return False

        ret = self.lib.VCI_OpenDevice(self.USBCAN_II, 0, 0)
        if ret == 0:
            print("  USBCAN 장치 열기 실패")
            return False

        self.device_opened = True
        print("  USBCAN 장치 열기 성공")
        return True

    def init_channel(self):
        """CAN 채널 초기화"""
        if not self.device_opened:
            return False

        init_config = ZCAN_CAN_INIT_CONFIG()
        init_config.AccCode = 0
        init_config.AccMask = 0xFFFFFFFF
        init_config.Reserved = 0
        init_config.Filter = 1
        init_config.Timing0 = self.baud_code & 0xff
        init_config.Timing1 = (self.baud_code >> 8) & 0xff
        init_config.Mode = 0

        ret = self.lib.VCI_InitCAN(self.USBCAN_II, 0, self.channel, byref(init_config))
        if ret == 0:
            print(f"  CAN 채널 {self.channel} 초기화 실패")
            return False
        print(f"  CAN 채널 {self.channel} 초기화 성공 (보드레이트: {self.baudrate}bps)")

        ret = self.lib.VCI_StartCAN(self.USBCAN_II, 0, self.channel)
        if ret == 0:
            print(f"  CAN 채널 {self.channel} 시작 실패")
            return False
        print(f"  CAN 채널 {self.channel} 시작 성공")

        return True

    def send_message(self, can_id, data):
        """CAN 메시지 전송"""
        msg = ZCAN_CAN_OBJ()
        msg.ID = can_id
        msg.SendType = 0  # 정상 전송
        msg.RemoteFlag = 0  # 데이터 프레임
        msg.ExternFlag = 0  # 표준 프레임
        msg.DataLen = len(data)
        for i, byte in enumerate(data):
            msg.Data[i] = byte

        ret = self.lib.VCI_Transmit(self.USBCAN_II, 0, self.channel, byref(msg), 1)
        return ret == 1

    def receive_messages(self, timeout_ms=500):
        """CAN 메시지 수신"""
        messages = []
        start_time = time.time()

        while (time.time() - start_time) * 1000 < timeout_ms:
            count = self.lib.VCI_GetReceiveNum(self.USBCAN_II, 0, self.channel)
            if count > 0:
                can_msgs = (ZCAN_CAN_OBJ * count)()
                rcount = self.lib.VCI_Receive(self.USBCAN_II, 0, self.channel, byref(can_msgs), count, 100)
                for i in range(rcount):
                    msg = can_msgs[i]
                    messages.append({
                        'id': msg.ID,
                        'data': bytes(msg.Data[:msg.DataLen]),
                        'timestamp': msg.TimeStamp
                    })
            time.sleep(0.01)

        return messages

    def read_motor_position(self, motor_id):
        """모터 현재 위치 읽기 (0x92 명령)"""
        can_id = 0x140 + motor_id
        response_id = 0x240 + motor_id

        # 0x92: Read multi-turn angle
        data = [0x92, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

        # 버퍼 비우기
        self.receive_messages(timeout_ms=50)

        if not self.send_message(can_id, data):
            return None

        # 응답 대기
        time.sleep(0.1)
        messages = self.receive_messages(timeout_ms=300)

        for msg in messages:
            if msg['id'] == response_id and len(msg['data']) >= 8 and msg['data'][0] == 0x92:
                # 바이트 4-7: 각도 (0.01도 단위, int64)
                angle_raw = int.from_bytes(msg['data'][4:8], byteorder='little', signed=True)
                angle_deg = angle_raw * 0.01
                return angle_deg

        return None

    def set_zero_position(self, motor_id):
        """현재 위치를 영점으로 설정 (0x64 명령)"""
        can_id = 0x140 + motor_id
        response_id = 0x240 + motor_id

        # 0x64: Write current multi-turn position to ROM as zero point
        data = [0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

        # 버퍼 비우기
        self.receive_messages(timeout_ms=50)

        if not self.send_message(can_id, data):
            return False, "전송 실패"

        # 응답 대기
        time.sleep(0.2)
        messages = self.receive_messages(timeout_ms=500)

        for msg in messages:
            if msg['id'] == response_id and len(msg['data']) >= 1 and msg['data'][0] == 0x64:
                return True, "성공"

        return True, "명령 전송됨 (응답 없음)"

    def close_device(self):
        """장치 닫기"""
        if self.lib and self.device_opened:
            self.lib.VCI_ResetCAN(self.USBCAN_II, 0, self.channel)
            self.lib.VCI_CloseDevice(self.USBCAN_II, 0)
            print("  USBCAN 장치 닫기 완료")


def main():
    parser = argparse.ArgumentParser(description='USBCAN Motor Home Calibration')
    parser.add_argument('--baudrate', type=int, default=1000000,
                        help='CAN baudrate (default: 1000000)')
    parser.add_argument('--motor-ids', type=str, default='1,2,3,4',
                        help='Motor IDs (comma-separated, default: 1,2,3,4)')
    parser.add_argument('--channel', type=int, default=0,
                        help='CAN channel (0 or 1, default: 0)')
    args = parser.parse_args()

    motor_ids = [int(x.strip()) for x in args.motor_ids.split(',')]

    print("")
    print("=" * 66)
    print("       Motor Home Position Calibration (USBCAN)")
    print("=" * 66)
    print("")
    print("  현재 모터 위치를 영점(0)으로 ROM에 저장합니다")
    print("")
    print("  [주의] 캘리브레이션 후 모터 전원을 껐다 켜야 적용됩니다!")
    print("")
    print("=" * 66)
    print("")

    calibrator = USBCANCalibrator(baudrate=args.baudrate, channel=args.channel)

    # Step 1: 장치 열기
    print("[Step 1] USBCAN 장치 열기...")
    if not calibrator.open_device():
        print("  장치 열기 실패. 종료합니다.")
        sys.exit(1)

    if not calibrator.init_channel():
        print("  채널 초기화 실패. 종료합니다.")
        calibrator.close_device()
        sys.exit(1)

    print("")

    # Step 2: 현재 위치 확인
    print("[Step 2] 현재 모터 위치 확인...")
    print("")
    for motor_id in motor_ids:
        angle = calibrator.read_motor_position(motor_id)
        if angle is not None:
            print(f"  모터 {motor_id} (0x{0x140 + motor_id:03X}): {angle:.2f}도")
        else:
            print(f"  모터 {motor_id} (0x{0x140 + motor_id:03X}): 응답 없음")

    print("")

    # Step 3: 영점 설정
    print("[Step 3] 영점 설정 중...")
    print("  명령: 0x64 (Write current position to ROM as zero)")
    print("")

    results = []
    for motor_id in motor_ids:
        success, msg = calibrator.set_zero_position(motor_id)
        results.append((motor_id, success, msg))
        status = "성공" if success else "실패"
        print(f"  모터 {motor_id} (0x{0x140 + motor_id:03X}): {msg}")
        time.sleep(0.3)

    print("")

    # Step 4: 영점 설정 확인
    print("[Step 4] 영점 설정 후 위치 확인...")
    time.sleep(0.5)
    print("")
    for motor_id in motor_ids:
        angle = calibrator.read_motor_position(motor_id)
        if angle is not None:
            print(f"  모터 {motor_id}: {angle:.2f}도")
        else:
            print(f"  모터 {motor_id}: 확인 실패")

    print("")

    # 장치 닫기
    calibrator.close_device()

    # 완료 메시지
    print("")
    print("=" * 66)
    print("  캘리브레이션 명령 전송 완료!")
    print("=" * 66)
    print("")
    print("  [중요] 영점 설정을 적용하려면:")
    print("")
    print("     1. 모터 전원을 끄세요 (또는 E-STOP)")
    print("     2. 2-3초 대기")
    print("     3. 모터 전원을 다시 켜세요")
    print("")
    print("  그 후 mm_moveit_hardware.launch.py를 실행하세요")
    print("")
    print("=" * 66)
    print("")


if __name__ == "__main__":
    main()
