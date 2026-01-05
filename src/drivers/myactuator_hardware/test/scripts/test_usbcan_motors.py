#!/usr/bin/env python3
"""
USBCAN-UC12 모터 연결 및 응답속도 테스트 스크립트

libusbcan.so를 사용하여 MyActuator RMD 모터 연결을 테스트합니다.

사용법:
    python3 test_usbcan_motors.py
    python3 test_usbcan_motors.py --motor-ids 1 2 3 4
    python3 test_usbcan_motors.py --benchmark
    python3 test_usbcan_motors.py --benchmark --iterations 100
"""

import argparse
import sys
import time
from ctypes import *

# ============================================================================
# USBCAN Library Structures
# ============================================================================

USBCAN_II = c_uint32(4)

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

BAUDRATE_TIMING = {
    1000000: 0x1400,
    500000: 0x1c00,
    250000: 0x1c01,
    125000: 0x1c03,
}

# ============================================================================
# USBCAN Functions
# ============================================================================

def load_library():
    try:
        lib = cdll.LoadLibrary("libusbcan.so")
        print("✓ libusbcan.so loaded")
        return lib
    except OSError as e:
        print(f"✗ Failed to load libusbcan.so: {e}")
        return None

def open_device(lib):
    ret = lib.VCI_OpenDevice(USBCAN_II, 0, 0)
    if ret == 1:
        print("✓ USBCAN device opened")
        return True
    print("✗ Failed to open USBCAN device")
    return False

def init_can_channel(lib, channel, baudrate):
    timing = BAUDRATE_TIMING.get(baudrate, 0x1400)
    config = ZCAN_CAN_INIT_CONFIG()
    config.AccCode = 0
    config.AccMask = 0xFFFFFFFF
    config.Reserved = 0
    config.Filter = 1
    config.Timing0 = timing & 0xFF
    config.Timing1 = (timing >> 8) & 0xFF
    config.Mode = 0
    
    ret = lib.VCI_InitCAN(USBCAN_II, 0, channel, byref(config))
    if ret == 1:
        print(f"✓ CAN channel {channel} initialized (baudrate: {baudrate})")
        return True
    return False

def start_can_channel(lib, channel):
    ret = lib.VCI_StartCAN(USBCAN_II, 0, channel)
    if ret == 1:
        print(f"✓ CAN channel {channel} started")
        return True
    return False

def close_device(lib):
    lib.VCI_ResetCAN(USBCAN_II, 0, 0)
    lib.VCI_CloseDevice(USBCAN_II, 0)
    print("✓ USBCAN device closed")

def send_can_frame(lib, channel, can_id, data):
    msg = ZCAN_CAN_OBJ()
    msg.ID = can_id
    msg.SendType = 0
    msg.RemoteFlag = 0
    msg.ExternFlag = 0
    msg.DataLen = len(data)
    for i, b in enumerate(data):
        msg.Data[i] = b
    ret = lib.VCI_Transmit(USBCAN_II, 0, channel, byref(msg), 1)
    return ret == 1

def receive_can_frame(lib, channel, timeout_ms=1):
    """Receive CAN frame with minimal timeout"""
    msg = ZCAN_CAN_OBJ()
    ret = lib.VCI_Receive(USBCAN_II, 0, channel, byref(msg), 1, timeout_ms)
    if ret > 0:
        data = [msg.Data[i] for i in range(msg.DataLen)]
        return msg.ID, data
    return None, None

def get_receive_count(lib, channel):
    """Get number of frames in receive buffer"""
    return lib.VCI_GetReceiveNum(USBCAN_II, 0, channel)

# ============================================================================
# Motor Communication (Optimized - No Sleep)
# ============================================================================

def read_motor_fast(lib, channel, motor_id, timeout_ms=1):
    """Read motor status with minimal latency (no sleep)"""
    tx_id = 0x140 + motor_id
    rx_id = 0x240 + motor_id
    
    # Send read motor status 2 command (0x9C)
    cmd = [0x9C, 0, 0, 0, 0, 0, 0, 0]
    
    if not send_can_frame(lib, channel, tx_id, cmd):
        return None
    
    # Poll for response (no sleep!)
    for _ in range(20):
        recv_id, recv_data = receive_can_frame(lib, channel, timeout_ms)
        if recv_id == rx_id and recv_data and recv_data[0] == 0x9C:
            return recv_data
    
    return None

def test_motor(lib, channel, motor_id):
    """Test motor communication"""
    data = read_motor_fast(lib, channel, motor_id, timeout_ms=10)
    if data and len(data) >= 8:
        temp = data[1]
        speed = (data[5] << 8) | data[4]
        if speed > 32767:
            speed -= 65536
        encoder = (data[7] << 8) | data[6]
        return True, f"Temp: {temp}°C, Speed: {speed}dps, Encoder: {encoder}"
    return False, "No response"

# ============================================================================
# Benchmark Functions
# ============================================================================

def benchmark_single_motor(lib, channel, motor_id, iterations=100):
    """Benchmark single motor read latency"""
    times = []
    success = 0
    
    for _ in range(iterations):
        start = time.perf_counter()
        data = read_motor_fast(lib, channel, motor_id, timeout_ms=1)
        elapsed = (time.perf_counter() - start) * 1000  # ms
        
        if data:
            success += 1
            times.append(elapsed)
    
    if times:
        avg = sum(times) / len(times)
        min_t = min(times)
        max_t = max(times)
        return success, iterations, avg, min_t, max_t
    return success, iterations, 0, 0, 0

def benchmark_all_motors(lib, channel, motor_ids, iterations=100):
    """Benchmark reading all motors sequentially"""
    times = []
    success = 0
    
    for _ in range(iterations):
        start = time.perf_counter()
        all_ok = True
        for motor_id in motor_ids:
            data = read_motor_fast(lib, channel, motor_id, timeout_ms=1)
            if not data:
                all_ok = False
        elapsed = (time.perf_counter() - start) * 1000  # ms
        
        if all_ok:
            success += 1
            times.append(elapsed)
    
    if times:
        avg = sum(times) / len(times)
        min_t = min(times)
        max_t = max(times)
        max_hz = 1000.0 / avg if avg > 0 else 0
        return success, iterations, avg, min_t, max_t, max_hz
    return success, iterations, 0, 0, 0, 0

# ============================================================================
# Main
# ============================================================================

def main():
    parser = argparse.ArgumentParser(description='Test USBCAN motor connection and latency')
    parser.add_argument('--motor-ids', nargs='+', type=int, default=[1, 2, 3, 4],
                        help='Motor IDs to test (default: 1 2 3 4)')
    parser.add_argument('--channel', type=int, default=0,
                        help='CAN channel (default: 0)')
    parser.add_argument('--baudrate', type=int, default=1000000,
                        choices=[1000000, 500000, 250000, 125000],
                        help='CAN baudrate (default: 1000000)')
    parser.add_argument('--benchmark', action='store_true',
                        help='Run latency benchmark')
    parser.add_argument('--iterations', type=int, default=100,
                        help='Benchmark iterations (default: 100)')
    args = parser.parse_args()
    
    print("=" * 60)
    print("  USBCAN-UC12 Motor Test")
    print("=" * 60)
    print()
    
    lib = load_library()
    if not lib:
        return 1
    
    if not open_device(lib):
        return 1
    
    try:
        if not init_can_channel(lib, args.channel, args.baudrate):
            return 1
        if not start_can_channel(lib, args.channel):
            return 1
        
        # Basic connection test
        print()
        print("-" * 60)
        print("  Connection Test")
        print("-" * 60)
        
        connected_motors = []
        for motor_id in args.motor_ids:
            print(f"\n  Motor ID {motor_id}:", end=" ")
            success, msg = test_motor(lib, args.channel, motor_id)
            if success:
                print(f"✓ {msg}")
                connected_motors.append(motor_id)
            else:
                print(f"✗ {msg}")
        
        print(f"\n  Connected: {len(connected_motors)}/{len(args.motor_ids)} motors")
        
        # Benchmark if requested
        if args.benchmark and connected_motors:
            print()
            print("-" * 60)
            print("  Latency Benchmark (No Sleep, Minimal Timeout)")
            print("-" * 60)
            
            # Single motor benchmark
            print(f"\n  Single Motor Read ({args.iterations} iterations):")
            for motor_id in connected_motors:
                success, total, avg, min_t, max_t = benchmark_single_motor(
                    lib, args.channel, motor_id, args.iterations)
                print(f"    Motor {motor_id}: avg={avg:.2f}ms, min={min_t:.2f}ms, max={max_t:.2f}ms ({success}/{total} ok)")
            
            # All motors benchmark
            print(f"\n  All Motors Read ({args.iterations} iterations):")
            success, total, avg, min_t, max_t, max_hz = benchmark_all_motors(
                lib, args.channel, connected_motors, args.iterations)
            print(f"    {len(connected_motors)} motors: avg={avg:.2f}ms, min={min_t:.2f}ms, max={max_t:.2f}ms")
            print(f"    Max update rate: {max_hz:.1f} Hz ({success}/{total} ok)")
            
            # Comparison
            print()
            print("-" * 60)
            print("  Analysis")
            print("-" * 60)
            if avg > 0:
                print(f"\n  Current: {avg:.2f}ms for {len(connected_motors)} motors = {max_hz:.1f} Hz")
                print(f"  Target 100Hz needs: <10ms for all motors")
                print(f"  Target 50Hz needs:  <20ms for all motors")
                if max_hz >= 100:
                    print(f"\n  ✓ Can achieve 100Hz update rate!")
                elif max_hz >= 50:
                    print(f"\n  ✓ Can achieve 50Hz update rate")
                else:
                    print(f"\n  ⚠ May need to reduce update rate to {int(max_hz * 0.8)}Hz")
        
        print()
        return 0
        
    finally:
        close_device(lib)

if __name__ == "__main__":
    sys.exit(main())
