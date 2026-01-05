#!/usr/bin/env python3
"""
듀얼 모터 사인파 움직임 테스트

두 개의 모터를 사인파 형태로 동시에 움직입니다.
ForwardCommandController를 사용합니다.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math
import time


class DualMotorSineWave(Node):
    def __init__(self):
        super().__init__('dual_motor_sine_wave')

        # Declare parameters with default values
        self.declare_parameter('amplitude', 0.5)
        self.declare_parameter('frequency', 0.1)
        self.declare_parameter('offset', 0.0)
        self.declare_parameter('phase_shift', 0.0)
        self.declare_parameter('timer_period', 0.02)

        # Get parameter values
        self.amplitude = self.get_parameter('amplitude').value
        self.frequency = self.get_parameter('frequency').value
        self.offset = self.get_parameter('offset').value
        self.phase_shift = self.get_parameter('phase_shift').value
        timer_period = self.get_parameter('timer_period').value

        # Publisher
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/forward_position_controller/commands',
            10
        )

        # Timer
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.start_time = time.time()
        
        self.get_logger().info('='*60)
        self.get_logger().info('  듀얼 모터 사인파 움직임 시작!')
        self.get_logger().info('='*60)
        self.get_logger().info(f'  진폭: {self.amplitude:.2f} rad ({math.degrees(self.amplitude):.1f}°)')
        self.get_logger().info(f'  주파수: {self.frequency:.2f} Hz')
        self.get_logger().info(f'  위상차: {self.phase_shift:.2f} rad ({math.degrees(self.phase_shift):.1f}°)')
        self.get_logger().info(f'  Ctrl+C로 종료')
        self.get_logger().info('='*60)
    
    def timer_callback(self):
        elapsed = time.time() - self.start_time
        
        # 사인파 계산 - 두 모터
        omega = 2 * math.pi * self.frequency
        position1 = self.offset + self.amplitude * math.sin(omega * elapsed)
        position2 = self.offset + self.amplitude * math.sin(omega * elapsed + self.phase_shift)
        
        # 메시지 발행
        msg = Float64MultiArray()
        msg.data = [position1, position2]
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = DualMotorSineWave()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('종료 중... 원점으로 복귀')
        # 원점 복귀
        msg = Float64MultiArray()
        msg.data = [0.0, 0.0]
        node.publisher.publish(msg)
        time.sleep(0.5)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
