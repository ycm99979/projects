#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import sys
import select
import termios
import tty
import threading
import time

class DualMotorTeleop(Node):
    def __init__(self):
        super().__init__('dual_motor_teleop')

        # Declare parameters with default values
        self.declare_parameter('position_step', 0.1)
        self.declare_parameter('max_position', 3.14)
        self.declare_parameter('min_position', -3.14)
        self.declare_parameter('timer_period', 0.1)

        # Get parameter values
        self.position_step = self.get_parameter('position_step').value
        self.max_position = self.get_parameter('max_position').value
        self.min_position = self.get_parameter('min_position').value
        timer_period = self.get_parameter('timer_period').value

        # Publisher for forward position controller
        self.position_pub = self.create_publisher(
            Float64MultiArray,
            '/forward_position_controller/commands',
            10
        )

        # Current positions
        self.joint1_pos = 0.0
        self.joint2_pos = 0.0

        # Timer for publishing commands
        self.timer = self.create_timer(timer_period, self.publish_positions)
        
        self.get_logger().info('Dual Motor Teleop Node Started')
        self.print_instructions()
        
    def print_instructions(self):
        print("\n" + "="*50)
        print("DUAL MOTOR TELEOP CONTROL")
        print("="*50)
        print("Motor 1 (Joint1) Controls:")
        print("  q/a : Increase/Decrease position")
        print("")
        print("Motor 2 (Joint2) Controls:")
        print("  w/s : Increase/Decrease position")
        print("")
        print("Both Motors:")
        print("  e/d : Both motors increase/decrease")
        print("  r   : Reset both to zero")
        print("  +/- : Increase/Decrease step size")
        print("  h   : Show this help")
        print("  ESC : Exit")
        print("="*50)
        print(f"Current step size: {self.position_step:.3f} rad")
        print(f"Joint1: {self.joint1_pos:.3f} rad")
        print(f"Joint2: {self.joint2_pos:.3f} rad")
        print("="*50)
        
    def publish_positions(self):
        msg = Float64MultiArray()
        msg.data = [self.joint1_pos, self.joint2_pos]
        self.position_pub.publish(msg)
        
    def update_position(self, joint, delta):
        if joint == 1:
            self.joint1_pos = max(self.min_position, 
                                min(self.max_position, 
                                    self.joint1_pos + delta))
        elif joint == 2:
            self.joint2_pos = max(self.min_position, 
                                min(self.max_position, 
                                    self.joint2_pos + delta))
        elif joint == 'both':
            self.joint1_pos = max(self.min_position, 
                                min(self.max_position, 
                                    self.joint1_pos + delta))
            self.joint2_pos = max(self.min_position, 
                                min(self.max_position, 
                                    self.joint2_pos + delta))
                                    
    def reset_positions(self):
        self.joint1_pos = 0.0
        self.joint2_pos = 0.0
        print("Positions reset to zero")
        
    def print_status(self):
        print(f"\rStep: {self.position_step:.3f} | Joint1: {self.joint1_pos:.3f} | Joint2: {self.joint2_pos:.3f}", end='', flush=True)

def get_key():
    """Get a single keypress from stdin"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

def main():
    rclpy.init()
    
    teleop = DualMotorTeleop()
    
    # Start ROS2 spinning in a separate thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(teleop,))
    spin_thread.daemon = True
    spin_thread.start()
    
    try:
        while rclpy.ok():
            key = get_key()
            
            if key == '\x1b':  # ESC key
                break
            elif key == 'q':
                teleop.update_position(1, teleop.position_step)
            elif key == 'a':
                teleop.update_position(1, -teleop.position_step)
            elif key == 'w':
                teleop.update_position(2, teleop.position_step)
            elif key == 's':
                teleop.update_position(2, -teleop.position_step)
            elif key == 'e':
                teleop.update_position('both', teleop.position_step)
            elif key == 'd':
                teleop.update_position('both', -teleop.position_step)
            elif key == 'r':
                teleop.reset_positions()
            elif key == '+' or key == '=':
                teleop.position_step = min(1.0, teleop.position_step + 0.05)
                print(f"\nStep size increased to: {teleop.position_step:.3f}")
            elif key == '-':
                teleop.position_step = max(0.01, teleop.position_step - 0.05)
                print(f"\nStep size decreased to: {teleop.position_step:.3f}")
            elif key == 'h':
                teleop.print_instructions()
            elif key == '\x03':  # Ctrl+C
                break
                
            teleop.print_status()
            
    except KeyboardInterrupt:
        pass
    finally:
        print("\nShutting down teleop...")
        teleop.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()