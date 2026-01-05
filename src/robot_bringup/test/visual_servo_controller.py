```#!/usr/bin/env python3
"""
============================================================================
Visual Servo Controller Node (Camera-to-EE Transform)
============================================================================

카메라 프레임 기준 목표점을 엔드이펙터 프레임으로 변환하여 추종

[동작 원리]
1. /target_point로 카메라 프레임 기준 목표 좌표 수신
2. TF를 이용해 카메라→엔드이펙터 변환
3. 엔드이펙터 프레임 기준 twist 명령 발행

[구독 토픽]
- /target_point (geometry_msgs/PointStamped): 카메라 프레임 기준 목표 좌표

[발행 토픽]  
- /servo_node/delta_twist_cmds (geometry_msgs/TwistStamped): Servo 명령

============================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PointStamped, TwistStamped
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs

import numpy as np
from threading import Lock


class VisualServoController(Node):
    def __init__(self):
        super().__init__('visual_servo_controller')
        
        # Parameters
        self.declare_parameter('servo_gain', 1.0)
        self.declare_parameter('max_linear_vel', 0.2)
        self.declare_parameter('position_tolerance', 0.01)
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('target_topic', 'target_point')
        self.declare_parameter('ee_frame', 'end_effector_link')
        self.declare_parameter('camera_frame', 'd405_optical_frame')
        self.declare_parameter('auto_enable', True)
        self.declare_parameter('servo_enable_service', '/servo_node/start_servo')

        self.servo_gain = self.get_parameter('servo_gain').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.control_rate = self.get_parameter('control_rate').value
        self.target_topic = self.get_parameter('target_topic').value
        self.ee_frame = self.get_parameter('ee_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.auto_enable = self.get_parameter('auto_enable').value
        self.servo_enable_service = self.get_parameter('servo_enable_service').value
        
        # Callback group
        self.cb_group = ReentrantCallbackGroup()
        
        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Target storage
        self.target_point = None
        self.target_lock = Lock()
        self.servo_enabled = False
        
        # Subscriber
        self.target_sub = self.create_subscription(
            PointStamped,
            self.target_topic,
            self.target_callback,
            10,
            callback_group=self.cb_group
        )
        
        # Publisher
        self.twist_pub = self.create_publisher(
            TwistStamped,
            '/servo_node/delta_twist_cmds',
            10
        )
        
        # Service
        self.enable_srv = self.create_service(
            SetBool,
            '~/enable',
            self.enable_callback,
            callback_group=self.cb_group
        )

        # Servo enable client
        self.servo_enable_client = self.create_client(Trigger, self.servo_enable_service)
        
        # Control loop
        self.control_timer = self.create_timer(
            1.0 / self.control_rate,
            self.control_loop,
            callback_group=self.cb_group
        )
        
        self.get_logger().info('='*60)
        self.get_logger().info('Visual Servo Controller (Camera-to-EE)')
        self.get_logger().info('='*60)
        self.get_logger().info(f'  Camera frame: {self.camera_frame}')
        self.get_logger().info(f'  EE frame: {self.ee_frame}')
        self.get_logger().info(f'  Servo gain: {self.servo_gain}')
        self.get_logger().info(f'  Max linear vel: {self.max_linear_vel} m/s')
        self.get_logger().info(f'  Position tolerance: {self.position_tolerance} m')
        self.get_logger().info('='*60)

        if self.auto_enable:
            self.servo_enabled = True
            self._enable_attempts = 0
            self._max_enable_attempts = 10
            self._enable_timer = self.create_timer(1.0, self._attempt_enable_service)
            self._call_servo_enable_service(True)

    def _attempt_enable_service(self):
        if self._enable_attempts >= self._max_enable_attempts:
            try:
                self.destroy_timer(self._enable_timer)
            except Exception:
                pass
            return

        if self.servo_enable_client.wait_for_service(timeout_sec=0.5):
            self._call_servo_enable_service(True)
            try:
                self.destroy_timer(self._enable_timer)
            except Exception:
                pass
            return

        self._enable_attempts += 1

    def _call_servo_enable_service(self, enable: bool):
        if not enable:
            return

        req = Trigger.Request()
        if self.servo_enable_client.wait_for_service(timeout_sec=2.0):
            fut = self.servo_enable_client.call_async(req)
            fut.add_done_callback(
                lambda f: self.get_logger().info(f'Servo started: {f.result().message}')
            )
        else:
            self.get_logger().warn('Servo service not available')

    def target_callback(self, msg: PointStamped):
        """목표점 수신"""
        with self.target_lock:
            self.target_point = msg
        self.get_logger().debug(
            f'Target received: frame={msg.header.frame_id} '
            f'pos=({msg.point.x:.3f}, {msg.point.y:.3f}, {msg.point.z:.3f})'
        )
        
    def enable_callback(self, request, response):
        self.servo_enabled = request.data
        response.success = True
        response.message = f"Visual servo {'enabled' if self.servo_enabled else 'disabled'}"
        self.get_logger().info(response.message)
        return response

    def transform_point_to_ee_frame(self, point_msg: PointStamped):
        """
        카메라 프레임 기준 좌표를 엔드이펙터 프레임으로 변환
        
        카메라에서 본 목표점 위치를 엔드이펙터 기준으로 변환하면,
        엔드이펙터가 그 방향으로 이동하면 목표점에 도달함
        """
        try:
            # 입력 프레임이 카메라 프레임이 아니면 먼저 카메라 프레임으로 변환
            if point_msg.header.frame_id != self.camera_frame:
                point_in_camera = self.tf_buffer.transform(
                    point_msg,
                    self.camera_frame,
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )
            else:
                point_in_camera = point_msg
            
            # 카메라 프레임 → 엔드이펙터 프레임 변환
            point_in_ee = self.tf_buffer.transform(
                point_in_camera,
                self.ee_frame,
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            return np.array([
                point_in_ee.point.x,
                point_in_ee.point.y,
                point_in_ee.point.z
            ])
            
        except TransformException as e:
            self.get_logger().warn(f'TF transform failed: {e}')
            return None
    
    def control_loop(self):
        """
        메인 제어 루프
        
        엔드이펙터 프레임 기준으로 목표점까지의 벡터를 계산하고,
        그 방향으로 twist 명령을 보냄
        """
        if not self.servo_enabled:
            return
        
        with self.target_lock:
            if self.target_point is None:
                return
            target = self.target_point
        
        # 목표점을 엔드이펙터 프레임으로 변환
        target_in_ee = self.transform_point_to_ee_frame(target)
        if target_in_ee is None:
            return
        
        # 엔드이펙터 프레임에서 목표점까지의 거리 = 에러
        error = target_in_ee  # 엔드이펙터 원점 기준
        error_norm = np.linalg.norm(error)
        
        # 허용 오차 내면 정지
        if error_norm < self.position_tolerance:
            self.publish_twist(0.0, 0.0, 0.0)
            return
        
        # 비례 제어 + 속도 제한
        velocity = self.servo_gain * error
        vel_norm = np.linalg.norm(velocity)
        
        if vel_norm > self.max_linear_vel:
            velocity = velocity * (self.max_linear_vel / vel_norm)
        
        # 엔드이펙터 프레임 기준 twist 발행
        self.publish_twist(velocity[0], velocity[1], velocity[2])
        
        self.get_logger().info(
            f'Error: {error_norm:.3f}m | '
            f'Target in EE: ({target_in_ee[0]:.3f}, {target_in_ee[1]:.3f}, {target_in_ee[2]:.3f}) | '
            f'Vel: ({velocity[0]:.3f}, {velocity[1]:.3f}, {velocity[2]:.3f})'
        )
    
    def publish_twist(self, vx: float, vy: float, vz: float):
        """엔드이펙터 프레임 기준 twist 발행"""
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = self.ee_frame
        
        twist_msg.twist.linear.x = vx
        twist_msg.twist.linear.y = vy
        twist_msg.twist.linear.z = vz
        twist_msg.twist.angular.x = 0.0
        twist_msg.twist.angular.y = 0.0
        twist_msg.twist.angular.z = 0.0
        
        self.twist_pub.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    node = VisualServoController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
