#!/usr/bin/env python3
"""
============================================================================
Arm Teleop Node - 4DOF 로봇팔 텔레옵 제어
============================================================================

각 조인트를 개별적으로 증감 제어할 수 있는 노드

[구독 토픽]
- /arm_teleop/joint_delta (std_msgs/Float64MultiArray): 각 조인트 증감값 [j1, j2, j3, j4]
- /arm_teleop/joint_cmd (std_msgs/Float64MultiArray): 각 조인트 절대값 [j1, j2, j3, j4]

[발행 토픽]
- /arm_controller/joint_trajectory (trajectory_msgs/JointTrajectory)

============================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

import numpy as np
from threading import Lock


class ArmTeleop(Node):
    def __init__(self):
        super().__init__('arm_teleop')

        self.cb_group = ReentrantCallbackGroup()

        # Joint names
        self.joint_names = [
            'link2_to_link1',
            'link3_to_link2',
            'link4_to_link3',
            'gripper_to_link4'
        ]

        # Joint limits (rad)
        self.joint_limits = {
            'link2_to_link1': (-3.14, 3.14),
            'link3_to_link2': (-3.14, 3.14),
            'link4_to_link3': (-3.14, 3.14),
            'gripper_to_link4': (-3.14, 3.14)
        }

        # Current joint positions
        self.current_positions = [0.0, 0.0, 0.0, 0.0]
        self.position_lock = Lock()

        # Parameters
        self.declare_parameter('move_duration', 0.5)
        self.move_duration = self.get_parameter('move_duration').value

        # Subscriber: joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
            callback_group=self.cb_group
        )

        # Subscriber: delta commands (증감)
        self.delta_sub = self.create_subscription(
            Float64MultiArray,
            '/arm_teleop/joint_delta',
            self.delta_callback,
            10,
            callback_group=self.cb_group
        )

        # Subscriber: absolute commands (절대값)
        self.cmd_sub = self.create_subscription(
            Float64MultiArray,
            '/arm_teleop/joint_cmd',
            self.cmd_callback,
            10,
            callback_group=self.cb_group
        )

        # Publisher: joint trajectory
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )

        self.get_logger().info('='*50)
        self.get_logger().info('Arm Teleop Node Ready')
        self.get_logger().info('='*50)
        self.get_logger().info('Topics:')
        self.get_logger().info('  /arm_teleop/joint_delta - 증감 제어 [j1, j2, j3, j4]')
        self.get_logger().info('  /arm_teleop/joint_cmd   - 절대값 제어 [j1, j2, j3, j4]')
        self.get_logger().info('='*50)

    def joint_state_callback(self, msg: JointState):
        """현재 조인트 상태 업데이트"""
        with self.position_lock:
            for i, name in enumerate(self.joint_names):
                if name in msg.name:
                    idx = msg.name.index(name)
                    self.current_positions[i] = msg.position[idx]

    def delta_callback(self, msg: Float64MultiArray):
        """증감 명령 처리"""
        if len(msg.data) != 4:
            self.get_logger().warn(f'Expected 4 values, got {len(msg.data)}')
            return

        with self.position_lock:
            target_positions = list(self.current_positions)

        # Apply delta
        for i in range(4):
            target_positions[i] += msg.data[i]
            # Apply limits
            min_val, max_val = self.joint_limits[self.joint_names[i]]
            target_positions[i] = np.clip(target_positions[i], min_val, max_val)

        self.send_trajectory(target_positions)
        self.get_logger().info(f'Delta move: {[f"{d:.3f}" for d in msg.data]}')

    def cmd_callback(self, msg: Float64MultiArray):
        """절대값 명령 처리"""
        if len(msg.data) != 4:
            self.get_logger().warn(f'Expected 4 values, got {len(msg.data)}')
            return

        target_positions = list(msg.data)

        # Apply limits
        for i in range(4):
            min_val, max_val = self.joint_limits[self.joint_names[i]]
            target_positions[i] = np.clip(target_positions[i], min_val, max_val)

        self.send_trajectory(target_positions)
        self.get_logger().info(f'Absolute move: {[f"{p:.3f}" for p in target_positions]}')

    def send_trajectory(self, positions: list):
        """JointTrajectory 메시지 발행"""
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(
            sec=int(self.move_duration),
            nanosec=int((self.move_duration % 1) * 1e9)
        )
        traj.points.append(point)

        self.traj_pub.publish(traj)


def main(args=None):
    rclpy.init(args=args)
    node = ArmTeleop()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
