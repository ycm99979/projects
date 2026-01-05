#!/usr/bin/env python3
"""
Mobile Manipulator Pick and Place Demo

Usage:
    # Terminal 1: Launch demo
    ros2 launch mobile_manipulator_moveit_config demo.launch.py
    
    # Terminal 2: Run this script
    python3 src/mobile_manipulator_ws/src/mobile_manipulator_moveit_config/scripts/pick_place_demo.py
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Point, Pose, PoseStamped
from std_msgs.msg import String

from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import (
    MotionPlanRequest, Constraints, JointConstraint,
    MoveItErrorCodes, RobotState, WorkspaceParameters
)
from control_msgs.action import GripperCommand, FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState

from rclpy.action import ActionClient

import time


class MobileManipulatorPickPlace(Node):
    def __init__(self):
        super().__init__('mobile_manipulator_pick_place')

        # Declare parameters with default values
        self.declare_parameter('gripper_open_position', 0.04)
        self.declare_parameter('gripper_close_position', 0.0)
        self.declare_parameter('gripper_max_effort', 50.0)
        self.declare_parameter('default_duration_sec', 3.0)

        # Get parameter values
        self.gripper_open_position = self.get_parameter('gripper_open_position').value
        self.gripper_close_position = self.get_parameter('gripper_close_position').value
        self.gripper_max_effort = self.get_parameter('gripper_max_effort').value
        self.default_duration_sec = self.get_parameter('default_duration_sec').value

        self.callback_group = ReentrantCallbackGroup()
        self._busy = False

        # Current joint states
        self.current_joint_state = None
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        
        # Action clients
        self.move_group_client = ActionClient(
            self, MoveGroup, '/move_action',
            callback_group=self.callback_group
        )
        
        self.gripper_client = ActionClient(
            self, GripperCommand, '/gripper_controller/gripper_cmd',
            callback_group=self.callback_group
        )
        
        self.arm_trajectory_client = ActionClient(
            self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory',
            callback_group=self.callback_group
        )
        
        # Wait for action servers
        self.get_logger().info('Waiting for action servers...')
        self.move_group_client.wait_for_server(timeout_sec=10.0)
        self.gripper_client.wait_for_server(timeout_sec=10.0)
        self.arm_trajectory_client.wait_for_server(timeout_sec=10.0)
        self.get_logger().info('All action servers ready!')
        
        # Subscribe to target position topic
        self.target_sub = self.create_subscription(
            Point, '/target_position', self.target_callback, 10,
            callback_group=self.callback_group
        )
        
        # Status publisher
        self.status_pub = self.create_publisher(String, '/pick_place_status', 10)
        
        # Arm joint names
        self.arm_joints = [
            'link2_to_link1',
            'link3_to_link2', 
            'link4_to_link3',
            'gripper_to_link4'
        ]
        
        # Predefined positions (from SRDF)
        self.named_positions = {
            'init': [0.0, 0.0, 0.0, 0.0],
            'home': [0.0, -1.0, 1.0, 1.0],
            'ready': [0.0, -0.785, 0.785, 0.0],
            'front': [0.0, -1.0, 1.0, 0.5],
            'left': [1.00, -1.0, 1.0, 0.5],
            'right': [-1.00, -1.0, 1.0, 0.5],
        }
        
        self.get_logger().info('Mobile Manipulator Pick and Place ready!')
        
        # Run demo sequence after short delay
        self.create_timer(2.0, self.run_demo_once)
        self._demo_run = False
    
    def run_demo_once(self):
        if self._demo_run:
            return
        self._demo_run = True
        self.get_logger().info('Running demo sequence...')
        self.demo_sequence()
    
    def joint_state_callback(self, msg: JointState):
        self.current_joint_state = msg
        
    def publish_status(self, message: str):
        msg = String()
        msg.data = message
        self.status_pub.publish(msg)
        self.get_logger().info(f'[Status] {message}')
    
    def move_to_joint_positions(self, positions: list, name: str = "", duration_sec: float = None) -> bool:
        """Move arm to joint positions using trajectory controller"""
        if duration_sec is None:
            duration_sec = self.default_duration_sec
        try:
            self.get_logger().info(f'Moving to {name}: {[f"{p:.2f}" for p in positions]}')
            
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = self.arm_joints
            
            point = JointTrajectoryPoint()
            point.positions = positions
            point.velocities = [0.0] * len(positions)
            point.time_from_start = Duration(sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))
            goal.trajectory.points.append(point)
            
            future = self.arm_trajectory_client.send_goal_async(goal)
            
            # Wait for goal acceptance
            timeout = 5.0
            start = time.time()
            while not future.done() and (time.time() - start) < timeout:
                time.sleep(0.05)
            
            if not future.done():
                self.get_logger().error('Goal send timeout')
                return False
            
            goal_handle = future.result()
            if not goal_handle or not goal_handle.accepted:
                self.get_logger().error('Goal rejected')
                return False
            
            # Wait for result
            result_future = goal_handle.get_result_async()
            timeout = duration_sec + 5.0
            start = time.time()
            while not result_future.done() and (time.time() - start) < timeout:
                time.sleep(0.05)
            
            if result_future.done():
                self.get_logger().info(f'Successfully moved to {name}')
                return True
            else:
                self.get_logger().error('Execution timeout')
                return False
            
        except Exception as e:
            self.get_logger().error(f'Move error: {str(e)}')
            return False
    
    def move_to_named(self, target_name: str) -> bool:
        """Move to named target"""
        if target_name not in self.named_positions:
            self.get_logger().error(f'Unknown target: {target_name}')
            return False
        
        return self.move_to_joint_positions(self.named_positions[target_name], target_name)
    
    def control_gripper(self, open: bool) -> bool:
        """Control gripper"""
        try:
            action = "open" if open else "close"
            self.get_logger().info(f'Gripper: {action}')
            
            goal = GripperCommand.Goal()
            goal.command.position = self.gripper_open_position if open else self.gripper_close_position
            goal.command.max_effort = self.gripper_max_effort
            
            future = self.gripper_client.send_goal_async(goal)
            
            timeout = 5.0
            start = time.time()
            while not future.done() and (time.time() - start) < timeout:
                time.sleep(0.05)
            
            if not future.done():
                return False
            
            goal_handle = future.result()
            if not goal_handle or not goal_handle.accepted:
                return False
            
            result_future = goal_handle.get_result_async()
            timeout = 5.0
            start = time.time()
            while not result_future.done() and (time.time() - start) < timeout:
                time.sleep(0.05)
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'Gripper error: {str(e)}')
            return False
    
    def demo_sequence(self):
        """Demo sequence showing arm movements"""
        self.publish_status("=== Starting Demo Sequence ===")
        
        # 1. Init position
        self.publish_status("1. Moving to INIT position")
        self.move_to_named("init")
        time.sleep(0.5)
        
        # 2. Open gripper
        self.publish_status("2. Opening gripper")
        self.control_gripper(open=True)
        time.sleep(0.3)
        
        # 3. Home position
        self.publish_status("3. Moving to HOME position")
        self.move_to_named("home")
        time.sleep(0.5)
        
        # 4. Front position
        self.publish_status("4. Moving to FRONT position")
        self.move_to_named("front")
        time.sleep(0.5)
        
        # 5. Close gripper (simulating grasp)
        self.publish_status("5. Closing gripper (grasp)")
        self.control_gripper(open=False)
        time.sleep(0.3)
        
        # 6. Home position (lift)
        self.publish_status("6. Lifting to HOME")
        self.move_to_named("home")
        time.sleep(0.5)
        
        # 7. Left position
        self.publish_status("7. Moving to LEFT position")
        self.move_to_named("left")
        time.sleep(0.5)
        
        # 8. Open gripper (release)
        self.publish_status("8. Opening gripper (release)")
        self.control_gripper(open=True)
        time.sleep(0.3)
        
        # 9. Home position
        self.publish_status("9. Returning to HOME")
        self.move_to_named("home")
        time.sleep(0.5)
        
        # 10. Right position
        self.publish_status("10. Moving to RIGHT position")
        self.move_to_named("right")
        time.sleep(0.5)
        
        # 11. Back to init
        self.publish_status("11. Returning to INIT")
        self.move_to_named("init")
        
        self.publish_status("=== Demo Sequence Completed! ===")
        self.get_logger().info("Waiting for target positions on /target_position topic...")
    
    def target_callback(self, msg: Point):
        """Callback for target position - interprets as joint angles"""
        if self._busy:
            self.get_logger().warn('Busy, ignoring new target')
            return
        
        self._busy = True
        try:
            # Interpret x, y, z as first 3 joint angles (radians)
            # This is a simple demo - in real use, you'd use IK
            positions = [msg.x, msg.y, msg.z, 0.0]
            
            self.publish_status(f"Moving to joint positions: [{msg.x:.2f}, {msg.y:.2f}, {msg.z:.2f}, 0.0]")
            
            self.control_gripper(open=True)
            time.sleep(0.2)
            
            if self.move_to_joint_positions(positions, "target"):
                self.control_gripper(open=False)
                time.sleep(0.3)
                self.move_to_named("home")
                self.publish_status("Target reached!")
            else:
                self.publish_status("Failed to reach target")
                
        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')
        finally:
            self._busy = False


def main(args=None):
    rclpy.init(args=args)
    
    node = MobileManipulatorPickPlace()
    
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
