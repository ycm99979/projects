#!/usr/bin/env python3
#ros2 topic pub /target_point geometry_msgs/Point "{x: 0.4, y: 0.0, z: 0.5}" --once && sleep 8 && ros2 topic pub /target_point geometry_msgs/Point "{x: 0.5, y: 0.2, z: 0.6}" --once && sleep 8 && ros2 topic pub /target_point geometry_msgs/Point "{x: 0.2, y: -0.2, z: 0.4}" --once

"""
MoveIt Pose Control Script
특정 위치 좌표(x, y, z)를 입력받아 MoveIt으로 Plan & Execute

Usage:
    # Terminal 1: Launch MoveIt + Hardware
    ros2 launch robot_bringup mm_moveit_hardware.launch.py
    
    # Terminal 2: Run this script
    python3 src/robot_bringup/scripts/moveit_pose_control.py
    
    # Terminal 3: Send target position
    ros2 topic pub /target_pose geometry_msgs/Pose "{position: {x: 0.25, y: 0.0, z: 0.15}, orientation: {w: 1.0}}" --once
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient

from geometry_msgs.msg import Pose, PoseStamped, Point
from std_msgs.msg import String
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, 
    Constraints, 
    PositionConstraint, 
    OrientationConstraint,
    BoundingVolume,
    MoveItErrorCodes,
    WorkspaceParameters
)
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState

import time
import math


class MoveItPoseControl(Node):
    def __init__(self):
        super().__init__('moveit_pose_control')
        
        self.callback_group = ReentrantCallbackGroup()
        self._busy = False
        
        # Parameters
        self.declare_parameter('planning_group', 'arm')
        self.declare_parameter('end_effector_link', 'gripper_base')  # IK solver의 tip link
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('planning_time', 5.0)
        self.declare_parameter('num_planning_attempts', 10)
        
        self.planning_group = self.get_parameter('planning_group').value
        self.end_effector_link = self.get_parameter('end_effector_link').value
        self.base_frame = self.get_parameter('base_frame').value
        self.planning_time = self.get_parameter('planning_time').value
        self.num_planning_attempts = self.get_parameter('num_planning_attempts').value
        
        # Current joint states
        self.current_joint_state = None
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        
        # MoveGroup action client
        self.move_group_client = ActionClient(
            self, MoveGroup, '/move_action',
            callback_group=self.callback_group
        )
        
        # Wait for action server
        self.get_logger().info('Waiting for MoveGroup action server...')
        if not self.move_group_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('MoveGroup action server not available!')
            return
        self.get_logger().info('MoveGroup action server ready!')
        
        # Subscribers for target pose
        self.pose_sub = self.create_subscription(
            Pose, '/target_pose', self.pose_callback, 10,
            callback_group=self.callback_group
        )
        
        self.point_sub = self.create_subscription(
            Point, '/target_point', self.point_callback, 10,
            callback_group=self.callback_group
        )
        
        # Status publisher
        self.status_pub = self.create_publisher(String, '/moveit_status', 10)
        
        self.get_logger().info('='*50)
        self.get_logger().info('MoveIt Pose Control Ready!')
        self.get_logger().info('='*50)
        self.get_logger().info('Send target pose:')
        self.get_logger().info('  ros2 topic pub /target_pose geometry_msgs/Pose \\')
        self.get_logger().info('    "{position: {x: 0.25, y: 0.0, z: 0.15}, orientation: {w: 1.0}}" --once')
        self.get_logger().info('')
        self.get_logger().info('Or send target point (orientation auto):')
        self.get_logger().info('  ros2 topic pub /target_point geometry_msgs/Point \\')
        self.get_logger().info('    "{x: 0.25, y: 0.0, z: 0.15}" --once')
        self.get_logger().info('='*50)
    
    def joint_state_callback(self, msg: JointState):
        self.current_joint_state = msg
    
    def publish_status(self, message: str):
        msg = String()
        msg.data = message
        self.status_pub.publish(msg)
        self.get_logger().info(f'[Status] {message}')
    
    def point_callback(self, msg: Point):
        """Point 메시지 수신 시 위치만으로 Pose 생성 (orientation 없음)"""
        pose = Pose()
        pose.position = msg
        # orientation을 0으로 설정하면 constraint 추가 안 함
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 0.0  # w=0이면 orientation constraint 스킵
        self.pose_callback(pose)
    
    def pose_callback(self, msg: Pose):
        """Pose 메시지 수신 시 MoveIt Plan & Execute"""
        if self._busy:
            self.get_logger().warn('Busy processing previous request, ignoring...')
            return
        
        self._busy = True
        try:
            self.move_to_pose(msg)
        finally:
            self._busy = False
    
    def move_to_pose(self, target_pose: Pose) -> bool:
        """MoveIt을 사용하여 목표 pose로 이동"""
        self.publish_status(f'Planning to pose: x={target_pose.position.x:.3f}, '
                           f'y={target_pose.position.y:.3f}, z={target_pose.position.z:.3f}')
        
        # MoveGroup Goal 생성
        goal = MoveGroup.Goal()
        
        # Motion Plan Request 설정
        goal.request.group_name = self.planning_group
        goal.request.num_planning_attempts = self.num_planning_attempts
        goal.request.allowed_planning_time = self.planning_time
        goal.request.max_velocity_scaling_factor = 0.5
        goal.request.max_acceleration_scaling_factor = 0.5
        
        # Workspace 설정
        goal.request.workspace_parameters.header.frame_id = self.base_frame
        goal.request.workspace_parameters.min_corner.x = -1.0
        goal.request.workspace_parameters.min_corner.y = -1.0
        goal.request.workspace_parameters.min_corner.z = -1.0
        goal.request.workspace_parameters.max_corner.x = 1.0
        goal.request.workspace_parameters.max_corner.y = 1.0
        goal.request.workspace_parameters.max_corner.z = 1.0
        
        # Goal Constraints 설정 (Position)
        constraints = Constraints()
        
        # Position Constraint
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = self.base_frame
        position_constraint.link_name = self.end_effector_link
        position_constraint.target_point_offset.x = 0.0
        position_constraint.target_point_offset.y = 0.0
        position_constraint.target_point_offset.z = 0.0
        
        # Bounding volume (허용 오차 영역)
        bounding_volume = BoundingVolume()
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.01]  # 1cm 허용 오차
        bounding_volume.primitives.append(primitive)
        
        primitive_pose = Pose()
        primitive_pose.position = target_pose.position
        primitive_pose.orientation.w = 1.0
        bounding_volume.primitive_poses.append(primitive_pose)
        
        position_constraint.constraint_region = bounding_volume
        position_constraint.weight = 1.0
        
        constraints.position_constraints.append(position_constraint)
        
        # Orientation Constraint (w != 0일 때만 추가)
        if target_pose.orientation.w != 0.0:
            orientation_constraint = OrientationConstraint()
            orientation_constraint.header.frame_id = self.base_frame
            orientation_constraint.link_name = self.end_effector_link
            orientation_constraint.orientation = target_pose.orientation
            orientation_constraint.absolute_x_axis_tolerance = 0.5  # 더 여유롭게
            orientation_constraint.absolute_y_axis_tolerance = 0.5
            orientation_constraint.absolute_z_axis_tolerance = 0.5
            orientation_constraint.weight = 0.3  # 낮은 가중치
            
            constraints.orientation_constraints.append(orientation_constraint)
        
        goal.request.goal_constraints.append(constraints)
        
        # Planning options
        goal.planning_options.plan_only = False  # Plan & Execute
        goal.planning_options.look_around = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3
        goal.planning_options.replan_delay = 0.1
        
        # Send goal
        self.publish_status('Sending goal to MoveGroup...')
        
        send_goal_future = self.move_group_client.send_goal_async(goal)
        
        # Wait for goal acceptance
        timeout = 10.0
        start_time = time.time()
        while not send_goal_future.done() and (time.time() - start_time) < timeout:
            time.sleep(0.05)
        
        if not send_goal_future.done():
            self.publish_status('ERROR: Goal send timeout')
            return False
        
        goal_handle = send_goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.publish_status('ERROR: Goal rejected')
            return False
        
        self.publish_status('Goal accepted, executing...')
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        timeout = 60.0  # 최대 60초 대기
        start_time = time.time()
        while not result_future.done() and (time.time() - start_time) < timeout:
            time.sleep(0.1)
        
        if not result_future.done():
            self.publish_status('ERROR: Execution timeout')
            return False
        
        result = result_future.result()
        
        if result.result.error_code.val == MoveItErrorCodes.SUCCESS:
            self.publish_status('SUCCESS: Target pose reached!')
            return True
        else:
            error_code = result.result.error_code.val
            error_names = {
                -1: 'FAILURE',
                -2: 'PLANNING_FAILED',
                -3: 'INVALID_MOTION_PLAN',
                -4: 'MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE',
                -5: 'CONTROL_FAILED',
                -6: 'UNABLE_TO_AQUIRE_SENSOR_DATA',
                -7: 'TIMED_OUT',
                -10: 'PREEMPTED',
                -12: 'START_STATE_IN_COLLISION',
                -13: 'START_STATE_VIOLATES_PATH_CONSTRAINTS',
                -31: 'INVALID_GROUP_NAME',
                -32: 'INVALID_GOAL_CONSTRAINTS',
                -33: 'INVALID_ROBOT_STATE',
                -34: 'INVALID_LINK_NAME',
                -35: 'INVALID_OBJECT_NAME',
                -36: 'FRAME_TRANSFORM_FAILURE',
                -37: 'COLLISION_CHECKING_UNAVAILABLE',
                -38: 'ROBOT_STATE_STALE',
                -39: 'SENSOR_INFO_STALE',
                -99: 'NO_IK_SOLUTION',
                99999: 'UNKNOWN_ERROR',
            }
            error_name = error_names.get(error_code, f'UNKNOWN({error_code})')
            self.publish_status(f'ERROR: MoveIt failed - {error_name} (code: {error_code})')
            return False
            return False


def main(args=None):
    rclpy.init(args=args)
    
    node = MoveItPoseControl()
    
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
