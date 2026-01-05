#!/usr/bin/env python3
"""
============================================================================
Pick and Place Controller - YOLO Target with Camera Frame Transform
============================================================================

YOLO에서 카메라 프레임 기준 목표 좌표를 받아서 pick and place 수행

[동작 순서]
1. /target_point (PointStamped) 토픽으로 카메라 프레임 기준 목표 좌표 수신
2. TF로 base_link 기준으로 변환
3. 목표 위치 위(approach)로 이동
4. 목표 위치로 하강
5. 그리퍼 닫기 (pick)
6. 위로 상승 (retreat)
7. place 위치로 이동
8. 그리퍼 열기 (place)
9. ready 자세로 복귀

[사용법]
ros2 launch robot_bringup mobile_manipulator_moveit.launch.py
ros2 run robot_bringup topic_pick_place.py

[토픽]
- 입력: /target_point (geometry_msgs/PointStamped) - 카메라 프레임 기준 목표 좌표
- 출력: /pick_place_status (std_msgs/String) - 상태 메시지

============================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient

from geometry_msgs.msg import Point, PointStamped, Pose
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs

from moveit_msgs.msg import (
    MotionPlanRequest, Constraints, PositionConstraint,
    BoundingVolume, MoveItErrorCodes, RobotTrajectory,
    JointConstraint
)
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from control_msgs.action import GripperCommand, FollowJointTrajectory
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

import math
import time
import copy
import numpy as np


class TopicPickAndPlace(Node):
    def __init__(self):
        super().__init__('topic_pick_place')

        # ================================================================
        # Parameters
        # ================================================================
        self.declare_parameter('approach_height', 0.08)      # 접근 높이 (목표 위 8cm)
        self.declare_parameter('retreat_height', 0.12)       # 후퇴 높이 (목표 위 12cm)
        self.declare_parameter('gripper_open_position', 0.019)
        self.declare_parameter('gripper_close_position', -0.01)
        self.declare_parameter('gripper_max_effort', 50.0)
        self.declare_parameter('camera_frame', 'd405_optical_frame')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('target_frame', 'gripper_base')  # 타겟 변환 프레임 (엔드이펙터)
        self.declare_parameter('planning_group', 'arm')
        self.declare_parameter('end_effector_link', 'gripper_base')
        
        # Place 위치 (base_link 기준 고정 위치)
        self.declare_parameter('place_x', 0.0)
        self.declare_parameter('place_y', 0.25)
        self.declare_parameter('place_z', 0.15)

        self.approach_height = self.get_parameter('approach_height').value
        self.retreat_height = self.get_parameter('retreat_height').value
        self.gripper_open_position = self.get_parameter('gripper_open_position').value
        self.gripper_close_position = self.get_parameter('gripper_close_position').value
        self.gripper_max_effort = self.get_parameter('gripper_max_effort').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.target_frame = self.get_parameter('target_frame').value
        self.planning_group = self.get_parameter('planning_group').value
        self.end_effector_link = self.get_parameter('end_effector_link').value
        self.place_x = self.get_parameter('place_x').value
        self.place_y = self.get_parameter('place_y').value
        self.place_z = self.get_parameter('place_z').value

        self.callback_group = ReentrantCallbackGroup()
        self._busy = False
        
        # 중복 타겟 방지를 위한 변수들
        self.last_target_time = None
        self.target_cooldown = 2.0  # 2초 쿨다운
        self.last_target_position = None
        self.position_threshold = 0.05  # 5cm 이내 동일 위치 무시

        # ================================================================
        # TF2 for coordinate transform
        # ================================================================
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ================================================================
        # Joint states
        # ================================================================
        self.current_joint_state = None
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

        # Arm joint names (from SRDF)
        self.arm_joints = [
            'link2_to_link1',
            'link3_to_link2',
            'link4_to_link3',
            'gripper_to_link4'
        ]

        # Ready pose (from SRDF - singularity-free)
        self.ready_positions = [0.0, -0.69813, -2.35619, 0.05236]

        # Place pose (물체를 놓는 위치)
        self.place_positions = [-1.815, 1.117, -0.820, 0.0]

        # ================================================================
        # MoveIt clients
        # ================================================================
        self.plan_client = self.create_client(
            GetMotionPlan, '/plan_kinematic_path',
            callback_group=self.callback_group
        )

        self.move_group_client = ActionClient(
            self, MoveGroup, '/move_action',
            callback_group=self.callback_group
        )

        self.execute_client = ActionClient(
            self, ExecuteTrajectory, '/execute_trajectory',
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

        # ================================================================
        # Wait for services
        # ================================================================
        self.get_logger().info('Waiting for MoveIt services...')
        self.plan_client.wait_for_service(timeout_sec=30.0)
        self.move_group_client.wait_for_server(timeout_sec=30.0)
        self.execute_client.wait_for_server(timeout_sec=30.0)
        self.gripper_client.wait_for_server(timeout_sec=30.0)
        self.arm_trajectory_client.wait_for_server(timeout_sec=30.0)
        self.get_logger().info('All services ready!')

        # ================================================================
        # Subscribers
        # ================================================================
        # PointStamped (with frame_id) - 카메라 프레임 기준 좌표
        self.target_sub = self.create_subscription(
            PointStamped, '/target_point', self.target_stamped_callback, 10,
            callback_group=self.callback_group
        )

        # Point (without frame_id) - base_link 기준으로 가정
        self.target_point_sub = self.create_subscription(
            Point, '/target_position', self.target_point_callback, 10,
            callback_group=self.callback_group
        )

        # Status publisher
        self.status_pub = self.create_publisher(String, '/pick_place_status', 10)

        self.get_logger().info('='*60)
        self.get_logger().info('Pick and Place Controller Ready!')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Camera frame: {self.camera_frame}')
        self.get_logger().info(f'Target frame: {self.target_frame} (end-effector)')
        self.get_logger().info(f'Base frame: {self.base_frame}')
        self.get_logger().info(f'Place position (base_link): ({self.place_x}, {self.place_y}, {self.place_z})')
        self.get_logger().info('')
        self.get_logger().info('Input topics (all in CAMERA FRAME):')
        self.get_logger().info('  /target_point (PointStamped) - with frame_id')
        self.get_logger().info('  /target_position (Point) - assumes camera frame')
        self.get_logger().info('Transform: camera -> end-effector (relative movement)')
        self.get_logger().info('='*60)

    # ================================================================
    # Callbacks
    # ================================================================
    def joint_state_callback(self, msg: JointState):
        self.current_joint_state = msg

    def publish_status(self, message: str):
        msg = String()
        msg.data = message
        self.status_pub.publish(msg)
        self.get_logger().info(f'[Status] {message}')

    def target_stamped_callback(self, msg: PointStamped):
        """PointStamped 메시지 수신 (카메라 프레임 기준)"""
        if self._busy:
            self.get_logger().warn('Busy, ignoring new target')
            return

        # 중복 타겟 방지 (쿨다운 체크)
        current_time = self.get_clock().now()
        if self.last_target_time is not None:
            time_diff = (current_time - self.last_target_time).nanoseconds / 1e9
            if time_diff < self.target_cooldown:
                return  # 조용히 무시

        # 동일 위치 체크
        current_pos = np.array([msg.point.x, msg.point.y, msg.point.z])
        if self.last_target_position is not None:
            distance = np.linalg.norm(current_pos - self.last_target_position)
            if distance < self.position_threshold:
                return  # 조용히 무시

        self.get_logger().info(
            f'Received target: frame={msg.header.frame_id} '
            f'pos=({msg.point.x:.3f}, {msg.point.y:.3f}, {msg.point.z:.3f})'
        )

        # Transform to gripper frame
        target_in_gripper = self.transform_to_gripper_frame(msg)
        if target_in_gripper is None:
            self.publish_status('ERROR: Failed to transform target to gripper frame')
            return

        self.get_logger().info(
            f'Target in gripper frame: ({target_in_gripper[0]:.3f}, {target_in_gripper[1]:.3f}, {target_in_gripper[2]:.3f})'
        )

        # 타겟 정보 업데이트
        self.last_target_time = current_time
        self.last_target_position = current_pos

        self._busy = True
        try:
            self.pick_and_place_sequence_relative(target_in_gripper[0], target_in_gripper[1], target_in_gripper[2])
        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')
            self.publish_status(f'ERROR: {str(e)}')
        finally:
            self._busy = False

    def target_point_callback(self, msg: Point):
        """Point 메시지 수신 (카메라 프레임 기준)"""
        if self._busy:
            self.get_logger().warn('Busy, ignoring new target')
            return

        self.get_logger().info(f'Received target (camera frame): ({msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f})')

        # Point를 PointStamped로 변환 (카메라 프레임)
        point_stamped = PointStamped()
        point_stamped.header.frame_id = self.camera_frame
        point_stamped.header.stamp = self.get_clock().now().to_msg()
        point_stamped.point = msg

        # gripper 프레임으로 변환
        target_in_gripper = self.transform_to_gripper_frame(point_stamped)
        if target_in_gripper is None:
            self.publish_status('ERROR: Failed to transform target to gripper frame')
            return

        self.get_logger().info(
            f'Target in gripper frame: ({target_in_gripper[0]:.3f}, {target_in_gripper[1]:.3f}, {target_in_gripper[2]:.3f})'
        )

        self._busy = True
        try:
            self.pick_and_place_sequence_relative(target_in_gripper[0], target_in_gripper[1], target_in_gripper[2])
        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')
            self.publish_status(f'ERROR: {str(e)}')
        finally:
            self._busy = False

    # ================================================================
    # TF Transform
    # ================================================================
    def transform_to_gripper_frame(self, point_msg: PointStamped):
        """카메라 프레임 좌표를 엔드이펙터(gripper) 프레임으로 변환"""
        try:
            # 이미 gripper 프레임이면 그대로 반환
            if point_msg.header.frame_id == self.target_frame:
                return np.array([point_msg.point.x, point_msg.point.y, point_msg.point.z])

            # 최신 시간으로 업데이트 (TF 시간 동기화 문제 해결)
            point_msg.header.stamp = self.get_clock().now().to_msg()

            # 카메라 프레임 -> 엔드이펙터 프레임
            point_in_gripper = self.tf_buffer.transform(
                point_msg,
                self.target_frame,
                timeout=rclpy.duration.Duration(seconds=2.0)
            )

            return np.array([
                point_in_gripper.point.x,
                point_in_gripper.point.y,
                point_in_gripper.point.z
            ])

        except TransformException as e:
            self.get_logger().error(f'Camera -> Gripper transform failed: {e}')
            return None

    # ================================================================
    # Motion Functions
    # ================================================================
    def move_to_pose(self, x: float, y: float, z: float, description: str = "") -> bool:
        """MoveIt으로 목표 위치로 이동 (MoveGroup Action 사용)"""
        self.publish_status(f'Moving to {description}: ({x:.3f}, {y:.3f}, {z:.3f})')

        goal = MoveGroup.Goal()
        goal.request.group_name = self.planning_group
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.3
        goal.request.max_acceleration_scaling_factor = 0.3

        # Workspace
        goal.request.workspace_parameters.header.frame_id = self.base_frame
        goal.request.workspace_parameters.min_corner.x = -1.0
        goal.request.workspace_parameters.min_corner.y = -1.0
        goal.request.workspace_parameters.min_corner.z = -0.5
        goal.request.workspace_parameters.max_corner.x = 1.0
        goal.request.workspace_parameters.max_corner.y = 1.0
        goal.request.workspace_parameters.max_corner.z = 1.0

        # Position constraint
        constraints = Constraints()
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = self.base_frame
        pos_constraint.link_name = self.end_effector_link
        pos_constraint.target_point_offset.x = 0.0
        pos_constraint.target_point_offset.y = 0.0
        pos_constraint.target_point_offset.z = 0.0

        bounding_volume = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.02]  # 2cm tolerance
        bounding_volume.primitives.append(sphere)

        sphere_pose = Pose()
        sphere_pose.position.x = x
        sphere_pose.position.y = y
        sphere_pose.position.z = z
        sphere_pose.orientation.w = 1.0
        bounding_volume.primitive_poses.append(sphere_pose)

        pos_constraint.constraint_region = bounding_volume
        pos_constraint.weight = 1.0
        constraints.position_constraints.append(pos_constraint)

        goal.request.goal_constraints.append(constraints)

        # Planning options
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        # Send goal
        future = self.move_group_client.send_goal_async(goal)

        # Wait for acceptance
        timeout = 10.0
        start = time.time()
        while not future.done() and (time.time() - start) < timeout:
            time.sleep(0.05)

        if not future.done():
            self.publish_status(f'TIMEOUT: Goal send for {description}')
            return False

        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.publish_status(f'REJECTED: Goal for {description}')
            return False

        self.get_logger().info(f'Goal accepted for {description}, executing...')

        # Wait for result
        result_future = goal_handle.get_result_async()
        timeout = 60.0
        start = time.time()
        while not result_future.done() and (time.time() - start) < timeout:
            time.sleep(0.1)

        if not result_future.done():
            self.publish_status(f'TIMEOUT: Execution for {description}')
            return False

        result = result_future.result()
        if result.result.error_code.val == MoveItErrorCodes.SUCCESS:
            self.publish_status(f'SUCCESS: {description}')
            return True
        else:
            self.publish_status(f'FAILED: {description} (error: {result.result.error_code.val})')
            return False

    def move_to_joint_positions(self, positions: list, description: str = "", duration_sec: float = 3.0) -> bool:
        """조인트 위치로 직접 이동"""
        self.publish_status(f'Moving joints to {description}')

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.arm_joints

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec % 1) * 1e9)
        )
        goal.trajectory.points.append(point)

        future = self.arm_trajectory_client.send_goal_async(goal)

        timeout = 5.0
        start = time.time()
        while not future.done() and (time.time() - start) < timeout:
            time.sleep(0.1)

        if not future.done():
            self.publish_status(f'TIMEOUT: Joint move goal send')
            return False

        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.publish_status(f'REJECTED: Joint move goal')
            return False

        result_future = goal_handle.get_result_async()
        timeout = duration_sec + 5.0
        start = time.time()
        while not result_future.done() and (time.time() - start) < timeout:
            time.sleep(0.1)

        if result_future.done():
            self.publish_status(f'SUCCESS: {description}')
            return True
        else:
            self.publish_status(f'TIMEOUT: {description}')
            return False

    def move_to_ready(self) -> bool:
        """Ready 자세로 이동"""
        return self.move_to_joint_positions(self.ready_positions, "ready pose", 3.0)

    def move_to_place(self) -> bool:
        """Place 자세로 이동"""
        return self.move_to_joint_positions(self.place_positions, "place pose", 3.0)

    def control_gripper(self, open: bool) -> bool:
        """그리퍼 제어"""
        action = "Opening" if open else "Closing"
        self.publish_status(f'{action} gripper')

        goal = GripperCommand.Goal()
        goal.command.position = self.gripper_open_position if open else self.gripper_close_position
        goal.command.max_effort = self.gripper_max_effort

        future = self.gripper_client.send_goal_async(goal)

        timeout = 5.0
        start = time.time()
        while not future.done() and (time.time() - start) < timeout:
            time.sleep(0.1)

        if not future.done():
            self.publish_status('TIMEOUT: Gripper goal send')
            return False

        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.publish_status('REJECTED: Gripper goal')
            return False

        result_future = goal_handle.get_result_async()
        timeout = 10.0
        start = time.time()
        while not result_future.done() and (time.time() - start) < timeout:
            time.sleep(0.1)

        self.publish_status(f'Gripper {"opened" if open else "closed"}')
        return True

    # ================================================================
    # Pick and Place Sequence (Relative to End-Effector)
    # ================================================================
    def pick_and_place_sequence_relative(self, rel_x: float, rel_y: float, rel_z: float):
        """
        엔드이펙터 기준 상대 좌표로 Pick and Place 수행
        
        Args:
            rel_x, rel_y, rel_z: 현재 엔드이펙터 기준 상대 좌표
        """
        self.publish_status('='*50)
        self.publish_status('Starting Relative Pick & Place')
        self.publish_status(f'Relative target (gripper frame): ({rel_x:.3f}, {rel_y:.3f}, {rel_z:.3f})')
        self.publish_status('='*50)

        # 1. 상대 위치로 이동 (엔드이펙터 기준)
        self.publish_status('[1/5] Moving to relative target position')
        success = self.move_relative_to_gripper(rel_x, rel_y, rel_z, "relative target")
        if not success:
            self.publish_status('FAILED: Could not reach relative target position')
            self.move_to_ready()
            return
        self.publish_status('[1/5] DONE - Relative target position reached')
        time.sleep(1.0)

        # 2. 그리퍼 닫기 (물체 잡기)
        self.publish_status('[2/5] Closing gripper (grasping)')
        success = self.control_gripper(open=False)
        if not success:
            self.publish_status('FAILED: Could not close gripper')
            self.move_to_ready()
            return
        self.publish_status('[2/5] DONE - Gripper closed')
        time.sleep(1.0)

        # 3. Place 자세로 이동
        self.publish_status('[3/5] Moving to place pose')
        success = self.move_to_place()
        if not success:
            self.publish_status('FAILED: Could not reach place pose')
            self.move_to_ready()
            return
        self.publish_status('[3/5] DONE - Place pose reached')
        time.sleep(1.0)

        # 4. 그리퍼 열기 (물체 놓기)
        self.publish_status('[4/5] Opening gripper (releasing)')
        success = self.control_gripper(open=True)
        if not success:
            self.publish_status('FAILED: Could not open gripper')
        self.publish_status('[4/5] DONE - Gripper opened')
        time.sleep(1.0)

        # 5. Ready 자세로 복귀
        self.publish_status('[5/5] Returning to ready pose')
        success = self.move_to_ready()
        if success:
            self.publish_status('[5/5] DONE - Ready pose reached')
        
        self.publish_status('='*50)
        self.publish_status('Relative Pick & Place COMPLETED!')
        self.publish_status('='*50)

    def move_relative_to_gripper(self, rel_x: float, rel_y: float, rel_z: float, description: str = "") -> bool:
        """엔드이펙터 기준 상대 좌표로 이동"""
        # TODO: 여기서 현재 엔드이펙터 위치를 가져와서 상대 좌표를 절대 좌표로 변환
        # 현재는 임시로 기존 move_to_pose 사용
        self.publish_status(f'Moving relative to gripper: ({rel_x:.3f}, {rel_y:.3f}, {rel_z:.3f})')
        
        # 임시: 상대 좌표를 base_link 기준으로 변환하여 사용
        # 실제로는 현재 엔드이펙터 위치 + 상대 좌표로 계산해야 함
        abs_x = 0.3 + rel_x  # 임시 기준점
        abs_y = 0.0 + rel_y
        abs_z = 0.2 + rel_z
        
        return self.move_to_pose(abs_x, abs_y, abs_z, description)

    # ================================================================
    # Pick and Place Sequence (Original - Absolute Coordinates)
    # ================================================================
    def pick_and_place_sequence(self, pick_x: float, pick_y: float, pick_z: float):
        """
        Pick and Place 시퀀스 실행 (ready 상태에서 시작)

        1. 목표 위치로 이동 (MoveIt)
        2. 그리퍼 닫기 (pick)
        3. place 자세로 이동 (조인트)
        4. 그리퍼 열기 (place)
        5. ready 자세로 복귀
        """
        self.publish_status('='*50)
        self.publish_status('Starting Pick & Place')
        self.publish_status(f'Target: ({pick_x:.3f}, {pick_y:.3f}, {pick_z:.3f})')
        self.publish_status('='*50)

        # 1. 목표 위치로 이동
        self.publish_status('[1/5] Moving to target position')
        success = self.move_to_pose(pick_x, pick_y, pick_z, "target")
        if not success:
            self.publish_status('FAILED: Could not reach target position')
            self.move_to_ready()
            return
        self.publish_status('[1/5] DONE - Target position reached')
        time.sleep(1.0)  # 안정화 대기

        # 2. 그리퍼 닫기 (물체 잡기)
        self.publish_status('[2/5] Closing gripper (grasping)')
        success = self.control_gripper(open=False)
        if not success:
            self.publish_status('FAILED: Could not close gripper')
            self.move_to_ready()
            return
        self.publish_status('[2/5] DONE - Gripper closed')
        time.sleep(1.0)  # 그리퍼 안정화 대기

        # 3. Place 자세로 이동
        self.publish_status('[3/5] Moving to place pose')
        success = self.move_to_place()
        if not success:
            self.publish_status('FAILED: Could not reach place pose')
            self.move_to_ready()
            return
        self.publish_status('[3/5] DONE - Place pose reached')
        time.sleep(1.0)  # 안정화 대기

        # 4. 그리퍼 열기 (물체 놓기)
        self.publish_status('[4/5] Opening gripper (releasing)')
        success = self.control_gripper(open=True)
        if not success:
            self.publish_status('FAILED: Could not open gripper')
        self.publish_status('[4/5] DONE - Gripper opened')
        time.sleep(1.0)  # 그리퍼 안정화 대기

        # 5. Ready 자세로 복귀
        self.publish_status('[5/5] Returning to ready pose')
        success = self.move_to_ready()
        if success:
            self.publish_status('[5/5] DONE - Ready pose reached')
        
        self.publish_status('='*50)
        self.publish_status('Pick & Place COMPLETED!')
        self.publish_status('='*50)


def main(args=None):
    rclpy.init(args=args)

    node = TopicPickAndPlace()

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
