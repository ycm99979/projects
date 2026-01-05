#!/usr/bin/env python3
"""
YOLO Visual Publisher Node (ROS Topic 기반)
- RealSense ROS 드라이버의 토픽을 구독하여 YOLO 추론
- 검출된 객체의 3D 좌표를 PointStamped로 발행
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import cv2
from ultralytics import YOLO
from message_filters import Subscriber, ApproximateTimeSynchronizer


class YoloVisualPublisher(Node):
    def __init__(self):
        super().__init__('yolo_visual_publisher')

        # Parameters
        self.declare_parameter('model_path', '/home/ycm/frbot_ws/best.pt')
        self.declare_parameter('camera_frame', 'd405_optical_frame')
        self.declare_parameter('target_topic', 'target_point')
        self.declare_parameter('color_topic', '/camera/camera/color/image_rect_raw')
        self.declare_parameter('depth_topic', '/camera/camera/depth/image_rect_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/depth/camera_info')
        self.declare_parameter('show_visualization', True)
        self.declare_parameter('confidence_threshold', 0.5)

        model_path = self.get_parameter('model_path').value
        self.camera_frame = self.get_parameter('camera_frame').value
        target_topic = self.get_parameter('target_topic').value
        color_topic = self.get_parameter('color_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        self.show_visualization = self.get_parameter('show_visualization').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value

        # CV Bridge
        self.bridge = CvBridge()

        # Camera intrinsics (from CameraInfo)
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.intrinsics_received = False

        # Publisher
        self.publisher_ = self.create_publisher(PointStamped, target_topic, 10)
        
        # Detection image publisher for web streaming
        self.detection_image_pub = self.create_publisher(Image, '/yolo/detection_image', 10)

        # QoS for camera topics
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # CameraInfo subscriber (한 번만 받으면 됨)
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.camera_info_callback,
            qos
        )

        # Synchronized color + depth subscribers
        self.color_sub = Subscriber(self, Image, color_topic, qos_profile=qos)
        self.depth_sub = Subscriber(self, Image, depth_topic, qos_profile=qos)

        self.sync = ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub],
            queue_size=5,
            slop=0.1
        )
        self.sync.registerCallback(self.image_callback)

        # YOLO 모델 로드
        self.get_logger().info(f"Loading YOLO model: {model_path}")
        self.model = YOLO(model_path)

        self.get_logger().info(f"YOLO Visual Publisher 시작")
        self.get_logger().info(f"  - Model: {model_path}")
        self.get_logger().info(f"  - Camera frame: {self.camera_frame}")
        self.get_logger().info(f"  - Target topic: {target_topic}")
        self.get_logger().info(f"  - Color topic: {color_topic}")
        self.get_logger().info(f"  - Depth topic: {depth_topic}")

    def camera_info_callback(self, msg: CameraInfo):
        """카메라 내부 파라미터 수신"""
        if not self.intrinsics_received:
            self.fx = msg.k[0]  # focal length x
            self.fy = msg.k[4]  # focal length y
            self.cx = msg.k[2]  # principal point x
            self.cy = msg.k[5]  # principal point y
            self.intrinsics_received = True
            self.get_logger().info(
                f"Camera intrinsics received: fx={self.fx:.2f}, fy={self.fy:.2f}, "
                f"cx={self.cx:.2f}, cy={self.cy:.2f}"
            )

    def deproject_pixel_to_point(self, u, v, depth):
        """픽셀 좌표를 3D 카메라 좌표로 변환"""
        if not self.intrinsics_received:
            return None

        x = (u - self.cx) * depth / self.fx
        y = (v - self.cy) * depth / self.fy
        z = depth
        return [x, y, z]

    def image_callback(self, color_msg: Image, depth_msg: Image):
        """동기화된 color + depth 이미지 처리"""
        if not self.intrinsics_received:
            self.get_logger().warn("Waiting for camera intrinsics...", throttle_duration_sec=2.0)
            return

        try:
            # ROS Image → OpenCV
            color_image = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")

            # depth 이미지 형식 확인 (16UC1 → meters 변환)
            if depth_image.dtype == np.uint16:
                depth_array = depth_image.astype(np.float32) / 1000.0  # mm → m
            else:
                depth_array = depth_image.astype(np.float32)

        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")
            return

        # YOLO 추론
        results = self.model(color_image, verbose=False)

        # 시각화용 이미지
        annotated_frame = color_image.copy()

        for result in results:
            for box in result.boxes:
                # Confidence 체크
                conf = float(box.conf[0])
                if conf < self.confidence_threshold:
                    continue

                # 중심점 좌표 추출
                x_c, y_c, w, h = box.xywh[0].cpu().numpy()
                ix, iy = int(x_c), int(y_c)

                # 이미지 범위 체크
                if ix < 0 or ix >= depth_array.shape[1] or iy < 0 or iy >= depth_array.shape[0]:
                    continue

                # 해당 좌표의 깊이 값
                depth = depth_array[iy, ix]

                # 유효한 깊이 값일 때만 발행
                if depth > 0.01 and depth < 2.0:  # D405 유효 범위
                    # 픽셀 → 3D 좌표 변환
                    point_3d = self.deproject_pixel_to_point(ix, iy, depth)

                    if point_3d is not None:
                        # ROS 2 토픽 발행 (PointStamped)
                        msg = PointStamped()
                        msg.header.stamp = color_msg.header.stamp
                        msg.header.frame_id = self.camera_frame
                        msg.point.x = float(point_3d[0])
                        msg.point.y = float(point_3d[1])
                        msg.point.z = float(point_3d[2])
                        self.publisher_.publish(msg)

                        self.get_logger().info(
                            f"Target: ({point_3d[0]:.3f}, {point_3d[1]:.3f}, {point_3d[2]:.3f})m, "
                            f"conf={conf:.2f}",
                            throttle_duration_sec=0.5
                        )

                # 시각화 - 항상 annotated_frame에 그리기 (웹 스트리밍용)
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(annotated_frame, (ix, iy), 5, (0, 0, 255), -1)

                class_name = self.model.names[int(box.cls[0])]
                if depth > 0.01:
                    point_3d = self.deproject_pixel_to_point(ix, iy, depth)
                    if point_3d:
                        label = f"{class_name}: X={point_3d[0]:.2f} Y={point_3d[1]:.2f} Z={point_3d[2]:.2f}m"
                    else:
                        label = f"{class_name}: {conf:.2f}"
                else:
                    label = f"{class_name}: no depth"

                cv2.putText(annotated_frame, label, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        # OpenCV 창 출력 (show_visualization이 True일 때만)
        if self.show_visualization:
            cv2.imshow('YOLOv8 RealSense D405', annotated_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info("Shutting down...")
                rclpy.shutdown()
        
        # Publish detection image for web streaming
        try:
            detection_msg = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
            detection_msg.header = color_msg.header
            self.detection_image_pub.publish(detection_msg)
        except Exception as e:
            self.get_logger().error(f"Detection image publish error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = YoloVisualPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
