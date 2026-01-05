# FRBot Workspace

Mobile Manipulator ROS2 Humble 워크스페이스

## 메인 런치 파일

```bash
ros2 launch robot_bringup full_system_with_pickplace.launch.py
```

모든 시스템을 순차적으로 실행:
1. Mobile Manipulator MoveIt + Hardware (즉시)
2. Dual RealSense Cameras + YOLO (5초 후)
3. Web Interface (8초 후)
4. Pick & Place Controller (12초 후)

## 시스템 구성

| 컴포넌트 | 통신 방식 | 포트 |
|----------|-----------|------|
| 모바일 베이스 (4WD) | RS485 Serial | `/dev/ttyUSB0`, `/dev/ttyUSB1` |
| 매니퓰레이터 (4DOF) | USBCAN-UC12 | Channel 0 |
| 그리퍼 | Serial | `/dev/ttyACM0` |
| RealSense D405 | USB | Gripper Camera |
| RealSense D455 | USB | Navigation Camera |

## 패키지 구조

```
frbot_ws/src/
│
├── robot_bringup/                    # 메인 런치 패키지
│   ├── launch/
│   │   ├── full_system_with_pickplace.launch.py  # 메인 런치
│   │   ├── mobile_manipulator_moveit.launch.py   # MoveIt + Hardware
│   │   └── dual_realsense.launch.py              # 카메라 + YOLO
│   └── scripts/
│       └── topic_pick_place.py                   # Pick & Place 컨트롤러
│
├── robot_description/                # URDF 모델
│   ├── urdf/
│   │   ├── mobile_manipulator_usbcan_hardware.xacro  # 메인 URDF
│   │   ├── robot_core.xacro
│   │   ├── ros2_control_mobile_manipulator_usbcan.urdf.xacro
│   │   ├── arm_only/                 # 매니퓰레이터 URDF
│   │   └── mobile_only/              # 모바일 베이스 URDF
│   └── meshes/                       # 3D 메쉬
│
├── moveit/
│   └── mobile_manipulator_moveit_config/  # MoveIt 설정
│       ├── config/                   # SRDF, kinematics, controllers
│       └── launch/
│
├── drivers/
│   ├── myactuator_hardware/          # 매니퓰레이터 ros2_control
│   │   └── src/myactuator_hardware_interface.cpp
│   ├── md_motor_hardware/            # 모바일 베이스 ros2_control
│   │   └── src/md_4wd_hardware.cpp
│   ├── myactuator_rmd/               # USBCAN 드라이버 라이브러리
│   └── serial-ros2/                  # 시리얼 통신 라이브러리
│
├── robot_web_interface/              # 웹 인터페이스
│   ├── launch/web_interface.launch.py
│   └── www/                          # 웹 페이지
│
├── yolo_realsense/                   # YOLO 객체 감지
│   └── yolo_realsense/yolo_node.py
│
├── navigation/                       # (미사용) 네비게이션
│   ├── robot_nav2/
│   └── robot_slam/
│
└── simulation/                       # (미사용) 시뮬레이션
    └── robot_gazebo/
```

## 빌드

```bash
cd ~/frbot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 실행

### 전체 시스템
```bash
ros2 launch robot_bringup full_system_with_pickplace.launch.py
```

### 파라미터 옵션
```bash
ros2 launch robot_bringup full_system_with_pickplace.launch.py \
    rviz:=true \
    can_channel:=0 \
    gripper_port:=/dev/ttyACM0 \
    port_front:=/dev/ttyUSB0 \
    port_rear:=/dev/ttyUSB1 \
    enable_pickplace:=true
```

## Pick & Place 사용

YOLO가 객체를 감지하면 자동으로 `/target_point` 토픽 발행 → Pick & Place 수행

수동 테스트:
```bash
ros2 topic pub /target_point geometry_msgs/PointStamped \
    "{header: {frame_id: 'd405_optical_frame'}, point: {x: 0.3, y: 0.0, z: 0.5}}"
```

## 토픽

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/target_point` | `geometry_msgs/PointStamped` | Pick 목표 좌표 |
| `/camera/camera/color/image_rect_raw` | `sensor_msgs/Image` | D405 컬러 이미지 |
| `/camera/camera/depth/image_rect_raw` | `sensor_msgs/Image` | D405 깊이 이미지 |
| `/cmd_vel` | `geometry_msgs/Twist` | 모바일 베이스 속도 명령 |
