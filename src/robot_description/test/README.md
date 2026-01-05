# Robot Description Package

로봇의 URDF/Xacro 모델 정의 패키지

## 📁 폴더 구조

```
robot_description/
├── urdf/                          # URDF/Xacro 파일
│   ├── mobile_manipulator.xacro   # ★ 메인 로봇 모델 (통합)
│   ├── robot_base.urdf.xacro      # 모바일 베이스
│   ├── robot_core.xacro           # 매니퓰레이터 (4DOF 로봇팔)
│   ├── front_left_wheel.xacro     # 휠 정의
│   ├── front_right_wheel.xacro
│   ├── rear_left_wheel.xacro
│   ├── rear_right_wheel.xacro
│   ├── inertial_macros.xacro      # 관성 매크로
│   ├── material.xacro             # 재질/색상 정의
│   ├── ros2_control.urdf.xacro    # 실제 하드웨어 ros2_control
│   ├── ros2_control_sim.urdf.xacro # 시뮬레이션 ros2_control
│   │
│   │   # ★★ 통합 하드웨어 파일 (신규)
│   ├── ros2_control_mobile_manipulator.urdf.xacro  # Arm+Gripper+Mobile 통합 ros2_control
│   └── mobile_manipulator_full_hardware.xacro      # 통합 URDF (최상위)
│
├── meshes/                        # 3D 모델 파일 (STL/DAE)
│   └── (로봇 파츠 메시 파일들)
│
├── launch/
│   └── mobile_manipulator.launch.py  # 로봇 모델 시각화
│
├── rviz/
│   └── frbot.rviz                 # RViz 설정
│
├── config/
│   └── README.md
│
├── CMakeLists.txt
└── package.xml
```

---

## 🤖 로봇 구성

### 모바일 베이스 (4WD Skid-Steer)

```
       Front
  FL ─────── FR
   │         │
   │ [Base]  │
   │         │
  RL ─────── RR
       Rear
```

| 파라미터 | 값 |
|----------|-----|
| 휠 반지름 | 0.05 m |
| 휠 간격 (좌우) | 0.3 m |
| 휠베이스 (전후) | 0.3 m |
| 구동 방식 | 4륜 스키드 스티어 |

### 매니퓰레이터 (4DOF)

```
Base → Actuator1 → Actuator2 → Upperarm → Forearm → Gripper
```

---

## 📐 TF 트리

```
base_footprint
    └── base_link
         ├── front_left_wheel_link
         ├── front_right_wheel_link
         ├── rear_left_wheel_link
         ├── rear_right_wheel_link
         ├── arm_base_link
         │    └── actuator1
         │         └── actuator2
         │              └── upperarm
         │                   └── forearm
         ├── camera_link (RealSense)
         ├── imu_link
         └── velodyne_link (LiDAR)
```

---

## 🚀 사용법

### 로봇 모델 시각화

```bash
ros2 launch robot_description mobile_manipulator.launch.py
```

### URDF 검증

```bash
# URDF 파싱 확인
xacro /path/to/mobile_manipulator.xacro > robot.urdf
check_urdf robot.urdf
```

---

## 🔧 ros2_control 설정

### 실제 하드웨어 (`ros2_control.urdf.xacro`)

```xml
<plugin>md_hardware/MD4WDHardware</plugin>
```

### 시뮬레이션 (`ros2_control_sim.urdf.xacro`)

```xml
<plugin>ign_ros2_control/IgnitionSystem</plugin>
```

---

## 📝 새 센서 추가 방법

1. `robot_base.urdf.xacro`에 링크/조인트 추가
2. 센서 플러그인 설정 (Gazebo용)
3. TF 프레임 확인

```xml
<!-- 예: 새 카메라 추가 -->
<link name="new_camera_link">
  <visual>...</visual>
  <collision>...</collision>
</link>

<joint name="new_camera_joint" type="fixed">
  <parent link="base_link"/>
  <child link="new_camera_link"/>
  <origin xyz="0.1 0 0.1" rpy="0 0 0"/>
</joint>
```
