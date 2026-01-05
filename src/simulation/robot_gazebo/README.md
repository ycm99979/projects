# Robot Gazebo Package

Gazebo Fortress 시뮬레이션 환경 패키지

## 📁 폴더 구조

```
robot_gazebo/
├── worlds/                        # Gazebo 월드 파일
│   ├── husarion_office.sdf        # 사무실 환경 (실내)
│   └── empty.sdf                  # 빈 월드
│
├── models/                        # 커스텀 모델
│   └── (환경 오브젝트들)
│
├── launch/
│   └── frbot_gz_sim.launch.py     # ★ 시뮬레이션 런치
│
├── config/
│   └── gz_bridge.yaml             # ros_gz_bridge 설정
│
├── CMakeLists.txt
└── package.xml
```

---

## 🌍 월드 환경

### husarion_office.sdf
- 실내 사무실 환경
- 벽, 책상, 장애물 포함
- SLAM/Navigation 테스트에 적합

### empty.sdf
- 빈 평면 환경
- 기본 테스트용

---

## 🚀 사용법

### 기본 시뮬레이션 실행

```bash
ros2 launch robot_gazebo frbot_gz_sim.launch.py
```

### 월드 선택

```bash
ros2 launch robot_gazebo frbot_gz_sim.launch.py world:=husarion_office.sdf
ros2 launch robot_gazebo frbot_gz_sim.launch.py world:=empty.sdf
```

---

## 🔗 ros_gz_bridge 토픽

시뮬레이션과 ROS2 간 토픽 브릿지:

| Gazebo 토픽 | ROS2 토픽 | 방향 |
|-------------|-----------|------|
| `/clock` | `/clock` | GZ → ROS |
| `/model/frbot/cmd_vel` | `/cmd_vel` | ROS → GZ |
| `/lidar/points` | `/velodyne_points` | GZ → ROS |
| `/camera/image` | `/camera/image_raw` | GZ → ROS |
| `/imu` | `/imu/data` | GZ → ROS |

---

## ⚙️ 런치 파일 구조

`frbot_gz_sim.launch.py`가 실행하는 노드들:

1. **gz_sim** - Gazebo Fortress 시뮬레이터
2. **robot_state_publisher** - URDF 퍼블리시
3. **ros_gz_bridge** - 토픽 브릿지
4. **ros2_control_node** - 시뮬레이션 컨트롤러
5. **spawner** - diff_drive_controller, joint_state_broadcaster

---

## 🎮 로봇 조작

### 키보드 텔레옵

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/diff_drive_controller/cmd_vel_unstamped
```

### Joy 컨트롤러

```bash
ros2 launch teleop_twist_joy teleop-launch.py
```

---

## 🛠️ 새 월드 추가

1. `worlds/` 폴더에 `.sdf` 파일 생성
2. Gazebo Model Editor 또는 직접 SDF 작성
3. 런치 파일에서 월드 파라미터로 선택

```python
# launch 파일에서
world_file = os.path.join(pkg_path, 'worlds', 'my_world.sdf')
```

---

## 📝 센서 시뮬레이션

### LiDAR (Velodyne)
- Type: GPU Ray sensor
- Range: 0.1 ~ 30.0 m
- Points: 360° 스캔

### IMU
- 가속도계 + 자이로스코프
- Topic: `/imu/data`

### RealSense 카메라
- RGB + Depth
- Topics: `/camera/color/image_raw`, `/camera/depth/image_raw`
