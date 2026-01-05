# Robot Nav2 Package

Navigation2 자율 주행 패키지

## 📁 폴더 구조

```
robot_nav2/
├── config/
│   └── nav2_params.yaml          # ★ Navigation2 파라미터
│
├── launch/
│   ├── navigation.launch.py      # 네비게이션 런치 (맵 기반)
│   └── slam_navigation.launch.py # SLAM + 네비게이션 동시 실행
│
├── CMakeLists.txt
└── package.xml
```

---

## 🔗 TF 프레임 구조

### ⚠️ 중요: 프레임 통일

모든 패키지에서 **`base_footprint`**를 로봇 베이스 프레임으로 사용합니다.

```
map
 └── odom                      ← (SLAM/AMCL이 발행)
      └── base_footprint       ← robot_localization (EKF) 또는 diff_drive_controller가 발행
           └── base_link       ← robot_state_publisher가 발행
                └── sensors, wheels...
```

### 프레임 설정 요약

| 컴포넌트 | 파라미터 | 값 |
|----------|----------|-----|
| **nav2_params.yaml** | `base_frame_id` | `base_footprint` |
| **ekf.yaml** | `base_link_frame` | `base_footprint` |
| **diff_drive_controller** | `base_frame_id` | `base_footprint` |
| **URDF** | base frame | `base_footprint` → `base_link` |

### 토픽 연결

| 컴포넌트 | 입력 토픽 | 출력 토픽 |
|----------|-----------|-----------|
| AMCL | `/scan`, `/map` | TF (map→odom) |
| Global Planner | `/map`, `/goal_pose` | `/plan` |
| Local Planner | `/scan`, `/plan` | `/cmd_vel` |
| BT Navigator | `/goal_pose` | 액션 조정 |
| EKF | `/diff_drive_controller/odom`, `/imu` | `/odometry/filtered`, TF (odom→base_footprint) |

---

## ⚙️ 주요 파라미터 (nav2_params.yaml)

### AMCL (위치 추정)

```yaml
amcl:
  base_frame_id: "base_footprint"   # ★ base_footprint 사용
  odom_frame_id: "odom"
  global_frame_id: "map"
  scan_topic: scan
  robot_model_type: "nav2_amcl::DifferentialMotionModel"
```

### BT Navigator (행동 트리)

```yaml
bt_navigator:
  odom_topic: /odometry/filtered    # EKF 오도메트리 사용
  robot_base_frame: base_footprint  # ★ base_footprint 사용
```

### Costmap (공통)

```yaml
local_costmap:
  robot_base_frame: base_footprint  # ★ base_footprint 사용
  global_frame: odom

global_costmap:
  robot_base_frame: base_footprint  # ★ base_footprint 사용
  global_frame: map
```

---

## 🚀 사용법

### 저장된 맵으로 네비게이션

```bash
# 1. 맵 서버 + AMCL + Navigation 시작
ros2 launch robot_nav2 navigation.launch.py map:=/path/to/map.yaml

# 2. RViz에서 초기 위치 설정 (2D Pose Estimate)
# 3. 목표 지점 설정 (2D Goal Pose)
```

### SLAM + 네비게이션 동시 실행

```bash
# SLAM으로 맵 생성하면서 네비게이션
ros2 launch robot_nav2 slam_navigation.launch.py
```

---

## 📐 로봇 풋프린트

```yaml
# 4WD 로봇의 풋프린트 (사각형)
footprint: "[[0.2, 0.15], [0.2, -0.15], [-0.2, -0.15], [-0.2, 0.15]]"
```

---

## 🛠️ 코스트맵 설정

### Global Costmap

```yaml
global_costmap:
  robot_base_frame: base_footprint  # ★ 통일된 프레임
  global_frame: map
  resolution: 0.05
  plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
```

### Local Costmap

```yaml
local_costmap:
  robot_base_frame: base_footprint  # ★ 통일된 프레임
  global_frame: odom
  rolling_window: true
  width: 3
  height: 3
  resolution: 0.05
```

---

## 🔧 관련 설정 파일

### EKF (robot_bringup/config/ekf.yaml)

```yaml
ekf_filter_node:
  base_link_frame: base_footprint  # ★ Nav2와 일치
  odom_frame: odom
  publish_tf: true                  # odom→base_footprint TF 발행
```

### diff_drive_controller (robot_hardware/config/md_4wd_controllers.yaml)

```yaml
diff_drive_controller:
  base_frame_id: base_footprint    # ★ Nav2와 일치
  odom_frame_id: odom
  enable_odom_tf: true             # EKF 미사용 시
  # enable_odom_tf: false          # EKF 사용 시 (robot_bringup)
```

---

## 🎯 네비게이션 액션

### Python으로 목표 전송

```python
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped

navigator = BasicNavigator()

goal_pose = PoseStamped()
goal_pose.header.frame_id = 'map'
goal_pose.pose.position.x = 1.0
goal_pose.pose.position.y = 2.0

navigator.goToPose(goal_pose)
```

### CLI로 목표 전송

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 2.0}}}}"
```

---

## 🔧 트러블슈팅

### "Transform timeout" 에러
- TF가 정상 발행되는지 확인
- `ros2 run tf2_tools view_frames` 로 TF 트리 확인
- **base_footprint 프레임이 있는지 확인**

### 로봇이 장애물을 피하지 못함
- inflation_radius 증가
- cost_scaling_factor 조정

### 경로가 생성되지 않음
- 맵이 정상 로드됐는지 확인
- 목표 지점이 장애물 위가 아닌지 확인

### "Extrapolation Error" 발생
- use_sim_time 설정 확인 (시뮬/실제 환경 일치)
- TF 발행 주기가 충분한지 확인

---

## 📊 프레임 체크리스트

모든 설정 파일에서 프레임이 일치하는지 확인하세요:

| 파일 | 설정 | 올바른 값 |
|------|------|-----------|
| `nav2_params.yaml` | `base_frame_id` | `base_footprint` ✅ |
| `nav2_params.yaml` | `robot_base_frame` | `base_footprint` ✅ |
| `ekf.yaml` | `base_link_frame` | `base_footprint` ✅ |
| `md_4wd_controllers*.yaml` | `base_frame_id` | `base_footprint` ✅ |
| `frbot_controllers*.yaml` | `base_frame_id` | `base_footprint` ✅ |
| `cartographer.lua` | `tracking_frame` | `base_link` (내부) |
