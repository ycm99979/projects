# Robot Web Interface

Mobile Manipulator를 웹 브라우저에서 제어하는 인터페이스

## 기능

- **Mobile Base**: 조이스틱으로 이동 제어 (`/diff_drive_controller/cmd_vel_unstamped`)
- **Manipulator Arm**: Ready/Place 자세, 그리퍼 열기/닫기, 조인트 텔레옵
- **Pick & Place**: 좌표 입력으로 목표 전송
- **Camera**: RealSense 영상 스트리밍 (`/camera/camera/color/image_rect_raw`)
- **Joint States**: 실시간 조인트 상태 모니터링

## 설치

### 의존성 패키지 설치
```bash
sudo apt install ros-humble-rosbridge-server ros-humble-web-video-server
```

### 빌드
```bash
cd ~/frbot_ws
colcon build --packages-select robot_web_interface robot_bringup
source install/setup.bash
```

## 사용법

### 1. 로봇 시스템 실행
```bash
# Terminal 1: MoveIt + Hardware
ros2 launch robot_bringup mobile_manipulator_moveit.launch.py
```

### 2. 웹 인터페이스 실행
```bash
# Terminal 2: Web Interface
source install/setup.bash
ros2 launch robot_web_interface web_interface.launch.py
```

### 3. (선택) Arm Teleop 노드 실행
```bash
# Terminal 3: Arm Teleop
ros2 run robot_bringup arm_teleop.py
```

### 4. (선택) Pick & Place 노드 실행
```bash
# Terminal 4: Pick & Place
ros2 run robot_bringup topic_pick_place.py
```

## 웹 브라우저 접속

### 같은 PC에서 접속
```
http://localhost:8000
```

### 다른 PC/모바일에서 접속

1. 로봇 PC의 IP 확인:
```bash
hostname -I
# 예: 192.168.1.100
```

2. 브라우저에서 접속:
```
http://192.168.1.100:8000
```

### 포트 정보
| 포트 | 용도 |
|------|------|
| 8000 | 웹 인터페이스 (HTTP) |
| 9090 | rosbridge (WebSocket) |
| 8080 | 카메라 스트리밍 (web_video_server) |

## 웹 인터페이스 기능

### Mobile Base (조이스틱)
- 마우스/터치로 조이스틱 드래그
- 위/아래: 전진/후진
- 좌/우: 회전
- 놓으면 자동 정지

### Manipulator Arm
- **Ready 자세**: 싱귤러리티 회피 자세
- **Place 자세**: 물체 놓는 자세
- **그리퍼 열기/닫기**: 그리퍼 제어

### Joint Teleop (슬라이더)
- 각 조인트를 개별적으로 제어
- 슬라이더로 목표 각도 설정
- 실시간 현재 각도 표시

### Pick & Place
- X, Y, Z 좌표 입력 (카메라 프레임 기준)
- **목표 전송**: `/target_point` 토픽 발행
- **Pick & Place 시작**: 전체 시퀀스 실행

### Camera
- RealSense 카메라 영상 스트리밍
- 시작/정지 버튼

## 토픽 목록

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/diff_drive_controller/cmd_vel_unstamped` | Twist | 모바일 베이스 속도 명령 |
| `/arm_teleop/joint_cmd` | Float64MultiArray | 조인트 절대값 명령 |
| `/arm_teleop/joint_delta` | Float64MultiArray | 조인트 증감 명령 |
| `/arm_controller/joint_trajectory` | JointTrajectory | 조인트 궤적 명령 |
| `/target_point` | PointStamped | Pick & Place 목표 좌표 |
| `/target_position` | Point | Pick & Place 목표 (카메라 프레임) |
| `/joint_states` | JointState | 조인트 상태 |
| `/pick_place_status` | String | Pick & Place 상태 메시지 |
| `/camera/camera/color/image_rect_raw` | Image | 카메라 영상 |

## 문제 해결

### 연결 안 됨 (Disconnected)
- rosbridge가 실행 중인지 확인
- 방화벽에서 포트 9090 허용
```bash
sudo ufw allow 9090
sudo ufw allow 8000
sudo ufw allow 8080
```

### 카메라 안 보임
- web_video_server 실행 확인
- 카메라 토픽 확인: `ros2 topic list | grep image`

### 조인트 제어 안 됨
- arm_controller가 활성화되어 있는지 확인
- `ros2 control list_controllers`
