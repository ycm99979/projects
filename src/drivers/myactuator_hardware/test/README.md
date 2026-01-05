# MyActuator Hardware

ROS 2 ros2_control 기반 다중 모터 제어 패키지 for MyActuator RMD 시리즈.

MoveIt2의 JointTrajectoryController와 연동하여 다중 모터를 동시에 제어할 수 있습니다.

## 특징

- 다중 모터 지원: 단일 CAN 버스에서 여러 모터 동시 제어
- ros2_control 통합: SystemInterface 기반 하드웨어 인터페이스
- MoveIt2 호환: JointTrajectoryController action 서버 지원
- 시뮬레이션 지원: mock_components/GenericSystem으로 가상 테스트
- 실제 모터 mesh: X8ProV2 CAD 모델 포함

## 요구사항

### 하드웨어
- MyActuator RMD 시리즈 모터 (X8ProV2, X12_150 등)
- SocketCAN 호환 CAN 어댑터 (필수!)
  - 권장: CANable, MKS CANable PRO, PEAK PCAN-USB
  - 주의: USBCAN-UC12 (Philips 0471:1200)는 지원되지 않습니다

### 소프트웨어
- ROS 2 Humble
- ros2_control
- myactuator_rmd 드라이버 라이브러리

## 설치

    cd ~/mobile_manipulator_ws
    colcon build --packages-select myactuator_hardware
    source install/setup.bash

## 사용법

### 1. 시뮬레이션 (하드웨어 없이)

    # RViz 시뮬레이션 시작
    ros2 launch myactuator_hardware rviz_simulation.launch.py

    # 다른 터미널에서 예제 실행
    python3 src/arm/myactuator_hardware/examples/sine_wave_trajectory.py

### 2. 실제 하드웨어

    # CAN 인터페이스 설정
    sudo ip link set can0 type can bitrate 1000000
    sudo ip link set up can0

    # 실제 하드웨어로 실행
    ros2 launch myactuator_hardware multi_motor_control.launch.py ifname:=can0

## 예제 스크립트

### 사인파 Trajectory (examples/sine_wave_trajectory.py)

MoveIt2 방식의 action 기반 trajectory 전송:

    # 기본 wave 모드 (90도 위상차)
    python3 examples/sine_wave_trajectory.py --mode wave

    # 동기화 모드 (모든 모터 동시 움직임)
    python3 examples/sine_wave_trajectory.py --mode sync

    # 교차 모드 (대각선 쌍이 반대로)
    python3 examples/sine_wave_trajectory.py --mode alternate

    # 커스텀 파라미터
    python3 examples/sine_wave_trajectory.py -a 0.5 -f 0.5 -d 20.0

옵션:
  -a, --amplitude : 진폭 (라디안), 기본값 1.0
  -f, --frequency : 주파수 (Hz), 기본값 0.3
  -d, --duration  : 지속시간 (초), 기본값 10.0
  -m, --mode      : 모드 (wave, sync, alternate), 기본값 wave

### Trajectory 모드 설명

  wave      : 위상 차이 사인파, 1->2->3->4 순서로 90도 위상차
  sync      : 동기화 모션, 모든 모터 동시 움직임
  alternate : 대각선 교차, 모터 1,4 vs 2,3 반대 방향

### 실시간 위치 퍼블리셔 (examples/simple_position_pub.py)

    python3 examples/simple_position_pub.py

## 파일 구조

    myactuator_hardware/
    ├── config/
    │   ├── controllers.yaml          # 실제 하드웨어용
    │   ├── mock_controllers.yaml     # 시뮬레이션용
    │   └── view_robot.rviz
    ├── examples/
    │   ├── sine_wave_trajectory.py   # 사인파 trajectory 예제
    │   ├── simple_position_pub.py    # 실시간 위치 퍼블리셔
    │   └── README.md
    ├── include/myactuator_hardware/
    │   └── multi_motor_hardware_interface.hpp
    ├── launch/
    │   ├── multi_motor_control.launch.py   # 실제 하드웨어용
    │   └── rviz_simulation.launch.py       # 시뮬레이션용
    ├── meshes/visual/
    │   ├── X8ProV2/                  # X8ProV2 모터 mesh
    │   └── X12_150/                  # X12_150 모터 mesh
    ├── src/
    │   └── multi_motor_hardware_interface.cpp
    ├── urdf/
    │   ├── multi_motor_robot.urdf.xacro      # 실제 하드웨어 URDF
    │   ├── multi_motor_robot_mock.urdf.xacro # 시뮬레이션 URDF
    │   └── motor_macros.xacro
    ├── CMakeLists.txt
    ├── package.xml
    └── myactuator_hardware_plugin.xml

## URDF 구성 (4개 모터 평면 배치)

            Y
            ^
       M3   |   M1    (앞)
      ------+-------> X
       M4   |   M2    (뒤)

  joint_1: 앞쪽 왼쪽  (0.12, 0.12)
  joint_2: 앞쪽 오른쪽 (0.12, -0.12)
  joint_3: 뒤쪽 왼쪽  (-0.12, 0.12)
  joint_4: 뒤쪽 오른쪽 (-0.12, -0.12)

## ROS 2 인터페이스

### 토픽
  /joint_states                                    : sensor_msgs/JointState
  /joint_trajectory_controller/joint_trajectory    : trajectory_msgs/JointTrajectory

### 액션
  /joint_trajectory_controller/follow_joint_trajectory : control_msgs/FollowJointTrajectory

## 트러블슈팅

### 깔끔한 오류 메시지

하드웨어 인터페이스는 문제 진단을 위한 상세한 오류 메시지를 제공합니다:

```
╔══════════════════════════════════════════════════════════════╗
║  CAN INTERFACE NOT FOUND                                     ║
╠══════════════════════════════════════════════════════════════╣
║  Interface 'can0' not found!                                 ║
║  Available CAN interfaces: vcan0                             ║
║                                                              ║
║  To set up CAN interface:                                    ║
║    sudo ip link set can0 type can bitrate 1000000            ║
║    sudo ip link set can0 up                                  ║
╚══════════════════════════════════════════════════════════════╝
```

### 모터 연결 실패 시 체크리스트

```
╔══════════════════════════════════════════════════════════════╗
║  MOTOR CONNECTION FAILED                                     ║
╠══════════════════════════════════════════════════════════════╣
║  Checklist:                                                  ║
║    □ Motor power is ON                                       ║
║    □ CAN wiring correct (CAN_H, CAN_L, GND)                  ║
║    □ Motor ID matches configuration                          ║
║    □ CAN bitrate matches motor (usually 1Mbps)               ║
║    □ 120Ω termination resistor installed                     ║
╚══════════════════════════════════════════════════════════════╝
```

### CAN 인터페이스가 보이지 않음
    ip link show | grep can
    sudo modprobe can
    sudo modprobe can_raw

### 가상 CAN으로 테스트
    sudo modprobe vcan
    sudo ip link add dev vcan0 type vcan
    sudo ip link set up vcan0
    ros2 launch myactuator_hardware multi_motor_control.launch.py ifname:=vcan0

### Action 서버 연결 실패
    ros2 control list_controllers
    ros2 action list

## 참고

- myactuator_rmd: https://github.com/2b-t/myactuator_rmd
- myactuator_rmd_ros: https://github.com/2b-t/myactuator_rmd_ros
- ros2_control: https://control.ros.org/
