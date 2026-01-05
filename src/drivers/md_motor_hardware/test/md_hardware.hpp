/**
 * ============================================================================
 * MD Motor Driver Hardware Interface for ros2_control
 * ============================================================================
 * 
 * MD 듀얼채널 모터 드라이버를 ros2_control Hardware Interface로 구현
 * 
 * [아키텍처]
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │                        Controller Manager                               │
 * │  ┌──────────────────────────┐  ┌─────────────────────────────────────┐ │
 * │  │  diff_drive_controller   │  │    joint_state_broadcaster          │ │
 * │  └──────────┬───────────────┘  └────────────────┬────────────────────┘ │
 * │             │ cmd_vel                           │ joint_states         │
 * │             ▼                                   ▼                      │
 * │  ┌──────────────────────────────────────────────────────────────────┐  │
 * │  │              MDMotorHardware (이 클래스)                         │  │
 * │  │  - on_init(): 시리얼 포트 초기화 (InitSerial)                    │  │
 * │  │  - write(): RPM 명령 전송 (PutMdData)                           │  │
 * │  │  - read(): 엔코더/RPM 수신 (ReceiveDataFromController)          │  │
 * │  └──────────────────────────────────────────────────────────────────┘  │
 * └─────────────────────────────────────────────────────────────────────────┘
 *                              │ RS-485 Serial
 *                              ▼
 *                    ┌─────────────────────┐
 *                    │   MD Motor Driver   │
 *                    │   (Dual Channel)    │
 *                    └─────────────────────┘
 * 
 * [URDF 설정 예시]
 * <ros2_control name="md_motor_system" type="system">
 *   <hardware>
 *     <plugin>md_hardware/MDMotorHardware</plugin>
 *     <param name="port">/dev/ttyMotor</param>
 *     <param name="baudrate">57600</param>
 *     <param name="motor_id">1</param>
 *     <param name="gear_ratio">15</param>
 *     <param name="poles">10</param>
 *     <param name="wheel_radius">0.033</param>
 *     <param name="wheel_separation">0.16</param>
 *   </hardware>
 *   <joint name="left_wheel_joint">
 *     <command_interface name="velocity"/>
 *     <state_interface name="position"/>
 *     <state_interface name="velocity"/>
 *   </joint>
 *   <joint name="right_wheel_joint">
 *     <command_interface name="velocity"/>
 *     <state_interface name="position"/>
 *     <state_interface name="velocity"/>
 *   </joint>
 * </ros2_control>
 * 
 * ============================================================================
 */

#ifndef MD_HARDWARE__MD_HARDWARE_HPP_
#define MD_HARDWARE__MD_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <cmath>

// ros2_control
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

// Serial communication
#include "serial/serial.h"

namespace md_hardware
{

/* ============================================================================
 * 상수 정의 (MD Motor Protocol)
 * ============================================================================ */

// 통신 상태
constexpr int ON = 1;
constexpr int OFF = 0;
constexpr int SUCCESS = 1;
constexpr int FAIL = 0;

// PID (Protocol ID) - MD 모터 드라이버 명령
constexpr uint8_t PID_REQ_PID_DATA = 4;      // 데이터 요청
constexpr uint8_t PID_TQ_OFF = 5;            // 토크 OFF
constexpr uint8_t PID_COMMAND = 10;          // 일반 명령
constexpr uint8_t PID_POSI_RESET = 13;       // 위치 리셋
constexpr uint8_t PID_VEL_CMD = 130;         // 단일 모터 속도 명령
constexpr uint8_t PID_MAIN_DATA = 193;       // 메인 데이터 (피드백)
constexpr uint8_t PID_PNT_VEL_CMD = 207;     // 듀얼 모터 속도 명령 ★

// 패킷 크기
constexpr size_t MAX_PACKET_SIZE = 26;
constexpr size_t MAX_DATA_SIZE = 23;

// 데이터 요청 타입
constexpr uint8_t REQUEST_PNT_MAIN_DATA = 2;

/* ============================================================================
 * 바이트 변환 구조체
 * ============================================================================ */
struct IByte {
    uint8_t low;   // 하위 바이트 (LSB)
    uint8_t high;  // 상위 바이트 (MSB)
};

/* ============================================================================
 * MDMotorHardware 클래스
 * ============================================================================
 * 
 * ros2_control Hardware Interface 구현
 * MD 듀얼채널 모터 드라이버와 직접 시리얼 통신
 * ============================================================================ */
class MDMotorHardware : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(MDMotorHardware)

    /* ────────────────────────────────────────────────────────────────────────
     * Lifecycle 콜백 함수들
     * ──────────────────────────────────────────────────────────────────────── */
    
    /**
     * on_init - 초기화 (URDF 파라미터 로드)
     * 
     * URDF의 <ros2_control> 섹션에서 파라미터를 읽어옵니다.
     * - port, baudrate, motor_id 등
     */
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo & info) override;

    /**
     * on_configure - 설정 (시리얼 포트 열기)
     * 
     * InitSerial() 호출하여 시리얼 통신 초기화
     */
    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;

    /**
     * on_activate - 활성화 (모터 토크 ON)
     */
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;

    /**
     * on_deactivate - 비활성화 (모터 토크 OFF)
     */
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;

    /**
     * on_cleanup - 정리 (시리얼 포트 닫기)
     */
    hardware_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State & previous_state) override;

    /* ────────────────────────────────────────────────────────────────────────
     * Interface Export 함수들
     * ──────────────────────────────────────────────────────────────────────── */
    
    /**
     * export_state_interfaces - 상태 인터페이스 내보내기
     * 
     * diff_drive_controller와 joint_state_broadcaster가 읽을 상태값:
     * - position (rad): 엔코더 → 라디안 변환
     * - velocity (rad/s): RPM → rad/s 변환
     */
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    /**
     * export_command_interfaces - 명령 인터페이스 내보내기
     * 
     * diff_drive_controller가 쓸 명령값:
     * - velocity (rad/s): rad/s → RPM 변환 후 모터에 전송
     */
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    /* ────────────────────────────────────────────────────────────────────────
     * 제어 루프 함수들 (매 주기 호출)
     * ──────────────────────────────────────────────────────────────────────── */
    
    /**
     * read - 모터 피드백 읽기
     * 
     * ReceiveDataFromController() 호출하여 엔코더/RPM 수신
     * 수신된 값을 hw_positions_, hw_velocities_에 저장
     */
    hardware_interface::return_type read(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

    /**
     * write - 모터 명령 전송
     * 
     * PutMdData(PID_PNT_VEL_CMD, ...) 호출하여 속도 명령 전송
     * hw_commands_를 RPM으로 변환하여 전송
     */
    hardware_interface::return_type write(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    /* ────────────────────────────────────────────────────────────────────────
     * 시리얼 통신 함수들 (com.cpp에서 가져옴)
     * ──────────────────────────────────────────────────────────────────────── */
    
    /**
     * initSerial - 시리얼 포트 초기화
     * @return SUCCESS(1) 또는 FAIL(0)
     */
    int initSerial();

    /**
     * putMdData - 모터 명령 패킷 전송
     * @param pid: 프로토콜 ID (PID_PNT_VEL_CMD 등)
     * @param motor_id: 모터 ID
     * @param left_rpm: 왼쪽 모터 RPM
     * @param right_rpm: 오른쪽 모터 RPM
     */
    int putMdData(uint8_t pid, uint8_t motor_id, int16_t left_rpm, int16_t right_rpm);

    /**
     * receiveDataFromController - 시리얼 데이터 수신
     * @param init: ON이면 초기화 모드
     */
    int receiveDataFromController(bool init);

    /**
     * analyzeReceivedData - 수신 패킷 분석 (상태 머신)
     */
    int analyzeReceivedData(uint8_t* buffer, size_t size);

    /**
     * processReceivedData - 수신 데이터 처리 (RPM, Position 추출)
     */
    void processReceivedData();

    /* ────────────────────────────────────────────────────────────────────────
     * 단위 변환 함수들
     * ──────────────────────────────────────────────────────────────────────── */
    
    /**
     * radPerSecToRpm - rad/s → RPM 변환
     */
    int16_t radPerSecToRpm(double rad_per_sec);

    /**
     * rpmToRadPerSec - RPM → rad/s 변환
     */
    double rpmToRadPerSec(int16_t rpm);

    /**
     * tickToRad - 엔코더 틱 → 라디안 변환
     */
    double tickToRad(int32_t tick);

    /* ────────────────────────────────────────────────────────────────────────
     * 바이트 변환 유틸리티
     * ──────────────────────────────────────────────────────────────────────── */
    
    IByte short2Byte(int16_t value);
    int16_t byte2Short(uint8_t low, uint8_t high);
    int32_t byte2Long(uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4);

    /* ────────────────────────────────────────────────────────────────────────
     * 디버그/에러 로깅 함수들
     * ──────────────────────────────────────────────────────────────────────── */
    
    /**
     * printSerialStatus - 시리얼 포트 상태 출력
     */
    void printSerialStatus();
    
    /**
     * printCommStatus - 통신 상태 출력
     */
    void printCommStatus();
    
    /**
     * printMotorStatus - 모터 상태 출력
     */
    void printMotorStatus();
    
    /**
     * printErrorReport - 에러 리포트 출력
     * @param function_name: 에러 발생 함수명
     * @param error_msg: 에러 메시지
     * @param error_code: 시스템 에러 코드 (errno)
     */
    void printErrorReport(const std::string& function_name, 
                          const std::string& error_msg,
                          int error_code = 0);

    /* ────────────────────────────────────────────────────────────────────────
     * 멤버 변수들
     * ──────────────────────────────────────────────────────────────────────── */
    
    // 시리얼 통신 객체
    serial::Serial serial_;

    // ─────────────────────────────────────────────────────────────────────
    // 통신 파라미터 (URDF에서 로드)
    // ─────────────────────────────────────────────────────────────────────
    std::string port_;                // 시리얼 포트 (예: "/dev/ttyMotor")
    int baudrate_;                    // 통신 속도 (예: 57600)
    int motor_id_;                    // 모터 드라이버 ID (1 또는 2)
    int id_mdui_;                     // MDUI ID (184)
    int id_mdt_;                      // MDT ID (183)

    // ─────────────────────────────────────────────────────────────────────
    // 로봇 파라미터 (URDF에서 로드)
    // ─────────────────────────────────────────────────────────────────────
    double wheel_radius_;             // 바퀴 반경 (m)
    double wheel_separation_;         // 바퀴 간격 (m)
    int gear_ratio_;                  // 기어비
    int poles_;                       // 모터 극 수

    // ─────────────────────────────────────────────────────────────────────
    // 엔코더 변환 계수
    // ─────────────────────────────────────────────────────────────────────
    double ppr_;                      // Pulses Per Revolution (poles * 3 * gear_ratio)
    double tick_to_rad_;              // 틱 → 라디안 변환 계수

    // ─────────────────────────────────────────────────────────────────────
    // 통신 상태 (패킷 파싱용)
    // ─────────────────────────────────────────────────────────────────────
    struct {
        uint8_t send_buf[MAX_PACKET_SIZE];
        uint8_t recv_buf[MAX_PACKET_SIZE];
        uint8_t step;                 // 파싱 상태 머신 단계
        uint8_t packet_num;           // 현재 패킷 위치
        uint8_t checksum;             // 체크섬
        uint8_t max_data_num;         // 예상 데이터 크기
        uint8_t data_num;             // 현재 데이터 수
        bool packet_ok;               // 패킷 수신 완료 플래그
        uint8_t error_count;          // 통신 오류 카운트
    } comm_;

    // ─────────────────────────────────────────────────────────────────────
    // 모터 상태 (피드백)
    // ─────────────────────────────────────────────────────────────────────
    struct {
        int16_t left_rpm;             // 왼쪽 모터 RPM
        int16_t right_rpm;            // 오른쪽 모터 RPM
        int32_t left_position;        // 왼쪽 엔코더 위치
        int32_t right_position;       // 오른쪽 엔코더 위치
        double left_last_rad;         // 왼쪽 이전 라디안 (누적 계산용)
        double right_last_rad;        // 오른쪽 이전 라디안 (누적 계산용)
        int32_t left_last_tick;       // 왼쪽 이전 틱
        int32_t right_last_tick;      // 오른쪽 이전 틱
        bool init_motor;              // 초기화 상태
    } motor_;

    // ─────────────────────────────────────────────────────────────────────
    // Hardware Interface 버퍼 (ros2_control 연동)
    // ─────────────────────────────────────────────────────────────────────
    
    // 명령 버퍼 (Controller → Hardware)
    // [0] = left_wheel velocity (rad/s)
    // [1] = right_wheel velocity (rad/s)
    std::vector<double> hw_commands_;

    // 상태 버퍼 (Hardware → Controller)
    // [0] = left_wheel position (rad)
    // [1] = right_wheel position (rad)
    std::vector<double> hw_positions_;

    // [0] = left_wheel velocity (rad/s)
    // [1] = right_wheel velocity (rad/s)
    std::vector<double> hw_velocities_;

    // 조인트 이름 (URDF에서 로드)
    std::vector<std::string> joint_names_;

    // 스레드 안전을 위한 뮤텍스
    std::mutex mutex_;
};

}  // namespace md_hardware

#endif  // MD_HARDWARE__MD_HARDWARE_HPP_
