/**
 * ============================================================================
 * MD Motor Driver 4WD Hardware Interface for ros2_control
 * ============================================================================
 * 
 * 4륜 개별 제어용 Hardware Interface
 * 기존 md_hardware.hpp를 기반으로 2개의 시리얼 포트 지원
 * 
 * ============================================================================
 */

#ifndef MD_HARDWARE__MD_4WD_HARDWARE_HPP_
#define MD_HARDWARE__MD_4WD_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <array>
#include <mutex>
#include <cmath>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "serial/serial.h"

namespace md_hardware
{

// 상수 정의
constexpr int SUCCESS_4WD = 1;
constexpr int FAIL_4WD = 0;
constexpr size_t NUM_WHEELS = 4;

// 휠 인덱스
constexpr size_t FRONT_LEFT = 0;
constexpr size_t FRONT_RIGHT = 1;
constexpr size_t REAR_LEFT = 2;
constexpr size_t REAR_RIGHT = 3;

// PID (Protocol ID)
constexpr uint8_t PID_MAIN_DATA_4WD = 210;      // 0xD2 - 실제 드라이버 응답 PID
constexpr uint8_t PID_PNT_VEL_CMD_4WD = 207;

// 패킷 크기
constexpr size_t MAX_PACKET_SIZE_4WD = 26;
constexpr size_t MAX_DATA_SIZE_4WD = 23;

// 데이터 요청 타입
constexpr uint8_t REQUEST_PNT_MAIN_DATA_4WD = 2;

// 로거 이름
static const char* LOGGER_NAME = "MD4WDHardware";

// 바이트 변환 구조체
struct IByte4WD {
    uint8_t low;
    uint8_t high;
};

// 모터 상태 구조체
struct MotorState {
    int16_t left_rpm;
    int16_t right_rpm;
    int32_t left_position;
    int32_t right_position;
    double left_last_rad;
    double right_last_rad;
    int32_t left_last_tick;
    int32_t right_last_tick;
    bool init_motor;
};

// 통신 상태 구조체
struct CommState {
    uint8_t recv_buf[MAX_PACKET_SIZE_4WD];
    uint8_t step;
    uint8_t packet_num;
    uint8_t checksum;
    uint8_t max_data_num;
    uint8_t data_num;
    bool packet_ok;
    uint8_t error_count;
};

class MD4WDHardware : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(MD4WDHardware)

    // Lifecycle 콜백
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo & info) override;
    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State & previous_state) override;

    // Interface Export
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    // 제어 루프
    hardware_interface::return_type read(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type write(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    // 시리얼 통신 함수
    int initSerial(serial::Serial& serial, const std::string& port);
    int putMdData(serial::Serial& serial, uint8_t motor_id, int16_t left_rpm, int16_t right_rpm);
    int receiveDataFromController(serial::Serial& serial, CommState& comm, MotorState& motor);
    int analyzeReceivedData(uint8_t* buffer, size_t size, CommState& comm, MotorState& motor);
    void processReceivedData(CommState& comm, MotorState& motor);

    // 헬퍼 함수
    void initMotorState(MotorState& motor);
    void initCommState(CommState& comm);
    void updateWheelState(size_t wheel_idx, int16_t rpm, int32_t position,
                          int32_t& last_tick, double& last_rad);

    // 단위 변환
    int16_t radPerSecToRpm(double rad_per_sec);
    double rpmToRadPerSec(int16_t rpm);
    IByte4WD short2Byte(int16_t value);
    int16_t byte2Short(uint8_t low, uint8_t high);
    int32_t byte2Long(uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4);

    // 시리얼 객체
    serial::Serial serial_front_;
    serial::Serial serial_rear_;

    // 파라미터
    std::string port_front_;
    std::string port_rear_;
    int baudrate_;
    int front_motor_id_;
    int rear_motor_id_;
    int id_mdui_;
    int id_mdt_;
    double wheel_radius_;
    double wheel_separation_;
    int gear_ratio_;
    int poles_;
    double ppr_;
    double tick_to_rad_;

    // 모터/통신 상태
    MotorState front_motor_;
    MotorState rear_motor_;
    CommState front_comm_;
    CommState rear_comm_;

    // Hardware Interface 버퍼
    std::array<double, NUM_WHEELS> hw_commands_;
    std::array<double, NUM_WHEELS> hw_positions_;
    std::array<double, NUM_WHEELS> hw_velocities_;
    std::vector<std::string> joint_names_;

    std::mutex mutex_;
};

}  // namespace md_hardware

#endif  // MD_HARDWARE__MD_4WD_HARDWARE_HPP_
