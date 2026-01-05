/**
 * ============================================================================
 * MD Motor Driver Hardware Interface Implementation
 * ============================================================================
 * 
 * MD 듀얼채널 모터 드라이버 ros2_control Hardware Interface 구현
 * 
 * [기존 com.cpp 함수들의 매핑]
 * - InitSerial()              → initSerial()
 * - PutMdData()               → putMdData()
 * - ReceiveDataFromController() → receiveDataFromController()
 * - AnalyzeReceivedData()     → analyzeReceivedData()
 * - MdReceiveProc()           → processReceivedData()
 * 
 * ============================================================================
 */

#include "md_hardware/md_hardware.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <errno.h>
#include <cstring>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace md_hardware
{

// 로거 이름 상수
static const char* LOGGER_NAME = "MDMotorHardware";

/* ============================================================================
 * 디버그/에러 로깅 헬퍼 함수들
 * ============================================================================ */

/**
 * 시리얼 포트 상태 출력
 */
void MDMotorHardware::printSerialStatus()
{
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
        "╔══════════════════════════════════════════════════════════════╗");
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
        "║              Serial Port Status Report                       ║");
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
        "╠══════════════════════════════════════════════════════════════╣");
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
        "║ Port:       %-47s ║", port_.c_str());
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
        "║ Baudrate:   %-47d ║", baudrate_);
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
        "║ Is Open:    %-47s ║", serial_.isOpen() ? "YES" : "NO");
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
        "╚══════════════════════════════════════════════════════════════╝");
}

/**
 * 통신 상태 출력
 */
void MDMotorHardware::printCommStatus()
{
    RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME), 
        "[COMM] Step: %d, PacketNum: %d, ErrorCount: %d, PacketOK: %s",
        comm_.step, comm_.packet_num, comm_.error_count, 
        comm_.packet_ok ? "true" : "false");
}

/**
 * 모터 상태 출력
 */
void MDMotorHardware::printMotorStatus()
{
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
        "╔══════════════════════════════════════════════════════════════╗");
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
        "║              Motor Status Report                             ║");
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
        "╠══════════════════════════════════════════════════════════════╣");
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
        "║ Left  RPM: %-6d | Position: %-10d | Rad: %8.4f     ║", 
        motor_.left_rpm, motor_.left_position, motor_.left_last_rad);
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
        "║ Right RPM: %-6d | Position: %-10d | Rad: %8.4f     ║", 
        motor_.right_rpm, motor_.right_position, motor_.right_last_rad);
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
        "╚══════════════════════════════════════════════════════════════╝");
}

/**
 * 에러 리포트 출력
 */
void MDMotorHardware::printErrorReport(const std::string& function_name, 
                                        const std::string& error_msg,
                                        int error_code)
{
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), 
        "╔══════════════════════════════════════════════════════════════╗");
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), 
        "║                    ⚠️  ERROR REPORT  ⚠️                        ║");
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), 
        "╠══════════════════════════════════════════════════════════════╣");
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), 
        "║ Function:   %-47s ║", function_name.c_str());
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), 
        "║ Error:      %-47s ║", error_msg.c_str());
    if (error_code != 0) {
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), 
            "║ Error Code: %-47d ║", error_code);
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), 
            "║ System:     %-47s ║", std::strerror(error_code));
    }
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), 
        "╠══════════════════════════════════════════════════════════════╣");
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), 
        "║ Possible Solutions:                                          ║");
    
    if (error_msg.find("serial") != std::string::npos || 
        error_msg.find("port") != std::string::npos) {
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), 
            "║  1. Check if device is connected: ls -la /dev/ttyMotor      ║");
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), 
            "║  2. Check permissions: sudo chmod 666 /dev/ttyMotor         ║");
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), 
            "║  3. Add user to dialout: sudo usermod -aG dialout $USER     ║");
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), 
            "║  4. Check USB connection and driver                         ║");
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), 
            "║  5. Try: dmesg | tail -20 to see kernel messages            ║");
    }
    
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), 
        "╚══════════════════════════════════════════════════════════════╝");
}

/* ============================================================================
 * Lifecycle 콜백 함수들
 * ============================================================================ */

/**
 * on_init - URDF 파라미터 로드 및 초기화
 */
hardware_interface::CallbackReturn MDMotorHardware::on_init(
    const hardware_interface::HardwareInfo & info)
{
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
        "╔══════════════════════════════════════════════════════════════╗");
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
        "║         MD Motor Hardware Interface Initializing             ║");
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
        "╚══════════════════════════════════════════════════════════════╝");

    // 부모 클래스 초기화
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        printErrorReport("on_init", "Parent class initialization failed", 0);
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[INIT] Loading parameters from URDF...");

    // ─────────────────────────────────────────────────────────────────────
    // URDF <ros2_control> 섹션에서 파라미터 로드
    // ─────────────────────────────────────────────────────────────────────
    
    // 시리얼 통신 파라미터
    port_ = info_.hardware_parameters.count("port") > 0 
            ? info_.hardware_parameters.at("port") : "/dev/ttyMotor";
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[PARAM] port = %s", port_.c_str());
    
    baudrate_ = info_.hardware_parameters.count("baudrate") > 0
                ? std::stoi(info_.hardware_parameters.at("baudrate")) : 57600;
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[PARAM] baudrate = %d", baudrate_);
    
    motor_id_ = info_.hardware_parameters.count("motor_id") > 0
                ? std::stoi(info_.hardware_parameters.at("motor_id")) : 1;
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[PARAM] motor_id = %d", motor_id_);
    
    id_mdui_ = info_.hardware_parameters.count("id_mdui") > 0
               ? std::stoi(info_.hardware_parameters.at("id_mdui")) : 184;
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[PARAM] id_mdui = %d", id_mdui_);
    
    id_mdt_ = info_.hardware_parameters.count("id_mdt") > 0
              ? std::stoi(info_.hardware_parameters.at("id_mdt")) : 183;
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[PARAM] id_mdt = %d", id_mdt_);

    // 로봇 파라미터
    wheel_radius_ = info_.hardware_parameters.count("wheel_radius") > 0
                    ? std::stod(info_.hardware_parameters.at("wheel_radius")) : 0.033;
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[PARAM] wheel_radius = %.4f m", wheel_radius_);
    
    wheel_separation_ = info_.hardware_parameters.count("wheel_separation") > 0
                        ? std::stod(info_.hardware_parameters.at("wheel_separation")) : 0.16;
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[PARAM] wheel_separation = %.4f m", wheel_separation_);
    
    gear_ratio_ = info_.hardware_parameters.count("gear_ratio") > 0
                  ? std::stoi(info_.hardware_parameters.at("gear_ratio")) : 15;
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[PARAM] gear_ratio = %d", gear_ratio_);
    
    poles_ = info_.hardware_parameters.count("poles") > 0
             ? std::stoi(info_.hardware_parameters.at("poles")) : 10;
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[PARAM] poles = %d", poles_);

    // ─────────────────────────────────────────────────────────────────────
    // 엔코더 변환 계수 계산
    // PPR = poles * 3(Hall U,V,W) * gear_ratio
    // ─────────────────────────────────────────────────────────────────────
    ppr_ = poles_ * 3.0 * gear_ratio_;
    tick_to_rad_ = (2.0 * M_PI) / ppr_;  // 1틱 = (2π / PPR) 라디안

    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[CALC] PPR = %.1f (poles=%d × 3 × gear_ratio=%d)", 
                ppr_, poles_, gear_ratio_);
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[CALC] Tick2Rad = %.6f rad/tick", tick_to_rad_);

    // ─────────────────────────────────────────────────────────────────────
    // 조인트 검증 (2개의 휠 조인트 필요)
    // ─────────────────────────────────────────────────────────────────────
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[INIT] Checking joints...");
    
    if (info_.joints.size() != 2)
    {
        printErrorReport("on_init", 
            "Expected 2 joints but got " + std::to_string(info_.joints.size()), 0);
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),
                     "[ERROR] URDF must define exactly 2 joints for diff_drive!");
        return hardware_interface::CallbackReturn::ERROR;
    }

    // 조인트 이름 저장
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[INIT] Found %zu joints:", info_.joints.size());
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        joint_names_.push_back(info_.joints[i].name);
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
                    "  [%zu] Joint: %s", i, info_.joints[i].name.c_str());
        
        // 인터페이스 확인
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
                    "       - Command interfaces: %zu", info_.joints[i].command_interfaces.size());
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
                    "       - State interfaces: %zu", info_.joints[i].state_interfaces.size());
    }

    // ─────────────────────────────────────────────────────────────────────
    // 버퍼 초기화
    // ─────────────────────────────────────────────────────────────────────
    hw_commands_.resize(2, 0.0);
    hw_positions_.resize(2, 0.0);
    hw_velocities_.resize(2, 0.0);

    // 통신 상태 초기화
    comm_.step = 0;
    comm_.packet_num = 0;
    comm_.checksum = 0;
    comm_.packet_ok = false;
    comm_.error_count = 0;

    // 모터 상태 초기화
    motor_.left_rpm = 0;
    motor_.right_rpm = 0;
    motor_.left_position = 0;
    motor_.right_position = 0;
    motor_.left_last_rad = 0;
    motor_.right_last_rad = 0;
    motor_.left_last_tick = 0;
    motor_.right_last_tick = 0;
    motor_.init_motor = true;

    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
        "╔══════════════════════════════════════════════════════════════╗");
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
        "║         ✓ Initialization Complete!                           ║");
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
        "╚══════════════════════════════════════════════════════════════╝");

    return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * on_configure - 시리얼 포트 열기
 */
hardware_interface::CallbackReturn MDMotorHardware::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
        "╔══════════════════════════════════════════════════════════════╗");
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
        "║              Configuring Serial Connection                   ║");
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
        "╚══════════════════════════════════════════════════════════════╝");

    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[CONFIG] Attempting to open serial port...");
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[CONFIG] Port: %s", port_.c_str());
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[CONFIG] Baudrate: %d", baudrate_);

    if (initSerial() != SUCCESS)
    {
        printErrorReport("on_configure", 
            "Failed to open serial port: " + port_, errno);
        return hardware_interface::CallbackReturn::ERROR;
    }

    printSerialStatus();
    
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
        "[CONFIG] ✓ Serial port configured successfully!");
    
    return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * on_activate - 모터 활성화
 */
hardware_interface::CallbackReturn MDMotorHardware::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
        "╔══════════════════════════════════════════════════════════════╗");
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
        "║                  Activating Motors                           ║");
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
        "╚══════════════════════════════════════════════════════════════╝");

    // 시리얼 포트 상태 확인
    if (!serial_.isOpen())
    {
        printErrorReport("on_activate", "Serial port is not open!", 0);
        return hardware_interface::CallbackReturn::ERROR;
    }

    // 초기 명령 0으로 설정
    hw_commands_[0] = 0.0;
    hw_commands_[1] = 0.0;

    // 모터 초기화 확인 (필요시 초기화 패킷 전송)
    motor_.init_motor = false;

    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[ACTIVATE] ✓ Motors activated!");
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[ACTIVATE] Ready to receive commands on /cmd_vel");
    
    return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * on_deactivate - 모터 비활성화 (토크 OFF)
 */
hardware_interface::CallbackReturn MDMotorHardware::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[DEACTIVATE] Stopping motors...");

    // 모터 정지 명령 전송
    if (serial_.isOpen())
    {
        putMdData(PID_PNT_VEL_CMD, motor_id_, 0, 0);
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[DEACTIVATE] Sent stop command (RPM=0, 0)");
    }
    else
    {
        RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME), 
            "[DEACTIVATE] Serial port not open, cannot send stop command!");
    }

    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[DEACTIVATE] ✓ Motors deactivated!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * on_cleanup - 시리얼 포트 닫기
 */
hardware_interface::CallbackReturn MDMotorHardware::on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[CLEANUP] Closing serial port...");

    if (serial_.isOpen())
    {
        serial_.close();
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[CLEANUP] Serial port closed.");
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[CLEANUP] Serial port was already closed.");
    }

    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[CLEANUP] ✓ Cleanup complete!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

/* ============================================================================
 * Interface Export 함수들
 * ============================================================================ */

std::vector<hardware_interface::StateInterface>
MDMotorHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    // 각 조인트의 position, velocity 상태 인터페이스 등록
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
        
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
MDMotorHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    // 각 조인트의 velocity 명령 인터페이스 등록
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
    }

    return command_interfaces;
}

/* ============================================================================
 * 제어 루프 함수들
 * ============================================================================ */

/**
 * read - 모터 피드백 읽기 (매 제어 주기 호출)
 */
hardware_interface::return_type MDMotorHardware::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    std::lock_guard<std::mutex> lock(mutex_);

    // 시리얼에서 데이터 수신
    receiveDataFromController(motor_.init_motor);

    // 엔코더 틱 → 라디안 변환 (누적 회전각)
    // 왼쪽 바퀴
    int32_t left_diff_tick = motor_.left_position - motor_.left_last_tick;
    motor_.left_last_rad += left_diff_tick * tick_to_rad_;
    motor_.left_last_tick = motor_.left_position;
    hw_positions_[0] = motor_.left_last_rad;

    // 오른쪽 바퀴
    int32_t right_diff_tick = motor_.right_position - motor_.right_last_tick;
    motor_.right_last_rad += right_diff_tick * tick_to_rad_;
    motor_.right_last_tick = motor_.right_position;
    hw_positions_[1] = motor_.right_last_rad;

    // RPM → rad/s 변환
    hw_velocities_[0] = rpmToRadPerSec(motor_.left_rpm);
    hw_velocities_[1] = rpmToRadPerSec(motor_.right_rpm);

    return hardware_interface::return_type::OK;
}

/**
 * write - 모터 명령 전송 (매 제어 주기 호출)
 */
hardware_interface::return_type MDMotorHardware::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    std::lock_guard<std::mutex> lock(mutex_);

    // rad/s → RPM 변환
    int16_t left_rpm = radPerSecToRpm(hw_commands_[0]);
    int16_t right_rpm = radPerSecToRpm(hw_commands_[1]);

    // 디버그: 명령 값 출력 (0이 아닐 때만)
    static int debug_count = 0;
    if ((left_rpm != 0 || right_rpm != 0) && (debug_count++ % 50 == 0)) {
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),
            "[WRITE] cmd: L=%.3f R=%.3f rad/s → RPM: L=%d R=%d",
            hw_commands_[0], hw_commands_[1], left_rpm, right_rpm);
    }

    // 듀얼 모터 속도 명령 전송
    putMdData(PID_PNT_VEL_CMD, motor_id_, left_rpm, right_rpm);

    return hardware_interface::return_type::OK;
}

/* ============================================================================
 * 시리얼 통신 함수들 (com.cpp 기반)
 * ============================================================================ */

/**
 * initSerial - 시리얼 포트 초기화
 */
int MDMotorHardware::initSerial()
{
    RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME), "[SERIAL] Starting initialization...");
    
    try
    {
        RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME), "[SERIAL] Setting port: %s", port_.c_str());
        serial_.setPort(port_);
        
        RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME), "[SERIAL] Setting baudrate: %d", baudrate_);
        serial_.setBaudrate(baudrate_);
        
        RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME), "[SERIAL] Setting timeout: 100ms");
        serial::Timeout timeout = serial::Timeout::simpleTimeout(100);
        serial_.setTimeout(timeout);
        
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[SERIAL] Opening port %s...", port_.c_str());
        serial_.open();

        if (serial_.isOpen())
        {
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),
                        "[SERIAL] ✓ Port opened successfully!");
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),
                        "[SERIAL]   - Port: %s", port_.c_str());
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),
                        "[SERIAL]   - Baudrate: %d", baudrate_);
            return SUCCESS;
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),
                        "[SERIAL] ✗ Port opened but isOpen() returned false!");
            return FAIL;
        }
    }
    catch (const serial::IOException & e)
    {
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),
                     "[SERIAL] ✗ IOException: %s", e.what());
        printErrorReport("initSerial", std::string("IOException: ") + e.what(), errno);
        return FAIL;
    }
    catch (const serial::SerialException & e)
    {
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),
                     "[SERIAL] ✗ SerialException: %s", e.what());
        printErrorReport("initSerial", std::string("SerialException: ") + e.what(), errno);
        return FAIL;
    }
    catch (const std::exception & e)
    {
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),
                     "[SERIAL] ✗ Exception: %s", e.what());
        printErrorReport("initSerial", std::string("Exception: ") + e.what(), errno);
        return FAIL;
    }

    return FAIL;
}

/**
 * putMdData - 모터 명령 패킷 전송
 * 
 * [PID_PNT_VEL_CMD (207) 패킷 구조]
 * ┌────────────────────────────────────────────────────────────────┐
 * │ Byte │ 내용          │ 값                                      │
 * ├────────────────────────────────────────────────────────────────┤
 * │ 0    │ RMID          │ 183 (MDT 모터드라이버)                   │
 * │ 1    │ TMID          │ 184 (PC)                                │
 * │ 2    │ ID            │ motor_id (1 또는 2)                     │
 * │ 3    │ PID           │ 207 (PID_PNT_VEL_CMD)                   │
 * │ 4    │ DataSize      │ 7                                       │
 * │ 5    │ D1_ENABLE     │ ID1 활성화 (1)                          │
 * │ 6    │ D1_RPM(L)     │ ID1 RPM 하위 바이트                     │
 * │ 7    │ D1_RPM(H)     │ ID1 RPM 상위 바이트                     │
 * │ 8    │ D2_ENABLE     │ ID2 활성화 (1)                          │
 * │ 9    │ D2_RPM(L)     │ ID2 RPM 하위 바이트                     │
 * │ 10   │ D2_RPM(H)     │ ID2 RPM 상위 바이트                     │
 * │ 11   │ REQUEST       │ 리턴 데이터 요청 (2)                    │
 * │ 12   │ Checksum      │ ~(합계) + 1                             │
 * └────────────────────────────────────────────────────────────────┘
 */
int MDMotorHardware::putMdData(uint8_t pid, uint8_t motor_id, 
                                int16_t left_rpm, int16_t right_rpm)
{
    if (!serial_.isOpen())
    {
        return FAIL;
    }

    uint8_t packet[MAX_PACKET_SIZE];
    size_t packet_size = 0;
    uint8_t checksum = 0;

    // 패킷 헤더
    packet[0] = id_mdt_;              // RMID: 수신자 (183 = MDT)
    packet[1] = id_mdui_;             // TMID: 송신자 (184 = PC)
    packet[2] = motor_id;             // 모터 ID
    packet[3] = pid;                  // PID

    checksum = packet[0] + packet[1] + packet[2] + packet[3];

    if (pid == PID_PNT_VEL_CMD)
    {
        // 듀얼 모터 속도 명령 (md_ws와 동일한 형식)
        packet[4] = 7;  // DataSize

        IByte left_bytes = short2Byte(left_rpm);
        IByte right_bytes = short2Byte(right_rpm);

        packet[5] = 1;                 // D1 ENABLE (ID1 활성화)
        packet[6] = left_bytes.low;    // D1 RPM (L)
        packet[7] = left_bytes.high;   // D1 RPM (H)
        packet[8] = 1;                 // D2 ENABLE (ID2 활성화)
        packet[9] = right_bytes.low;   // D2 RPM (L)
        packet[10] = right_bytes.high; // D2 RPM (H)
        packet[11] = REQUEST_PNT_MAIN_DATA;  // 리턴 데이터 요청

        for (int i = 4; i <= 11; ++i)
        {
            checksum += packet[i];
        }

        packet[12] = ~checksum + 1;    // Checksum (2의 보수)
        packet_size = 13;
    }
    else if (pid == PID_TQ_OFF)
    {
        // 토크 OFF
        packet[4] = 0;  // DataSize
        checksum += packet[4];
        packet[5] = ~checksum + 1;
        packet_size = 6;
    }
    else if (pid == PID_REQ_PID_DATA)
    {
        // 데이터 요청
        packet[4] = 1;  // DataSize
        packet[5] = PID_MAIN_DATA;  // 요청할 PID
        checksum += packet[4] + packet[5];
        packet[6] = ~checksum + 1;
        packet_size = 7;
    }

    // 패킷 전송
    try
    {
        serial_.write(packet, packet_size);
        serial_.flush();
    }
    catch (const serial::IOException & e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("MDMotorHardware"),
                     "Serial write error: %s", e.what());
        return FAIL;
    }

    return SUCCESS;
}

/**
 * receiveDataFromController - 시리얼 데이터 수신
 */
int MDMotorHardware::receiveDataFromController(bool init)
{
    if (!serial_.isOpen())
    {
        return FAIL;
    }

    size_t available = serial_.available();
    if (available == 0)
    {
        return SUCCESS;
    }

    uint8_t buffer[MAX_DATA_SIZE];
    size_t read_size = std::min(available, static_cast<size_t>(MAX_DATA_SIZE));
    
    try
    {
        serial_.read(buffer, read_size);
    }
    catch (const serial::IOException & e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("MDMotorHardware"),
                     "Serial read error: %s", e.what());
        return FAIL;
    }

    if (init)
    {
        // 초기화 모드: 모터 연결 확인
        if (read_size >= 3 && buffer[2] == motor_id_)
        {
            RCLCPP_INFO(rclcpp::get_logger("MDMotorHardware"),
                        "Motor ID %d initialized!", motor_id_);
            motor_.init_motor = false;
        }
    }
    else
    {
        // 정상 모드: 패킷 분석
        analyzeReceivedData(buffer, read_size);
    }

    return SUCCESS;
}

/**
 * analyzeReceivedData - 수신 패킷 분석 (상태 머신)
 * 
 * [상태 머신 흐름]
 * Step 0: RMID/TMID 확인 (183 또는 184)
 * Step 1: 모터 ID 확인
 * Step 2: PID 저장
 * Step 3: DataSize 저장
 * Step 4: 데이터 수신
 * Step 5: 체크섬 확인 → processReceivedData()
 */
int MDMotorHardware::analyzeReceivedData(uint8_t* buffer, size_t size)
{
    for (size_t j = 0; j < size; ++j)
    {
        if (comm_.packet_num >= MAX_PACKET_SIZE)
        {
            comm_.step = 0;
            comm_.packet_num = 0;
            comm_.checksum = 0;
            return FAIL;
        }

        switch (comm_.step)
        {
            case 0:  // RMID/TMID 확인
                if (buffer[j] == id_mdui_ || buffer[j] == id_mdt_)
                {
                    comm_.checksum += buffer[j];
                    comm_.recv_buf[comm_.packet_num++] = buffer[j];
                    comm_.error_count = 0;
                    
                    // RMID, TMID 둘 다 받으면 다음 단계
                    if (comm_.packet_num >= 2)
                    {
                        comm_.step++;
                    }
                }
                else
                {
                    comm_.step = 0;
                    comm_.packet_num = 0;
                    comm_.checksum = 0;
                    comm_.error_count++;
                }
                break;

            case 1:  // 모터 ID 확인
                if (buffer[j] == 1 || buffer[j] == 2)
                {
                    comm_.checksum += buffer[j];
                    comm_.recv_buf[comm_.packet_num++] = buffer[j];
                    comm_.step++;
                    comm_.error_count = 0;
                }
                else
                {
                    comm_.step = 0;
                    comm_.packet_num = 0;
                    comm_.checksum = 0;
                    comm_.error_count++;
                }
                break;

            case 2:  // PID 저장
                comm_.checksum += buffer[j];
                comm_.recv_buf[comm_.packet_num++] = buffer[j];
                comm_.step++;
                break;

            case 3:  // DataSize 저장
                comm_.max_data_num = buffer[j];
                comm_.data_num = 0;
                comm_.checksum += buffer[j];
                comm_.recv_buf[comm_.packet_num++] = buffer[j];
                comm_.step++;
                break;

            case 4:  // 데이터 수신
                comm_.recv_buf[comm_.packet_num++] = buffer[j];
                comm_.checksum += buffer[j];
                comm_.data_num++;

                if (comm_.data_num >= MAX_DATA_SIZE)
                {
                    comm_.step = 0;
                    comm_.packet_num = 0;
                    comm_.checksum = 0;
                    break;
                }

                if (comm_.data_num >= comm_.max_data_num)
                {
                    comm_.step++;
                }
                break;

            case 5:  // 체크섬 확인
                comm_.checksum += buffer[j];
                comm_.recv_buf[comm_.packet_num++] = buffer[j];

                if (comm_.checksum == 0)  // 체크섬 통과
                {
                    comm_.packet_ok = true;
                }

                comm_.step = 0;
                comm_.packet_num = 0;
                comm_.checksum = 0;
                break;

            default:
                comm_.step = 0;
                comm_.packet_num = 0;
                comm_.checksum = 0;
                break;
        }

        // 패킷 수신 완료 시 데이터 처리
        if (comm_.packet_ok)
        {
            comm_.packet_ok = false;
            processReceivedData();
        }

        // 통신 오류 처리 (10회 연속 오류 시 리셋)
        if (comm_.error_count >= 10)
        {
            comm_.error_count = 0;
            comm_.step = 0;
            comm_.packet_num = 0;
            comm_.checksum = 0;
            break;
        }
    }

    return SUCCESS;
}

/**
 * processReceivedData - 수신 데이터 처리 (RPM, Position 추출)
 * 
 * [PID_MAIN_DATA (193) 데이터 구조]
 * Data[0-1]: 왼쪽 RPM (2바이트)
 * Data[2-5]: 왼쪽 Position (4바이트)
 * Data[6-7]: 왼쪽 전류
 * Data[8-9]: 오른쪽 RPM (2바이트)
 * Data[10-13]: 오른쪽 Position (4바이트)
 * Data[14-15]: 오른쪽 전류
 */
void MDMotorHardware::processReceivedData()
{
    uint8_t pid = comm_.recv_buf[3];  // PID 위치

    if (pid == PID_MAIN_DATA)
    {
        // 데이터는 recv_buf[5]부터 시작 (헤더 5바이트 이후)
        uint8_t* data = &comm_.recv_buf[5];

        // 왼쪽 모터 데이터
        motor_.left_rpm = byte2Short(data[0], data[1]);
        motor_.left_position = byte2Long(data[2], data[3], data[4], data[5]);

        // 오른쪽 모터 데이터 (offset 8)
        motor_.right_rpm = byte2Short(data[8], data[9]);
        motor_.right_position = byte2Long(data[10], data[11], data[12], data[13]);
    }
}

/* ============================================================================
 * 단위 변환 함수들
 * ============================================================================ */

/**
 * radPerSecToRpm - rad/s → RPM 변환
 * RPM = (rad/s) × 60 / (2π)
 */
int16_t MDMotorHardware::radPerSecToRpm(double rad_per_sec)
{
    return static_cast<int16_t>(rad_per_sec * 60.0 / (2.0 * M_PI));
}

/**
 * rpmToRadPerSec - RPM → rad/s 변환
 * rad/s = RPM × (2π) / 60
 */
double MDMotorHardware::rpmToRadPerSec(int16_t rpm)
{
    return rpm * (2.0 * M_PI) / 60.0;
}

/**
 * tickToRad - 엔코더 틱 → 라디안 변환
 */
double MDMotorHardware::tickToRad(int32_t tick)
{
    return tick * tick_to_rad_;
}

/* ============================================================================
 * 바이트 변환 유틸리티
 * ============================================================================ */

IByte MDMotorHardware::short2Byte(int16_t value)
{
    IByte result;
    result.low = value & 0xFF;
    result.high = (value >> 8) & 0xFF;
    return result;
}

int16_t MDMotorHardware::byte2Short(uint8_t low, uint8_t high)
{
    return static_cast<int16_t>(low | (high << 8));
}

int32_t MDMotorHardware::byte2Long(uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4)
{
    return static_cast<int32_t>(b1 | (b2 << 8) | (b3 << 16) | (b4 << 24));
}

}  // namespace md_hardware

/* ============================================================================
 * 플러그인 등록
 * ============================================================================
 * 
 * ros2_control이 이 Hardware Interface를 찾을 수 있도록 등록합니다.
 * URDF에서 <plugin>md_hardware/MDMotorHardware</plugin>으로 참조
 * ============================================================================ */
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    md_hardware::MDMotorHardware,
    hardware_interface::SystemInterface
)
