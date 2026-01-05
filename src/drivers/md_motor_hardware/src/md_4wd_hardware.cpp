/**
 * ============================================================================
 * MD Motor Driver 4WD Hardware Interface Implementation
 * ============================================================================
 * 
 * 4륜 개별 제어용 Hardware Interface 구현
 * 2개의 MD 듀얼채널 모터 드라이버를 사용하여 4개 바퀴를 개별 제어
 * 
 * 기존 md_hardware.cpp를 기반으로 2개의 시리얼 포트 지원
 * 
 * [모터 드라이버 매핑]
 * - Front Driver (ID=1, port_front): CH1=Front Left, CH2=Front Right
 * - Rear Driver (ID=2, port_rear):  CH1=Rear Left,  CH2=Rear Right
 * 
 * ============================================================================
 */

#include "md_hardware/md_4wd_hardware.hpp"

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

/* ============================================================================
 * Lifecycle 콜백 함수들
 * ============================================================================ */

hardware_interface::CallbackReturn MD4WDHardware::on_init(
    const hardware_interface::HardwareInfo & info)
{
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
        "╔══════════════════════════════════════════════════════════════╗");
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
        "║      MD Motor 4WD Hardware Interface Initializing            ║");
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
        "╚══════════════════════════════════════════════════════════════╝");

    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // 시리얼 통신 파라미터
    port_front_ = info_.hardware_parameters.count("port_front") > 0 
            ? info_.hardware_parameters.at("port_front") : "/dev/ttyUSB0";
    port_rear_ = info_.hardware_parameters.count("port_rear") > 0 
            ? info_.hardware_parameters.at("port_rear") : "/dev/ttyUSB1";
    baudrate_ = info_.hardware_parameters.count("baudrate") > 0
                ? std::stoi(info_.hardware_parameters.at("baudrate")) : 19200;
    
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[PARAM] port_front = %s", port_front_.c_str());
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[PARAM] port_rear = %s", port_rear_.c_str());
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[PARAM] baudrate = %d", baudrate_);
    
    // 모터 드라이버 ID
    front_motor_id_ = info_.hardware_parameters.count("front_driver_id") > 0
                       ? std::stoi(info_.hardware_parameters.at("front_driver_id")) : 1;
    rear_motor_id_ = info_.hardware_parameters.count("rear_driver_id") > 0
                      ? std::stoi(info_.hardware_parameters.at("rear_driver_id")) : 2;
    id_mdui_ = info_.hardware_parameters.count("id_mdui") > 0
               ? std::stoi(info_.hardware_parameters.at("id_mdui")) : 184;
    id_mdt_ = info_.hardware_parameters.count("id_mdt") > 0
              ? std::stoi(info_.hardware_parameters.at("id_mdt")) : 183;

    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[PARAM] front_driver_id = %d", front_motor_id_);
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[PARAM] rear_driver_id = %d", rear_motor_id_);

    // 로봇 파라미터
    wheel_radius_ = info_.hardware_parameters.count("wheel_radius") > 0
                    ? std::stod(info_.hardware_parameters.at("wheel_radius")) : 0.098;
    wheel_separation_ = info_.hardware_parameters.count("wheel_separation") > 0
                        ? std::stod(info_.hardware_parameters.at("wheel_separation")) : 0.32;
    gear_ratio_ = info_.hardware_parameters.count("gear_ratio") > 0
                  ? std::stoi(info_.hardware_parameters.at("gear_ratio")) : 15;
    poles_ = info_.hardware_parameters.count("poles") > 0
             ? std::stoi(info_.hardware_parameters.at("poles")) : 10;

    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[PARAM] wheel_radius = %.4f m", wheel_radius_);
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[PARAM] gear_ratio = %d, poles = %d", gear_ratio_, poles_);

    // 엔코더 변환 계수
    ppr_ = poles_ * 3.0 * gear_ratio_;
    tick_to_rad_ = (2.0 * M_PI) / ppr_;

    // 조인트 검증 (4개)
    if (info_.joints.size() != NUM_WHEELS)
    {
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),
                     "[ERROR] Expected 4 joints but got %zu!", info_.joints.size());
        return hardware_interface::CallbackReturn::ERROR;
    }

    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        joint_names_.push_back(info_.joints[i].name);
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "  [%zu] Joint: %s", i, info_.joints[i].name.c_str());
    }

    // 버퍼 초기화
    hw_commands_.fill(0.0);
    hw_positions_.fill(0.0);
    hw_velocities_.fill(0.0);

    // 모터 상태 초기화
    initMotorState(front_motor_);
    initMotorState(rear_motor_);

    // 통신 상태 초기화
    initCommState(front_comm_);
    initCommState(rear_comm_);

    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[INIT] ✓ 4WD Initialization Complete!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MD4WDHardware::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[CONFIG] Opening serial ports...");

    // Front 시리얼 포트
    if (initSerial(serial_front_, port_front_) != SUCCESS_4WD)
    {
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "[CONFIG] ✗ Failed to open FRONT port!");
        return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[CONFIG] ✓ FRONT port opened: %s", port_front_.c_str());

    // Rear 시리얼 포트
    if (initSerial(serial_rear_, port_rear_) != SUCCESS_4WD)
    {
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "[CONFIG] ✗ Failed to open REAR port!");
        return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[CONFIG] ✓ REAR port opened: %s", port_rear_.c_str());

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MD4WDHardware::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[ACTIVATE] Activating 4WD motors...");

    if (!serial_front_.isOpen() || !serial_rear_.isOpen())
    {
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "[ACTIVATE] Serial ports not open!");
        return hardware_interface::CallbackReturn::ERROR;
    }

    hw_commands_.fill(0.0);
    front_motor_.init_motor = false;
    rear_motor_.init_motor = false;

    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[ACTIVATE] ✓ All 4 motors activated!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MD4WDHardware::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[DEACTIVATE] Emergency stop...");

    // 모터 정지 명령 (3회 전송) - ID=1 고정
    for (int i = 0; i < 3; ++i)
    {
        if (serial_front_.isOpen())
            putMdData(serial_front_, 1, 0, 0);
        if (serial_rear_.isOpen())
            putMdData(serial_rear_, 1, 0, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[DEACTIVATE] ✓ Motors stopped!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MD4WDHardware::on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[CLEANUP] Closing serial ports...");

    if (serial_front_.isOpen()) serial_front_.close();
    if (serial_rear_.isOpen()) serial_rear_.close();

    return hardware_interface::CallbackReturn::SUCCESS;
}

/* ============================================================================
 * Interface Export 함수들
 * ============================================================================ */

std::vector<hardware_interface::StateInterface>
MD4WDHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (size_t i = 0; i < NUM_WHEELS; ++i)
    {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
MD4WDHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (size_t i = 0; i < NUM_WHEELS; ++i)
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
    }

    return command_interfaces;
}

/* ============================================================================
 * 제어 루프 함수들
 * ============================================================================ */

hardware_interface::return_type MD4WDHardware::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    std::lock_guard<std::mutex> lock(mutex_);

    // Front 드라이버에서 데이터 수신
    receiveDataFromController(serial_front_, front_comm_, front_motor_);
    
    // Rear 드라이버에서 데이터 수신
    receiveDataFromController(serial_rear_, rear_comm_, rear_motor_);

    // Front Driver: FL(left), FR(right)
    updateWheelState(FRONT_LEFT, front_motor_.left_rpm, front_motor_.left_position,
                     front_motor_.left_last_tick, front_motor_.left_last_rad);
    updateWheelState(FRONT_RIGHT, front_motor_.right_rpm, front_motor_.right_position,
                     front_motor_.right_last_tick, front_motor_.right_last_rad);

    // Rear Driver: RL(left), RR(right)
    updateWheelState(REAR_LEFT, rear_motor_.left_rpm, rear_motor_.left_position,
                     rear_motor_.left_last_tick, rear_motor_.left_last_rad);
    updateWheelState(REAR_RIGHT, rear_motor_.right_rpm, rear_motor_.right_position,
                     rear_motor_.right_last_tick, rear_motor_.right_last_rad);

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type MD4WDHardware::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    std::lock_guard<std::mutex> lock(mutex_);

    // rad/s → RPM 변환 --> 바퀴 방향 이상하면 이거 수정
    int16_t fl_rpm = radPerSecToRpm(hw_commands_[FRONT_LEFT]);
    int16_t fr_rpm = radPerSecToRpm(hw_commands_[FRONT_RIGHT]);  
    int16_t rl_rpm = radPerSecToRpm(hw_commands_[REAR_LEFT]);
    int16_t rr_rpm = radPerSecToRpm(hw_commands_[REAR_RIGHT]);   

    // 디버그 로그
    static int log_counter = 0;
    if ((fl_rpm != 0 || fr_rpm != 0 || rl_rpm != 0 || rr_rpm != 0) && (++log_counter % 50 == 0))
    {
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),
            "[WRITE] Front: L=%d R=%d RPM, Rear: L=%d R=%d RPM",
            fl_rpm, fr_rpm, rl_rpm, rr_rpm);
    }

    // Front 드라이버에 명령 전송 (FL, FR) - ID=1 고정  -> 모터 채널이 반전되있는거같아서 이거 수정함
    putMdData(serial_front_, 1, fr_rpm, fl_rpm);   
    
    // Rear 드라이버에 명령 전송 (RL, RR) - ID=1 고정 (별도 시리얼이라 OK)
    putMdData(serial_rear_, 1, rl_rpm, rr_rpm);

    return hardware_interface::return_type::OK;
}


/* ============================================================================
 * 시리얼 통신 함수들
 * ============================================================================ */

int MD4WDHardware::initSerial(serial::Serial& serial, const std::string& port)
{
    try
    {
        serial.setPort(port);
        serial.setBaudrate(baudrate_);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(100);
        serial.setTimeout(timeout);
        serial.open();

        if (serial.isOpen())
        {
            return SUCCESS_4WD;
        }
    }
    catch (const serial::IOException & e)
    {
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "[SERIAL] IOException: %s", e.what());
    }
    catch (const std::exception & e)
    {
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "[SERIAL] Exception: %s", e.what());
    }

    return FAIL_4WD;
}

/**
 * putMdData - 모터 명령 패킷 전송 (md_hardware.cpp와 동일한 형식)
 */
int MD4WDHardware::putMdData(serial::Serial& serial, uint8_t motor_id, 
                              int16_t left_rpm, int16_t right_rpm)
{
    if (!serial.isOpen())
    {
        return FAIL_4WD;
    }

    uint8_t packet[MAX_PACKET_SIZE_4WD];
    uint8_t checksum = 0;

    // 패킷 헤더
    packet[0] = id_mdt_;              // RMID: 수신자 (183 = MDT)
    packet[1] = id_mdui_;             // TMID: 송신자 (184 = PC)
    packet[2] = motor_id;             // 모터 ID
    packet[3] = PID_PNT_VEL_CMD_4WD;  // PID: 207

    checksum = packet[0] + packet[1] + packet[2] + packet[3];

    // 듀얼 모터 속도 명령 (md_hardware.cpp와 동일)
    packet[4] = 7;  // DataSize

    IByte4WD left_bytes = short2Byte(left_rpm);
    IByte4WD right_bytes = short2Byte(right_rpm);

    packet[5] = 1;                 // D1 ENABLE (ID1 활성화)
    packet[6] = left_bytes.low;    // D1 RPM (L)
    packet[7] = left_bytes.high;   // D1 RPM (H)
    packet[8] = 1;                 // D2 ENABLE (ID2 활성화)
    packet[9] = right_bytes.low;   // D2 RPM (L)
    packet[10] = right_bytes.high; // D2 RPM (H)
    packet[11] = REQUEST_PNT_MAIN_DATA_4WD;  // 리턴 데이터 요청

    for (int i = 4; i <= 11; ++i)
    {
        checksum += packet[i];
    }

    packet[12] = ~checksum + 1;    // Checksum (2의 보수)

    try
    {
        serial.write(packet, 13);
        serial.flush();
    }
    catch (const serial::IOException & e)
    {
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),
                     "[WRITE] Serial error: %s", e.what());
        return FAIL_4WD;
    }

    return SUCCESS_4WD;
}

int MD4WDHardware::receiveDataFromController(serial::Serial& serial, CommState& comm, MotorState& motor)
{
    if (!serial.isOpen())
    {
        return FAIL_4WD;
    }

    size_t available = serial.available();
    if (available == 0)
    {
        return SUCCESS_4WD;
    }

    uint8_t buffer[MAX_DATA_SIZE_4WD];
    size_t read_size = std::min(available, static_cast<size_t>(MAX_DATA_SIZE_4WD));
    
    try
    {
        serial.read(buffer, read_size);
    }
    catch (const serial::IOException & e)
    {
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "[READ] Serial error: %s", e.what());
        return FAIL_4WD;
    }

    if (motor.init_motor)
    {
        motor.init_motor = false;
    }
    else
    {
        analyzeReceivedData(buffer, read_size, comm, motor);
    }

    return SUCCESS_4WD;
}

/**
 * analyzeReceivedData - 수신 패킷 분석
 */
int MD4WDHardware::analyzeReceivedData(uint8_t* buffer, size_t size, CommState& comm, MotorState& motor)
{
    for (size_t j = 0; j < size; ++j)
    {
        if (comm.packet_num >= MAX_PACKET_SIZE_4WD)
        {
            comm.step = 0;
            comm.packet_num = 0;
            comm.checksum = 0;
            continue;
        }

        switch (comm.step)
        {
            case 0:  // RMID/TMID 확인
                if (buffer[j] == id_mdui_ || buffer[j] == id_mdt_)
                {
                    comm.checksum += buffer[j];
                    comm.recv_buf[comm.packet_num++] = buffer[j];
                    comm.error_count = 0;
                    if (comm.packet_num >= 2) comm.step++;
                }
                else
                {
                    comm.step = 0;
                    comm.packet_num = 0;
                    comm.checksum = 0;
                    comm.error_count++;
                }
                break;

            case 1:  // 모터 ID 확인
                if (buffer[j] == 1 || buffer[j] == 2)
                {
                    comm.checksum += buffer[j];
                    comm.recv_buf[comm.packet_num++] = buffer[j];
                    comm.step++;
                    comm.error_count = 0;
                }
                else
                {
                    comm.step = 0;
                    comm.packet_num = 0;
                    comm.checksum = 0;
                    comm.error_count++;
                }
                break;

            case 2:  // PID 저장
                comm.checksum += buffer[j];
                comm.recv_buf[comm.packet_num++] = buffer[j];
                comm.step++;
                break;

            case 3:  // DataSize 저장
                comm.max_data_num = buffer[j];
                comm.data_num = 0;
                comm.checksum += buffer[j];
                comm.recv_buf[comm.packet_num++] = buffer[j];
                comm.step++;
                break;

            case 4:  // 데이터 수신
                comm.recv_buf[comm.packet_num++] = buffer[j];
                comm.checksum += buffer[j];
                comm.data_num++;

                if (comm.data_num >= MAX_DATA_SIZE_4WD)
                {
                    comm.step = 0;
                    comm.packet_num = 0;
                    comm.checksum = 0;
                    break;
                }

                if (comm.data_num >= comm.max_data_num)
                {
                    comm.step++;
                }
                break;

            case 5:  // 체크섬 확인
                comm.checksum += buffer[j];
                comm.recv_buf[comm.packet_num++] = buffer[j];

                if (comm.checksum == 0)
                {
                    comm.packet_ok = true;
                }

                comm.step = 0;
                comm.packet_num = 0;
                comm.checksum = 0;
                break;

            default:
                comm.step = 0;
                comm.packet_num = 0;
                comm.checksum = 0;
                break;
        }

        if (comm.packet_ok)
        {
            comm.packet_ok = false;
            processReceivedData(comm, motor);
        }

        if (comm.error_count >= 10)
        {
            comm.error_count = 0;
            comm.step = 0;
            comm.packet_num = 0;
            comm.checksum = 0;
        }
    }

    return SUCCESS_4WD;
}

/**
 * processReceivedData - 수신 데이터 처리 (md_hardware.cpp와 동일)
 */
void MD4WDHardware::processReceivedData(CommState& comm, MotorState& motor)
{
    uint8_t pid = comm.recv_buf[3];

    if (pid == PID_MAIN_DATA_4WD)
    {
        uint8_t* data = &comm.recv_buf[5];

        // 왼쪽 모터 데이터
        motor.left_rpm = byte2Short(data[0], data[1]);
        motor.left_position = byte2Long(data[2], data[3], data[4], data[5]);

        // 오른쪽 모터 데이터 (offset 8)
        motor.right_rpm = byte2Short(data[8], data[9]);
        motor.right_position = byte2Long(data[10], data[11], data[12], data[13]);

        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),
            "[READ] L_RPM=%d R_RPM=%d L_Pos=%d R_Pos=%d", 
            motor.left_rpm, motor.right_rpm, motor.left_position, motor.right_position);
    }
}

/* ============================================================================
 * 헬퍼 함수들
 * ============================================================================ */

void MD4WDHardware::initMotorState(MotorState& motor)
{
    motor.left_rpm = 0;
    motor.right_rpm = 0;
    motor.left_position = 0;
    motor.right_position = 0;
    motor.left_last_rad = 0;
    motor.right_last_rad = 0;
    motor.left_last_tick = 0;
    motor.right_last_tick = 0;
    motor.init_motor = true;
}

void MD4WDHardware::initCommState(CommState& comm)
{
    comm.step = 0;
    comm.packet_num = 0;
    comm.checksum = 0;
    comm.max_data_num = 0;
    comm.data_num = 0;
    comm.packet_ok = false;
    comm.error_count = 0;
}

void MD4WDHardware::updateWheelState(size_t wheel_idx, int16_t rpm, int32_t position,
                                      int32_t& last_tick, double& last_rad)
{
    int32_t diff_tick = position - last_tick;
    last_rad += diff_tick * tick_to_rad_;
    last_tick = position;
    
    hw_positions_[wheel_idx] = last_rad;
    hw_velocities_[wheel_idx] = rpmToRadPerSec(rpm);
}

/* ============================================================================
 * 단위 변환 함수들
 * ============================================================================ */

int16_t MD4WDHardware::radPerSecToRpm(double rad_per_sec)
{
    return static_cast<int16_t>(rad_per_sec * 60.0 / (2.0 * M_PI));
}

double MD4WDHardware::rpmToRadPerSec(int16_t rpm)
{
    return rpm * (2.0 * M_PI) / 60.0;
}

IByte4WD MD4WDHardware::short2Byte(int16_t value)
{
    IByte4WD result;
    result.low = value & 0xFF;
    result.high = (value >> 8) & 0xFF;
    return result;
}

int16_t MD4WDHardware::byte2Short(uint8_t low, uint8_t high)
{
    return static_cast<int16_t>(low | (high << 8));
}

int32_t MD4WDHardware::byte2Long(uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4)
{
    return static_cast<int32_t>(b1 | (b2 << 8) | (b3 << 16) | (b4 << 24));
}

}  // namespace md_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(md_hardware::MD4WDHardware, hardware_interface::SystemInterface)
