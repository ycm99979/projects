#include "myactuator_hardware/myactuator_hardware_interface.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <cerrno>
#include <dlfcn.h>

#include <hardware_interface/types/hardware_interface_type_values.hpp>

// For serial communication (gripper)
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace myactuator_hardware
{

// ============================================================================
// Helper: Print formatted error/info boxes
// ============================================================================
static void printErrorBox(rclcpp::Logger logger, const std::string& title, 
                          const std::vector<std::string>& lines)
{
  RCLCPP_ERROR(logger, " ");
  RCLCPP_ERROR(logger, "╔══════════════════════════════════════════════════════════════╗");
  RCLCPP_ERROR(logger, "║  %-60s║", title.c_str());
  RCLCPP_ERROR(logger, "╠══════════════════════════════════════════════════════════════╣");
  for (const auto& line : lines) {
    RCLCPP_ERROR(logger, "║  %-60s║", line.c_str());
  }
  RCLCPP_ERROR(logger, "╚══════════════════════════════════════════════════════════════╝");
  RCLCPP_ERROR(logger, " ");
}

static void printInfoBox(rclcpp::Logger logger, const std::string& title,
                         const std::vector<std::string>& lines)
{
  RCLCPP_INFO(logger, " ");
  RCLCPP_INFO(logger, "┌──────────────────────────────────────────────────────────────┐");
  RCLCPP_INFO(logger, "│  %-60s│", title.c_str());
  RCLCPP_INFO(logger, "├──────────────────────────────────────────────────────────────┤");
  for (const auto& line : lines) {
    RCLCPP_INFO(logger, "│  %-60s│", line.c_str());
  }
  RCLCPP_INFO(logger, "└──────────────────────────────────────────────────────────────┘");
  RCLCPP_INFO(logger, " ");
}

// ============================================================================
// Destructor
// ============================================================================
MyActuatorHardwareInterface::~MyActuatorHardwareInterface()
{
  on_cleanup(rclcpp_lifecycle::State());
}

rclcpp::Logger MyActuatorHardwareInterface::getLogger() const
{
  return rclcpp::get_logger("MyActuator_USBCAN_HW");
}

// ============================================================================
// USBCAN Library Functions
// ============================================================================
bool MyActuatorHardwareInterface::loadUSBCANLibrary()
{
  lib_handle_ = dlopen("libusbcan.so", RTLD_NOW);
  if (!lib_handle_) {
    printErrorBox(getLogger(), "LIBRARY LOAD ERROR",
      {"Failed to load libusbcan.so: " + std::string(dlerror()),
       "",
       "Install the library:",
       "  sudo cp libusbcan.so /lib/",
       "  sudo ldconfig"});
    return false;
  }

  // Load function pointers
  VCI_OpenDevice_ = (VCI_OpenDevice_t)dlsym(lib_handle_, "VCI_OpenDevice");
  VCI_CloseDevice_ = (VCI_CloseDevice_t)dlsym(lib_handle_, "VCI_CloseDevice");
  VCI_InitCAN_ = (VCI_InitCAN_t)dlsym(lib_handle_, "VCI_InitCAN");
  VCI_StartCAN_ = (VCI_StartCAN_t)dlsym(lib_handle_, "VCI_StartCAN");
  VCI_ResetCAN_ = (VCI_ResetCAN_t)dlsym(lib_handle_, "VCI_ResetCAN");
  VCI_Transmit_ = (VCI_Transmit_t)dlsym(lib_handle_, "VCI_Transmit");
  VCI_Receive_ = (VCI_Receive_t)dlsym(lib_handle_, "VCI_Receive");
  VCI_GetReceiveNum_ = (VCI_GetReceiveNum_t)dlsym(lib_handle_, "VCI_GetReceiveNum");
  VCI_ReadBoardInfo_ = (VCI_ReadBoardInfo_t)dlsym(lib_handle_, "VCI_ReadBoardInfo");

  if (!VCI_OpenDevice_ || !VCI_CloseDevice_ || !VCI_InitCAN_ || 
      !VCI_StartCAN_ || !VCI_Transmit_ || !VCI_Receive_) {
    printErrorBox(getLogger(), "LIBRARY SYMBOL ERROR",
      {"Failed to load required functions from libusbcan.so",
       "The library may be incompatible or corrupted"});
    dlclose(lib_handle_);
    lib_handle_ = nullptr;
    return false;
  }

  RCLCPP_INFO(getLogger(), "✓ libusbcan.so loaded successfully");
  return true;
}

void MyActuatorHardwareInterface::unloadUSBCANLibrary()
{
  if (lib_handle_) {
    dlclose(lib_handle_);
    lib_handle_ = nullptr;
  }
}

bool MyActuatorHardwareInterface::openUSBCAN()
{
  if (!VCI_OpenDevice_) return false;

  int ret = VCI_OpenDevice_(device_type_, device_index_, 0);
  if (ret != 1) {
    printErrorBox(getLogger(), "USBCAN OPEN ERROR",
      {"Failed to open USBCAN device",
       "Device type: " + std::to_string(device_type_),
       "Device index: " + std::to_string(device_index_),
       "",
       "Check:",
       "  □ Device is connected (lsusb | grep 0471)",
       "  □ udev rules are set up",
       "  □ User has permission"});
    return false;
  }

  device_opened_ = true;
  RCLCPP_INFO(getLogger(), "✓ USBCAN device opened");
  return true;
}

void MyActuatorHardwareInterface::closeUSBCAN()
{
  if (device_opened_ && VCI_CloseDevice_) {
    // Reset channels first
    resetCANChannel(can_channel_);
    VCI_CloseDevice_(device_type_, device_index_);
    device_opened_ = false;
    RCLCPP_INFO(getLogger(), "USBCAN device closed");
  }
}

bool MyActuatorHardwareInterface::initCANChannel(uint32_t channel)
{
  if (!VCI_InitCAN_) return false;

  ZCAN_CAN_INIT_CONFIG config;
  config.AccCode = 0;
  config.AccMask = 0xFFFFFFFF;
  config.Reserved = 0;
  config.Filter = 1;
  config.Timing0 = baudrate_timing_ & 0xFF;
  config.Timing1 = (baudrate_timing_ >> 8) & 0xFF;
  config.Mode = 0;  // Normal mode

  int ret = VCI_InitCAN_(device_type_, device_index_, channel, &config);
  if (ret != 1) {
    RCLCPP_ERROR(getLogger(), "Failed to init CAN channel %d", channel);
    return false;
  }

  RCLCPP_INFO(getLogger(), "✓ CAN channel %d initialized (timing: 0x%04X)", channel, baudrate_timing_);
  return true;
}

bool MyActuatorHardwareInterface::startCANChannel(uint32_t channel)
{
  if (!VCI_StartCAN_) return false;

  int ret = VCI_StartCAN_(device_type_, device_index_, channel);
  if (ret != 1) {
    RCLCPP_ERROR(getLogger(), "Failed to start CAN channel %d", channel);
    return false;
  }

  RCLCPP_INFO(getLogger(), "✓ CAN channel %d started", channel);
  return true;
}

bool MyActuatorHardwareInterface::resetCANChannel(uint32_t channel)
{
  if (!VCI_ResetCAN_) return false;
  VCI_ResetCAN_(device_type_, device_index_, channel);
  return true;
}

// ============================================================================
// CAN Communication
// ============================================================================
bool MyActuatorHardwareInterface::sendCANFrame(uint32_t id, const uint8_t* data, uint8_t len)
{
  if (!VCI_Transmit_) return false;

  ZCAN_CAN_OBJ msg;
  std::memset(&msg, 0, sizeof(msg));
  msg.ID = id;
  msg.SendType = 0;      // Normal send
  msg.RemoteFlag = 0;    // Data frame
  msg.ExternFlag = 0;    // Standard frame
  msg.DataLen = len;
  std::memcpy(msg.Data, data, len);

  int ret = VCI_Transmit_(device_type_, device_index_, can_channel_, &msg, 1);
  return ret == 1;
}

bool MyActuatorHardwareInterface::receiveCANFrame(uint32_t& id, uint8_t* data, uint8_t& len, int timeout_ms)
{
  if (!VCI_Receive_) return false;

  ZCAN_CAN_OBJ msg;
  int ret = VCI_Receive_(device_type_, device_index_, can_channel_, &msg, 1, timeout_ms);
  if (ret <= 0) return false;

  id = msg.ID;
  len = msg.DataLen;
  std::memcpy(data, msg.Data, len);
  return true;
}

int MyActuatorHardwareInterface::getReceiveCount()
{
  if (!VCI_GetReceiveNum_) return 0;
  return VCI_GetReceiveNum_(device_type_, device_index_, can_channel_);
}

// ============================================================================
// Motor Communication
// ============================================================================
bool MyActuatorHardwareInterface::readMotorState(MotorData & motor)
{
  // Send read motor status 2 command (0x9C)
  uint8_t cmd[8] = {0x9C, 0, 0, 0, 0, 0, 0, 0};
  uint32_t tx_id = 0x140 + motor.actuator_id;
  
  if (!sendCANFrame(tx_id, cmd, 8)) {
    return false;
  }

  // No sleep - poll immediately for response
  uint32_t rx_id;
  uint8_t rx_data[8];
  uint8_t rx_len;
  uint32_t expected_id = 0x240 + motor.actuator_id;

  // Try to receive response (minimal timeout)
  for (int i = 0; i < 20; i++) {
    if (receiveCANFrame(rx_id, rx_data, rx_len, 1)) {
      if (rx_id == expected_id && rx_data[0] == 0x9C && rx_len >= 8) {
        // Parse motor status 2 response
        // Byte 1: temperature
        // Byte 2-3: torque current (int16, 0.01A)
        // Byte 4-5: speed (int16, 1 dps)
        // Byte 6-7: shaft angle (int16, degrees, multi-turn capable)
        
        int16_t current_raw = static_cast<int16_t>((rx_data[3] << 8) | rx_data[2]);
        int16_t speed_raw = static_cast<int16_t>((rx_data[5] << 8) | rx_data[4]);
        // Byte 6-7: shaft angle (int16, degrees) - NOT raw encoder!
        // This matches the myactuator_rmd library's interpretation
        int16_t shaft_angle = static_cast<int16_t>((rx_data[7] << 8) | rx_data[6]);

        // Convert shaft angle (degrees) to radians
        motor.position_state = shaft_angle * M_PI / 180.0;
        motor.velocity_state = speed_raw * M_PI / 180.0;  // dps to rad/s
        motor.effort_state = current_raw * 0.01 * motor.torque_constant;
        
        return true;
      }
    }
  }

  return false;
}

bool MyActuatorHardwareInterface::writeMotorCommand(MotorData & motor)
{
  // Position control command (0xA4 = position closed-loop control 2)
  double position_deg = motor.position_command * 180.0 / M_PI;
  double velocity_dps = std::abs(motor.velocity_command) * 180.0 / M_PI;

  // Use configured velocity limits
  if (velocity_dps < 5.0) velocity_dps = default_velocity_dps_;
  if (velocity_dps > max_velocity_dps_) velocity_dps = max_velocity_dps_;

  int32_t angle_ctrl = static_cast<int32_t>(position_deg * 100);  // 0.01 deg units
  uint16_t speed_ctrl = static_cast<uint16_t>(velocity_dps);

  uint8_t cmd[8];
  cmd[0] = 0xA4;  // Position control command
  cmd[1] = 0x00;  // Spin direction
  cmd[2] = speed_ctrl & 0xFF;
  cmd[3] = (speed_ctrl >> 8) & 0xFF;
  cmd[4] = angle_ctrl & 0xFF;
  cmd[5] = (angle_ctrl >> 8) & 0xFF;
  cmd[6] = (angle_ctrl >> 16) & 0xFF;
  cmd[7] = (angle_ctrl >> 24) & 0xFF;

  uint32_t tx_id = 0x140 + motor.actuator_id;
  return sendCANFrame(tx_id, cmd, 8);
}

// ============================================================================
// Motor Acceleration Setting
// ============================================================================
bool MyActuatorHardwareInterface::setMotorAcceleration(int actuator_id, uint32_t accel_dps2)
{
  // 0x43: Write acceleration to RAM (position/velocity planning)
  // Data format: [0x43, 0x00, 0x00, 0x00, accel_L, accel_H, accel_HH, accel_HHH]
  // Acceleration unit: 1 dps/s (deg/s^2)

  uint8_t cmd[8];
  cmd[0] = 0x43;  // Write acceleration command
  cmd[1] = 0x00;
  cmd[2] = 0x00;
  cmd[3] = 0x00;
  cmd[4] = accel_dps2 & 0xFF;
  cmd[5] = (accel_dps2 >> 8) & 0xFF;
  cmd[6] = (accel_dps2 >> 16) & 0xFF;
  cmd[7] = (accel_dps2 >> 24) & 0xFF;

  uint32_t tx_id = 0x140 + actuator_id;
  if (!sendCANFrame(tx_id, cmd, 8)) {
    return false;
  }

  // Small delay for motor to process
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
  clearReceiveBuffer();

  return true;
}

// ============================================================================
// Utility Functions
// ============================================================================
void MyActuatorHardwareInterface::clearReceiveBuffer()
{
  // Clear any pending messages in receive buffer
  uint32_t id;
  uint8_t data[8];
  uint8_t len;
  int cleared = 0;
  while (receiveCANFrame(id, data, len, 1) && cleared < 100) {
    cleared++;
  }
}

// ============================================================================
// Optimized Batch Communication
// ============================================================================
void MyActuatorHardwareInterface::writeAllMotorCommands()
{
  // Send all motor commands without waiting for responses
  // This is faster than sending one-by-one
  for (auto & motor : motors_) {
    writeMotorCommand(motor);
  }
}

void MyActuatorHardwareInterface::readAllMotorStates()
{
  // Optimized batch read: Send all read requests first, then collect responses

  // Step 1: Send read commands to all motors at once
  for (auto & motor : motors_) {
    uint8_t cmd[8] = {0x9C, 0, 0, 0, 0, 0, 0, 0};
    uint32_t tx_id = 0x140 + motor.actuator_id;
    sendCANFrame(tx_id, cmd, 8);
  }

  // Step 2: Small delay to allow all responses to arrive
  std::this_thread::sleep_for(std::chrono::microseconds(500));

  // Step 3: Collect all responses
  uint32_t rx_id;
  uint8_t rx_data[8];
  uint8_t rx_len;

  size_t responses_needed = motors_.size();
  int attempts = 0;
  const int max_attempts = static_cast<int>(responses_needed * 15);

  while (responses_needed > 0 && attempts < max_attempts) {
    if (receiveCANFrame(rx_id, rx_data, rx_len, 1)) {
      // Check if this is a motor status response (0x9C)
      if (rx_data[0] == 0x9C && rx_len >= 8) {
        // Find which motor this response belongs to
        for (auto & motor : motors_) {
          uint32_t expected_id = 0x240 + motor.actuator_id;
          if (rx_id == expected_id) {
            // Parse motor status 2 response
            int16_t current_raw = static_cast<int16_t>((rx_data[3] << 8) | rx_data[2]);
            int16_t speed_raw = static_cast<int16_t>((rx_data[5] << 8) | rx_data[4]);
            int16_t shaft_angle = static_cast<int16_t>((rx_data[7] << 8) | rx_data[6]);

            motor.position_state = shaft_angle * M_PI / 180.0;
            motor.velocity_state = speed_raw * M_PI / 180.0;
            motor.effort_state = current_raw * 0.01 * motor.torque_constant;

            responses_needed--;
            break;
          }
        }
      }
    }
    attempts++;
  }
}

bool MyActuatorHardwareInterface::initializeMotor(
  const hardware_interface::ComponentInfo & joint, size_t index)
{
  MotorData & motor = motors_[index];
  motor.joint_name = joint.name;

  if (joint.parameters.find("actuator_id") == joint.parameters.end()) {
    printErrorBox(getLogger(), "CONFIGURATION ERROR",
      {"Joint '" + joint.name + "' missing 'actuator_id'",
       "",
       "Add to your URDF joint definition:",
       "  <param name=\"actuator_id\">1</param>"});
    return false;
  }
  motor.actuator_id = std::stoi(joint.parameters.at("actuator_id"));

  if (joint.parameters.find("torque_constant") != joint.parameters.end()) {
    motor.torque_constant = std::stod(joint.parameters.at("torque_constant"));
  } else {
    motor.torque_constant = 1.0;
  }

  RCLCPP_INFO(getLogger(), "  → Joint[%zu]: '%s' → Motor ID=%d, Kt=%.3f",
    index, motor.joint_name.c_str(), motor.actuator_id, motor.torque_constant);

  return true;
}

// ============================================================================
// on_init
// ============================================================================
hardware_interface::CallbackReturn MyActuatorHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Parse CAN channel (default: 0)
  if (info_.hardware_parameters.find("can_channel") != info_.hardware_parameters.end()) {
    can_channel_ = std::stoi(info_.hardware_parameters.at("can_channel"));
  }

  // Parse baudrate
  if (info_.hardware_parameters.find("baudrate") != info_.hardware_parameters.end()) {
    int baudrate = std::stoi(info_.hardware_parameters.at("baudrate"));
    switch (baudrate) {
      case 1000000: baudrate_timing_ = 0x1400; break;
      case 500000:  baudrate_timing_ = 0x1c00; break;
      case 250000:  baudrate_timing_ = 0x1c01; break;
      case 125000:  baudrate_timing_ = 0x1c03; break;
      default:      baudrate_timing_ = 0x1400; break;
    }
  }

  // Parse optional parameters
  if (info_.hardware_parameters.find("cycle_time") != info_.hardware_parameters.end()) {
    cycle_time_ = std::chrono::milliseconds(
      std::stol(info_.hardware_parameters.at("cycle_time")));
  }
  if (info_.hardware_parameters.find("auto_home") != info_.hardware_parameters.end()) {
    std::string auto_home_str = info_.hardware_parameters.at("auto_home");
    auto_home_ = (auto_home_str == "true" || auto_home_str == "True" || auto_home_str == "1");
  }

  // Parse motor dynamics parameters
  if (info_.hardware_parameters.find("motor_acceleration") != info_.hardware_parameters.end()) {
    motor_acceleration_ = std::stoul(info_.hardware_parameters.at("motor_acceleration"));
  }
  if (info_.hardware_parameters.find("motor_deceleration") != info_.hardware_parameters.end()) {
    motor_deceleration_ = std::stoul(info_.hardware_parameters.at("motor_deceleration"));
  }
  if (info_.hardware_parameters.find("max_velocity") != info_.hardware_parameters.end()) {
    max_velocity_dps_ = std::stod(info_.hardware_parameters.at("max_velocity"));
  }
  if (info_.hardware_parameters.find("default_velocity") != info_.hardware_parameters.end()) {
    default_velocity_dps_ = std::stod(info_.hardware_parameters.at("default_velocity"));
  }
  if (info_.hardware_parameters.find("enable_timing_log") != info_.hardware_parameters.end()) {
    std::string log_str = info_.hardware_parameters.at("enable_timing_log");
    enable_timing_log_ = (log_str == "true" || log_str == "True" || log_str == "1");
  }

  RCLCPP_INFO(getLogger(), "Motor dynamics: accel=%u dps², max_vel=%.0f dps, default_vel=%.0f dps",
    motor_acceleration_, max_velocity_dps_, default_velocity_dps_);

  // Parse gripper parameters
  if (info_.hardware_parameters.find("gripper_port") != info_.hardware_parameters.end()) {
    gripper_port_ = info_.hardware_parameters.at("gripper_port");
    if (gripper_port_ != "none" && !gripper_port_.empty()) {
      gripper_enabled_ = true;
    }
  }
  if (info_.hardware_parameters.find("gripper_baudrate") != info_.hardware_parameters.end()) {
    gripper_baudrate_ = std::stoi(info_.hardware_parameters.at("gripper_baudrate"));
  }

  // Count motors and gripper
  size_t motor_count = 0;
  for (const auto & joint : info_.joints) {
    if (joint.parameters.find("actuator_id") != joint.parameters.end()) {
      motor_count++;
    } else if (joint.name.find("gripper") != std::string::npos) {
      gripper_.joint_name = joint.name;
    }
  }

  motors_.resize(motor_count);
  RCLCPP_INFO(getLogger(), "Initializing %zu motors via USBCAN-UC12", motor_count);

  size_t idx = 0;
  for (const auto & joint : info_.joints) {
    if (joint.parameters.find("actuator_id") != joint.parameters.end()) {
      if (!initializeMotor(joint, idx++)) {
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  }

  RCLCPP_INFO(getLogger(), "✓ Configuration parsed successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}


// ============================================================================
// on_configure
// ============================================================================
hardware_interface::CallbackReturn MyActuatorHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(getLogger(), " ");
  RCLCPP_INFO(getLogger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  RCLCPP_INFO(getLogger(), "  Configuring MyActuator USBCAN Hardware Interface");
  RCLCPP_INFO(getLogger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");

  // Step 1: Load library
  RCLCPP_INFO(getLogger(), "[1/4] Loading libusbcan.so...");
  if (!loadUSBCANLibrary()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Step 2: Open device
  RCLCPP_INFO(getLogger(), "[2/4] Opening USBCAN device...");
  if (!openUSBCAN()) {
    unloadUSBCANLibrary();
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Step 3: Init and start CAN channel
  RCLCPP_INFO(getLogger(), "[3/4] Initializing CAN channel %d...", can_channel_);
  if (!initCANChannel(can_channel_) || !startCANChannel(can_channel_)) {
    closeUSBCAN();
    unloadUSBCANLibrary();
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Step 4: Test motor communication and set acceleration
  RCLCPP_INFO(getLogger(), "[4/5] Testing motor communication...");
  for (auto & motor : motors_) {
    RCLCPP_INFO(getLogger(), "  → Testing motor '%s' (ID=%d)...",
      motor.joint_name.c_str(), motor.actuator_id);

    if (readMotorState(motor)) {
      RCLCPP_INFO(getLogger(), "    ✓ Motor responded, position: %.3f rad (%.1f deg)",
        motor.position_state, motor.position_state * 180.0 / M_PI);
    } else {
      printErrorBox(getLogger(), "MOTOR COMMUNICATION FAILED",
        {"Motor: " + motor.joint_name,
         "ID: " + std::to_string(motor.actuator_id),
         "",
         "Checklist:",
         "  □ Motor power is ON",
         "  □ CAN wiring correct (CAN_H, CAN_L, GND)",
         "  □ Motor ID matches configuration",
         "  □ 120Ω termination resistor installed"});
      closeUSBCAN();
      unloadUSBCANLibrary();
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // Step 5: Set motor acceleration for smooth motion
  RCLCPP_INFO(getLogger(), "[5/5] Setting motor acceleration (%u dps²)...", motor_acceleration_);
  for (auto & motor : motors_) {
    if (setMotorAcceleration(motor.actuator_id, motor_acceleration_)) {
      RCLCPP_INFO(getLogger(), "  → Motor %d: acceleration set to %u dps²",
        motor.actuator_id, motor_acceleration_);
    } else {
      RCLCPP_WARN(getLogger(), "  → Motor %d: failed to set acceleration (non-critical)",
        motor.actuator_id);
    }
  }

  // Connect gripper if enabled
  if (gripper_enabled_) {
    if (!openGripperSerial()) {
      RCLCPP_WARN(getLogger(), "Gripper connection failed - continuing without gripper");
      gripper_enabled_ = false;
    }
  }

  // Start async thread
  startAsyncThread();

  printInfoBox(getLogger(), "CONFIGURATION SUCCESSFUL",
    {"Motors: " + std::to_string(motors_.size()) + " connected via USBCAN",
     "CAN Channel: " + std::to_string(can_channel_),
     "Baudrate: " + std::to_string(baudrate_timing_ == 0x1400 ? 1000000 : 
                                   baudrate_timing_ == 0x1c00 ? 500000 :
                                   baudrate_timing_ == 0x1c01 ? 250000 : 125000),
     "Gripper: " + std::string(gripper_enabled_ ? "enabled" : "disabled")});

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// on_cleanup
// ============================================================================
hardware_interface::CallbackReturn MyActuatorHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(getLogger(), "Cleaning up...");

  stopAsyncThread();
  stopHoldingThread();
  closeGripperSerial();
  closeUSBCAN();
  unloadUSBCANLibrary();

  RCLCPP_INFO(getLogger(), "✓ Cleanup complete");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// on_activate
// ============================================================================
hardware_interface::CallbackReturn MyActuatorHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(getLogger(), " ");
  RCLCPP_INFO(getLogger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  RCLCPP_INFO(getLogger(), "  Activating MyActuator USBCAN Hardware");
  RCLCPP_INFO(getLogger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");

  // Read current positions
  RCLCPP_INFO(getLogger(), "[1/3] Reading current positions...");
  for (auto & motor : motors_) {
    if (readMotorState(motor)) {
      RCLCPP_INFO(getLogger(), "  → '%s': %.3f rad (%.1f deg)", 
        motor.joint_name.c_str(), motor.position_state, motor.position_state * 180.0 / M_PI);
    }
  }

  // Move to home if enabled
  if (auto_home_) {
    RCLCPP_INFO(getLogger(), "[2/3] Moving to home position...");
    for (auto & motor : motors_) {
      motor.position_command = 0.0;
      motor.velocity_command = 50.0 * M_PI / 180.0;  // 50 deg/s
      writeMotorCommand(motor);
    }

    // Wait for motors to reach home
    bool all_home = false;
    for (int i = 0; i < 100 && !all_home; i++) {
      all_home = true;
      for (auto & motor : motors_) {
        readMotorState(motor);
        if (std::abs(motor.position_state) > 0.05) {
          all_home = false;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    RCLCPP_INFO(getLogger(), "  ✓ Motors at home position");

    // Wait 1 second before moving to ready position
    RCLCPP_INFO(getLogger(), "  Waiting 1 second before moving to ready position...");
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Move to ready position (singularity avoidance)
    // Ready pose from SRDF: link2_to_link1=0.0, link3_to_link2=-0.69813, link4_to_link3=-2.35619, gripper_to_link4=0.05236
    RCLCPP_INFO(getLogger(), "[2.5/3] Moving to ready position (singularity-free)...");
    const std::vector<double> ready_positions_rad = {0.0, -0.69813, -2.35619, 0.05236};
    
    for (size_t i = 0; i < motors_.size() && i < ready_positions_rad.size(); ++i) {
      motors_[i].position_command = ready_positions_rad[i];
      motors_[i].velocity_command = 50.0 * M_PI / 180.0;  // 50 deg/s
      writeMotorCommand(motors_[i]);
      RCLCPP_INFO(getLogger(), "  → '%s': moving to %.3f rad (%.1f deg)",
        motors_[i].joint_name.c_str(), ready_positions_rad[i], ready_positions_rad[i] * 180.0 / M_PI);
    }

    // Wait for motors to reach ready position
    bool all_ready = false;
    for (int i = 0; i < 100 && !all_ready; i++) {
      all_ready = true;
      for (size_t j = 0; j < motors_.size() && j < ready_positions_rad.size(); ++j) {
        readMotorState(motors_[j]);
        if (std::abs(motors_[j].position_state - ready_positions_rad[j]) > 0.05) {
          all_ready = false;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    RCLCPP_INFO(getLogger(), "  ✓ Motors at ready position");
  } else {
    RCLCPP_INFO(getLogger(), "[2/3] Auto-home disabled");
  }

  // Initialize commands
  RCLCPP_INFO(getLogger(), "[3/3] Initializing command values...");
  for (auto & motor : motors_) {
    motor.position_command = motor.position_state;
    motor.velocity_command = 0.0;
    motor.effort_command = 0.0;
  }
  gripper_.position_command = gripper_.position_state;

  RCLCPP_INFO(getLogger(), "✓ Activation complete");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// on_deactivate
// ============================================================================
hardware_interface::CallbackReturn MyActuatorHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(getLogger(), "Deactivating...");

  if (auto_home_) {
    RCLCPP_INFO(getLogger(), "Moving to home position...");
    for (auto & motor : motors_) {
      motor.position_command = 0.0;
      motor.velocity_command = 100.0 * M_PI / 180.0;
      writeMotorCommand(motor);
    }
    std::this_thread::sleep_for(std::chrono::seconds(3));
  }

  stopHoldingThread();
  startHoldingThread();

  RCLCPP_INFO(getLogger(), "✓ Deactivation complete");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// on_error
// ============================================================================
hardware_interface::CallbackReturn MyActuatorHardwareInterface::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_ERROR(getLogger(), "Hardware error occurred!");
  stopAsyncThread();
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// export_state_interfaces
// ============================================================================
std::vector<hardware_interface::StateInterface>
MyActuatorHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (auto & motor : motors_) {
    state_interfaces.emplace_back(
      motor.joint_name, hardware_interface::HW_IF_POSITION, &motor.position_state);
    state_interfaces.emplace_back(
      motor.joint_name, hardware_interface::HW_IF_VELOCITY, &motor.velocity_state);
    state_interfaces.emplace_back(
      motor.joint_name, hardware_interface::HW_IF_EFFORT, &motor.effort_state);
  }

  if (!gripper_.joint_name.empty()) {
    state_interfaces.emplace_back(
      gripper_.joint_name, hardware_interface::HW_IF_POSITION, &gripper_.position_state);
    state_interfaces.emplace_back(
      gripper_.joint_name, hardware_interface::HW_IF_VELOCITY, &gripper_.velocity_state);
  }

  return state_interfaces;
}

// ============================================================================
// export_command_interfaces
// ============================================================================
std::vector<hardware_interface::CommandInterface>
MyActuatorHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (auto & motor : motors_) {
    command_interfaces.emplace_back(
      motor.joint_name, hardware_interface::HW_IF_POSITION, &motor.position_command);
    command_interfaces.emplace_back(
      motor.joint_name, hardware_interface::HW_IF_VELOCITY, &motor.velocity_command);
    command_interfaces.emplace_back(
      motor.joint_name, hardware_interface::HW_IF_EFFORT, &motor.effort_command);
  }

  if (!gripper_.joint_name.empty()) {
    command_interfaces.emplace_back(
      gripper_.joint_name, hardware_interface::HW_IF_POSITION, &gripper_.position_command);
  }

  return command_interfaces;
}

// ============================================================================
// read
// ============================================================================
hardware_interface::return_type MyActuatorHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::lock_guard<std::mutex> lock(mutex_);

  auto start = std::chrono::high_resolution_clock::now();

  // Use optimized batch read
  readAllMotorStates();

  auto end = std::chrono::high_resolution_clock::now();
  last_read_time_ms_ = std::chrono::duration<double, std::milli>(end - start).count();

  // Update rolling average
  comm_count_++;
  avg_read_time_ms_ = avg_read_time_ms_ + (last_read_time_ms_ - avg_read_time_ms_) / comm_count_;

  // Log timing periodically (every 500 cycles)
  if (enable_timing_log_ && comm_count_ % 500 == 0) {
    RCLCPP_INFO(getLogger(), "[TIMING] read=%.2fms (avg=%.2fms), write=%.2fms (avg=%.2fms)",
      last_read_time_ms_, avg_read_time_ms_, last_write_time_ms_, avg_write_time_ms_);
  }

  gripper_.position_state = gripper_.position_command;

  return hardware_interface::return_type::OK;
}

// ============================================================================
// write
// ============================================================================
hardware_interface::return_type MyActuatorHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::lock_guard<std::mutex> lock(mutex_);

  auto start = std::chrono::high_resolution_clock::now();

  // Use optimized batch write
  writeAllMotorCommands();

  auto end = std::chrono::high_resolution_clock::now();
  last_write_time_ms_ = std::chrono::duration<double, std::milli>(end - start).count();

  // Update rolling average
  avg_write_time_ms_ = avg_write_time_ms_ + (last_write_time_ms_ - avg_write_time_ms_) / comm_count_;

  // Gripper control
  if (gripper_enabled_ && gripper_serial_fd_ >= 0) {
    int dxl_pos = positionToGripperDxl(gripper_.position_command);
    if (std::abs(dxl_pos - gripper_.last_dxl_command) > 10) {
      if (sendGripperCommand(dxl_pos)) {
        gripper_.last_dxl_command = dxl_pos;
      }
    }
  }

  return hardware_interface::return_type::OK;
}

// ============================================================================
// Thread management
// ============================================================================
bool MyActuatorHardwareInterface::startAsyncThread()
{
  stop_async_thread_.store(false);
  async_thread_ = std::thread(&MyActuatorHardwareInterface::asyncThreadFunc, this);
  return true;
}

void MyActuatorHardwareInterface::stopAsyncThread()
{
  stop_async_thread_.store(true);
  if (async_thread_.joinable()) {
    async_thread_.join();
  }
}

void MyActuatorHardwareInterface::asyncThreadFunc()
{
  while (!stop_async_thread_.load()) {
    std::this_thread::sleep_for(cycle_time_);
  }
}

bool MyActuatorHardwareInterface::startHoldingThread()
{
  stop_holding_thread_.store(false);
  holding_thread_ = std::thread(&MyActuatorHardwareInterface::holdingThreadFunc, this);
  return true;
}

void MyActuatorHardwareInterface::stopHoldingThread()
{
  stop_holding_thread_.store(true);
  if (holding_thread_.joinable()) {
    holding_thread_.join();
  }
}

void MyActuatorHardwareInterface::holdingThreadFunc()
{
  while (!stop_holding_thread_.load()) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto & motor : motors_) {
      motor.position_command = 0.0;
      motor.velocity_command = 10.0 * M_PI / 180.0;
      writeMotorCommand(motor);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
}

// ============================================================================
// Gripper functions
// ============================================================================
bool MyActuatorHardwareInterface::openGripperSerial()
{
  if (gripper_port_.empty() || gripper_port_ == "none") return false;

  gripper_serial_fd_ = open(gripper_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (gripper_serial_fd_ < 0) {
    RCLCPP_WARN(getLogger(), "Failed to open gripper port: %s", gripper_port_.c_str());
    return false;
  }

  struct termios tty;
  memset(&tty, 0, sizeof(tty));
  tcgetattr(gripper_serial_fd_, &tty);

  speed_t baud = B115200;
  cfsetospeed(&tty, baud);
  cfsetispeed(&tty, baud);

  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag |= CREAD | CLOCAL;

  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG);
  tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
  tty.c_oflag &= ~(OPOST | ONLCR);

  tcsetattr(gripper_serial_fd_, TCSANOW, &tty);
  
  RCLCPP_INFO(getLogger(), "✓ Gripper serial opened: %s", gripper_port_.c_str());
  return true;
}

void MyActuatorHardwareInterface::closeGripperSerial()
{
  if (gripper_serial_fd_ >= 0) {
    close(gripper_serial_fd_);
    gripper_serial_fd_ = -1;
  }
}

bool MyActuatorHardwareInterface::sendGripperCommand(int position)
{
  if (gripper_serial_fd_ < 0) return false;

  position = std::max(0, std::min(4095, position));
  uint8_t data[2] = {
    static_cast<uint8_t>((position >> 8) & 0xFF),
    static_cast<uint8_t>(position & 0xFF)
  };

  return ::write(gripper_serial_fd_, data, 2) == 2;
}

int MyActuatorHardwareInterface::positionToGripperDxl(double position_meters)
{
  double ratio = (position_meters - gripper_.min_position) / 
                 (gripper_.max_position - gripper_.min_position);
  ratio = std::max(0.0, std::min(1.0, ratio));
  
  return gripper_.min_dxl_position + 
         static_cast<int>(ratio * (gripper_.max_dxl_position - gripper_.min_dxl_position));
}

}  // namespace myactuator_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  myactuator_hardware::MyActuatorHardwareInterface,
  hardware_interface::SystemInterface)
