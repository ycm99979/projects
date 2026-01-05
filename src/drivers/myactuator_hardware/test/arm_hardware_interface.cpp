#include "myactuator_hardware/arm_hardware_interface.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <fstream>
#include <cerrno>
#include <cstring>

#include <hardware_interface/types/hardware_interface_type_values.hpp>

// For CAN interface checking
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>

// For serial communication (gripper)
#include <fcntl.h>
#include <termios.h>

namespace myactuator_hardware
{

// ============================================================================
// Helper: Print formatted error box
// ============================================================================
void printArmErrorBox(rclcpp::Logger logger, const std::string& title, 
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

void printArmInfoBox(rclcpp::Logger logger, const std::string& title,
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
ArmHardwareInterface::~ArmHardwareInterface()
{
  if (auto_home_ && driver_) {
    RCLCPP_INFO(getLogger(), "Shutdown: Moving to home position...");
    
    for (auto & motor : motors_) {
      if (motor.actuator) {
        try {
          motor.actuator->sendPositionAbsoluteSetpoint(0.0f, 100.0f);
        } catch (...) {}
      }
    }
    
    for (int i = 0; i < 50; ++i) {
      bool all_at_home = true;
      for (auto & motor : motors_) {
        if (motor.actuator) {
          try {
            auto feedback = motor.actuator->getMotorStatus2();
            if (std::abs(feedback.shaft_angle) > 3.0f) {
              all_at_home = false;
            }
          } catch (...) {}
        }
      }
      if (all_at_home) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    for (auto & motor : motors_) {
      if (motor.actuator) {
        try {
          motor.actuator->sendPositionAbsoluteSetpoint(0.0f, 10.0f);
        } catch (...) {}
      }
    }
    
    RCLCPP_INFO(getLogger(), "Shutdown: Motors at home position (holding)");
  }
  
  // Close gripper serial
  closeGripperSerial();
  
  on_cleanup(rclcpp_lifecycle::State());
}

rclcpp::Logger ArmHardwareInterface::getLogger() const
{
  return rclcpp::get_logger("Arm_HW");
}

// ============================================================================
// on_init: Parse URDF parameters
// ============================================================================
hardware_interface::CallbackReturn ArmHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Parse CAN interface name
  if (info_.hardware_parameters.find("ifname") == info_.hardware_parameters.end()) {
    printArmErrorBox(getLogger(), "CONFIGURATION ERROR",
      {"Missing required parameter: 'ifname'",
       "",
       "Add to your ros2_control URDF:",
       "  <param name=\"ifname\">can0</param>"});
    return hardware_interface::CallbackReturn::ERROR;
  }
  ifname_ = info_.hardware_parameters.at("ifname");

  // Parse optional parameters
  if (info_.hardware_parameters.find("cycle_time") != info_.hardware_parameters.end()) {
    cycle_time_ = std::chrono::milliseconds(
      std::stol(info_.hardware_parameters.at("cycle_time")));
  }
  if (info_.hardware_parameters.find("timeout") != info_.hardware_parameters.end()) {
    timeout_ = std::chrono::milliseconds(
      std::stoi(info_.hardware_parameters.at("timeout")));
  }
  
  // Parse auto_home parameter
  if (info_.hardware_parameters.find("auto_home") != info_.hardware_parameters.end()) {
    std::string auto_home_str = info_.hardware_parameters.at("auto_home");
    auto_home_ = (auto_home_str == "true" || auto_home_str == "True" || auto_home_str == "1");
  } else {
    auto_home_ = true;
  }
  RCLCPP_INFO(getLogger(), "Auto-home on activate: %s", auto_home_ ? "enabled" : "disabled");

  // ========== Parse gripper parameters ==========
  if (info_.hardware_parameters.find("gripper_port") != info_.hardware_parameters.end()) {
    gripper_port_ = info_.hardware_parameters.at("gripper_port");
    gripper_enabled_ = true;
    RCLCPP_INFO(getLogger(), "Gripper enabled on port: %s", gripper_port_.c_str());
  }
  if (info_.hardware_parameters.find("gripper_baudrate") != info_.hardware_parameters.end()) {
    gripper_baudrate_ = std::stoi(info_.hardware_parameters.at("gripper_baudrate"));
  }

  // Count arm motors (joints with actuator_id) and gripper joints
  size_t arm_joint_count = 0;
  for (const auto & joint : info_.joints) {
    if (joint.parameters.find("actuator_id") != joint.parameters.end()) {
      arm_joint_count++;
    } else if (joint.name.find("gripper") != std::string::npos) {
      // This is a gripper joint
      gripper_.joint_name = joint.name;
      RCLCPP_INFO(getLogger(), "Gripper joint detected: %s", joint.name.c_str());
    }
  }

  motors_.resize(arm_joint_count);
  
  RCLCPP_INFO(getLogger(), "Initializing %zu arm joints on CAN interface '%s'",
    arm_joint_count, ifname_.c_str());

  size_t motor_index = 0;
  for (const auto & joint : info_.joints) {
    if (joint.parameters.find("actuator_id") != joint.parameters.end()) {
      if (!initializeMotor(joint, motor_index)) {
        return hardware_interface::CallbackReturn::ERROR;
      }
      motor_index++;
    }
  }

  RCLCPP_INFO(getLogger(), "✓ Configuration parsed successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// initializeMotor: Parse joint parameters
// ============================================================================
bool ArmHardwareInterface::initializeMotor(
  const hardware_interface::ComponentInfo & joint, size_t index)
{
  MotorData & motor = motors_[index];
  motor.joint_name = joint.name;

  if (joint.parameters.find("actuator_id") == joint.parameters.end()) {
    printArmErrorBox(getLogger(), "CONFIGURATION ERROR",
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
// checkCANInterface
// ============================================================================
bool ArmHardwareInterface::checkCANInterface()
{
  std::vector<std::string> available_can;
  
  std::ifstream proc_net("/proc/net/dev");
  if (proc_net.is_open()) {
    std::string line;
    while (std::getline(proc_net, line)) {
      if (line.find("can") != std::string::npos || line.find("vcan") != std::string::npos) {
        size_t pos = line.find(':');
        if (pos != std::string::npos) {
          std::string iface = line.substr(0, pos);
          iface.erase(0, iface.find_first_not_of(" \t"));
          iface.erase(iface.find_last_not_of(" \t") + 1);
          available_can.push_back(iface);
        }
      }
    }
  }

  int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (sock < 0) {
    printArmErrorBox(getLogger(), "CAN SOCKET ERROR",
      {"Failed to create CAN socket: " + std::string(std::strerror(errno)),
       "",
       "Try running:",
       "  sudo modprobe can",
       "  sudo modprobe can_raw"});
    return false;
  }

  struct ifreq ifr;
  std::strncpy(ifr.ifr_name, ifname_.c_str(), IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';

  if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0) {
    close(sock);
    
    std::vector<std::string> error_lines = {
      "Interface '" + ifname_ + "' not found!",
      ""
    };
    
    if (!available_can.empty()) {
      error_lines.push_back("Available CAN interfaces:");
      for (const auto& iface : available_can) {
        error_lines.push_back("  • " + iface);
      }
      error_lines.push_back("");
    }
    
    error_lines.push_back("To set up CAN interface:");
    error_lines.push_back("  sudo ip link set " + ifname_ + " type can bitrate 1000000");
    error_lines.push_back("  sudo ip link set " + ifname_ + " up");
    
    printArmErrorBox(getLogger(), "CAN INTERFACE NOT FOUND", error_lines);
    return false;
  }

  if (ioctl(sock, SIOCGIFFLAGS, &ifr) >= 0) {
    if (!(ifr.ifr_flags & IFF_UP)) {
      close(sock);
      printArmErrorBox(getLogger(), "CAN INTERFACE DOWN",
        {"Interface '" + ifname_ + "' exists but is DOWN",
         "",
         "Run: sudo ip link set " + ifname_ + " up"});
      return false;
    }
  }

  close(sock);
  RCLCPP_INFO(getLogger(), "✓ CAN interface '%s' is UP", ifname_.c_str());
  return true;
}

// ============================================================================
// Gripper serial communication
// ============================================================================
bool ArmHardwareInterface::openGripperSerial()
{
  if (gripper_port_.empty()) {
    RCLCPP_WARN(getLogger(), "Gripper port not configured");
    return false;
  }

  gripper_serial_fd_ = open(gripper_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (gripper_serial_fd_ < 0) {
    printArmErrorBox(getLogger(), "GRIPPER SERIAL ERROR",
      {"Failed to open serial port: " + gripper_port_,
       "Error: " + std::string(std::strerror(errno)),
       "",
       "Check:",
       "  □ OpenCR is connected",
       "  □ Port permissions (sudo chmod 666 " + gripper_port_ + ")",
       "  □ User is in dialout group"});
    return false;
  }

  // Configure serial port
  struct termios tty;
  memset(&tty, 0, sizeof(tty));
  
  if (tcgetattr(gripper_serial_fd_, &tty) != 0) {
    RCLCPP_ERROR(getLogger(), "tcgetattr failed: %s", std::strerror(errno));
    close(gripper_serial_fd_);
    gripper_serial_fd_ = -1;
    return false;
  }

  // Set baud rate
  speed_t baud;
  switch (gripper_baudrate_) {
    case 9600:   baud = B9600; break;
    case 19200:  baud = B19200; break;
    case 38400:  baud = B38400; break;
    case 57600:  baud = B57600; break;
    case 115200: baud = B115200; break;
    default:     baud = B115200; break;
  }
  cfsetospeed(&tty, baud);
  cfsetispeed(&tty, baud);

  // 8N1 mode
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag |= CREAD | CLOCAL;

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO;
  tty.c_lflag &= ~ECHOE;
  tty.c_lflag &= ~ECHONL;
  tty.c_lflag &= ~ISIG;

  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

  tty.c_oflag &= ~OPOST;
  tty.c_oflag &= ~ONLCR;

  tty.c_cc[VTIME] = 1;
  tty.c_cc[VMIN] = 0;

  if (tcsetattr(gripper_serial_fd_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(getLogger(), "tcsetattr failed: %s", std::strerror(errno));
    close(gripper_serial_fd_);
    gripper_serial_fd_ = -1;
    return false;
  }

  // Wait for OpenCR ready
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  
  RCLCPP_INFO(getLogger(), "✓ Gripper serial opened: %s @ %d baud", 
    gripper_port_.c_str(), gripper_baudrate_);
  return true;
}

void ArmHardwareInterface::closeGripperSerial()
{
  if (gripper_serial_fd_ >= 0) {
    close(gripper_serial_fd_);
    gripper_serial_fd_ = -1;
    RCLCPP_INFO(getLogger(), "Gripper serial closed");
  }
}

bool ArmHardwareInterface::sendGripperCommand(int position)
{
  if (gripper_serial_fd_ < 0) {
    return false;
  }

  // Clamp position to valid range
  position = std::max(0, std::min(4095, position));

  // Send 2 bytes (high, low) - same protocol as gripper_bridge
  uint8_t data[2];
  data[0] = (position >> 8) & 0xFF;
  data[1] = position & 0xFF;

  ssize_t written = ::write(gripper_serial_fd_, data, 2);
  if (written != 2) {
    RCLCPP_WARN(getLogger(), "Gripper write failed: %s", std::strerror(errno));
    return false;
  }

  return true;
}

int ArmHardwareInterface::positionToGripperDxl(double position_meters)
{
  // Map position (meters) to Dynamixel position
  // position_meters: 0.0 (closed) ~ 0.04 (open)
  // Dynamixel: 2048 (closed) ~ 0 (open)
  
  double ratio = (position_meters - gripper_.min_position) / 
                 (gripper_.max_position - gripper_.min_position);
  ratio = std::max(0.0, std::min(1.0, ratio));
  
  int dxl_pos = gripper_.min_dxl_position + 
                static_cast<int>(ratio * (gripper_.max_dxl_position - gripper_.min_dxl_position));
  
  return dxl_pos;
}

// ============================================================================
// on_configure: Connect to motors and gripper
// ============================================================================
hardware_interface::CallbackReturn ArmHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(getLogger(), " ");
  RCLCPP_INFO(getLogger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  RCLCPP_INFO(getLogger(), "  Configuring Arm Hardware Interface");
  RCLCPP_INFO(getLogger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");

  // Step 1: Check CAN interface
  RCLCPP_INFO(getLogger(), "[1/4] Checking CAN interface '%s'...", ifname_.c_str());
  if (!checkCANInterface()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Step 2: Create CAN driver
  RCLCPP_INFO(getLogger(), "[2/4] Creating CAN driver...");
  try {
    driver_ = std::make_unique<myactuator_rmd::CanDriver>(ifname_);
    RCLCPP_INFO(getLogger(), "✓ CAN driver created");
  } catch (const std::exception & e) {
    printArmErrorBox(getLogger(), "CAN DRIVER ERROR",
      {"Failed to create CAN driver: " + std::string(e.what()),
       "",
       "Check that no other process is using the interface"});
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Step 3: Connect to each motor
  RCLCPP_INFO(getLogger(), "[3/4] Connecting to arm motors...");
  
  for (auto & motor : motors_) {
    RCLCPP_INFO(getLogger(), "  → Connecting to '%s' (ID=%d)...", 
      motor.joint_name.c_str(), motor.actuator_id);
    
    try {
      motor.actuator = std::make_unique<myactuator_rmd::ActuatorInterface>(
        *driver_, motor.actuator_id);
      
      std::string model = motor.actuator->getMotorModel();
      RCLCPP_INFO(getLogger(), "    ✓ Connected: %s", model.c_str());
      
      motor.actuator->setAcceleration(500, myactuator_rmd::AccelerationType::POSITION_PLANNING_ACCELERATION);
      motor.actuator->setAcceleration(500, myactuator_rmd::AccelerationType::POSITION_PLANNING_DECELERATION);
      RCLCPP_INFO(getLogger(), "    ✓ Smooth motion enabled (acceleration=500 deg/s²)");
      
    } catch (const std::exception & e) {
      std::vector<std::string> error_lines = {
        "Motor: " + motor.joint_name,
        "ID: " + std::to_string(motor.actuator_id),
        "Error: " + std::string(e.what()),
        "",
        "Checklist:",
        "  □ Motor power is ON",
        "  □ CAN wiring correct (CAN_H, CAN_L, GND)",
        "  □ Motor ID matches configuration",
        "  □ CAN bitrate matches motor (usually 1Mbps)",
        "  □ 120Ω termination resistor installed"
      };
      printArmErrorBox(getLogger(), "MOTOR CONNECTION FAILED", error_lines);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // Step 4: Connect to gripper (if enabled)
  if (gripper_enabled_) {
    RCLCPP_INFO(getLogger(), "[4/4] Connecting to gripper...");
    if (!openGripperSerial()) {
      RCLCPP_WARN(getLogger(), "  ⚠ Gripper connection failed - continuing without gripper");
      gripper_enabled_ = false;
    } else {
      RCLCPP_INFO(getLogger(), "  ✓ Gripper connected");
    }
  } else {
    RCLCPP_INFO(getLogger(), "[4/4] Gripper not configured - skipping");
  }

  // Start async thread
  if (!startAsyncThread()) {
    RCLCPP_ERROR(getLogger(), "Failed to start async thread");
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::vector<std::string> summary = {
    "Arm motors: " + std::to_string(motors_.size()) + " connected",
    "CAN Interface: " + ifname_,
    "Gripper: " + std::string(gripper_enabled_ ? "enabled" : "disabled")
  };
  printArmInfoBox(getLogger(), "CONFIGURATION SUCCESSFUL", summary);

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// on_cleanup
// ============================================================================
hardware_interface::CallbackReturn ArmHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(getLogger(), "Cleaning up...");

  stopAsyncThread();
  stopHoldingThread();

  for (auto & motor : motors_) {
    if (motor.actuator) {
      try {
        motor.actuator->shutdownMotor();
      } catch (...) {}
      motor.actuator.reset();
    }
  }
  driver_.reset();
  
  closeGripperSerial();

  RCLCPP_INFO(getLogger(), "✓ Cleanup complete");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// on_activate
// ============================================================================
hardware_interface::CallbackReturn ArmHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(getLogger(), " ");
  RCLCPP_INFO(getLogger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  RCLCPP_INFO(getLogger(), "  Activating Arm + Gripper");
  RCLCPP_INFO(getLogger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");

  // 1. Read current positions
  RCLCPP_INFO(getLogger(), "[1/4] Reading current arm positions...");
  for (auto & motor : motors_) {
    if (motor.actuator) {
      try {
        auto feedback = motor.actuator->getMotorStatus2();
        double current_pos_rad = static_cast<double>(feedback.shaft_angle) * M_PI / 180.0;
        motor.position_state = current_pos_rad;
        RCLCPP_INFO(getLogger(), "  → '%s': current position = %.3f rad (%.1f deg)", 
          motor.joint_name.c_str(), current_pos_rad, feedback.shaft_angle);
      } catch (const std::exception & e) {
        RCLCPP_WARN(getLogger(), "  → '%s': read failed (%s)", 
          motor.joint_name.c_str(), e.what());
      }
    }
  }

  // 2. Move to home if enabled
  if (auto_home_) {
    RCLCPP_INFO(getLogger(), "[2/4] Moving arm joints to home position (0 rad)...");
    
    for (auto & motor : motors_) {
      if (motor.actuator) {
        try {
          motor.position_command = 0.0;
          float target_deg = 0.0f;
          float max_speed = 50.0f;
          motor.actuator->sendPositionAbsoluteSetpoint(target_deg, max_speed);
          RCLCPP_INFO(getLogger(), "  → '%s': moving to home (0 rad)", motor.joint_name.c_str());
        } catch (const std::exception & e) {
          RCLCPP_WARN(getLogger(), "  → '%s': home command failed (%s)", 
            motor.joint_name.c_str(), e.what());
        }
      }
    }

    RCLCPP_INFO(getLogger(), "  Waiting for motors to reach home position...");
    
    bool all_at_home = false;
    int max_attempts = 100;
    int attempt = 0;
    
    while (!all_at_home && attempt < max_attempts) {
      all_at_home = true;
      
      for (auto & motor : motors_) {
        if (motor.actuator) {
          try {
            auto feedback = motor.actuator->getMotorStatus2();
            double current_pos_rad = static_cast<double>(feedback.shaft_angle) * M_PI / 180.0;
            motor.position_state = current_pos_rad;
            
            if (std::abs(current_pos_rad) > 0.05) {
              all_at_home = false;
            }
          } catch (...) {}
        }
      }
      
      if (!all_at_home) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        attempt++;
        if (attempt % 10 == 0) {
          RCLCPP_INFO(getLogger(), "  ... waiting (%.1f sec)", attempt * 0.1);
        }
      }
    }

    if (all_at_home) {
      RCLCPP_INFO(getLogger(), "  ✓ All arm motors at home position");
    } else {
      RCLCPP_WARN(getLogger(), "  ⚠ Timeout waiting for home position");
    }

    // 2.5. Move to ready position (singularity avoidance)
    RCLCPP_INFO(getLogger(), "  Waiting 1 second before moving to ready position...");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    RCLCPP_INFO(getLogger(), "[2.5/4] Moving arm joints to ready position...");
    
    // Ready pose from SRDF (singularity-free pose)
    // link2_to_link1: 0.0, link3_to_link2: -0.69813, link4_to_link3: -2.35619, gripper_to_link4: 0.05236
    const std::vector<double> ready_positions_rad = {0.0, -0.69813, -2.35619, 0.05236};
    
    for (size_t i = 0; i < motors_.size() && i < ready_positions_rad.size(); ++i) {
      auto & motor = motors_[i];
      if (motor.actuator) {
        try {
          motor.position_command = ready_positions_rad[i];
          float target_deg = static_cast<float>(ready_positions_rad[i] * 180.0 / M_PI);
          float max_speed = 50.0f;
          motor.actuator->sendPositionAbsoluteSetpoint(target_deg, max_speed);
          RCLCPP_INFO(getLogger(), "  → '%s': moving to ready (%.3f rad / %.1f deg)", 
            motor.joint_name.c_str(), ready_positions_rad[i], target_deg);
        } catch (const std::exception & e) {
          RCLCPP_WARN(getLogger(), "  → '%s': ready command failed (%s)", 
            motor.joint_name.c_str(), e.what());
        }
      }
    }

    RCLCPP_INFO(getLogger(), "  Waiting for motors to reach ready position...");
    
    bool all_at_ready = false;
    max_attempts = 100;
    attempt = 0;
    
    while (!all_at_ready && attempt < max_attempts) {
      all_at_ready = true;
      
      for (size_t i = 0; i < motors_.size() && i < ready_positions_rad.size(); ++i) {
        auto & motor = motors_[i];
        if (motor.actuator) {
          try {
            auto feedback = motor.actuator->getMotorStatus2();
            double current_pos_rad = static_cast<double>(feedback.shaft_angle) * M_PI / 180.0;
            motor.position_state = current_pos_rad;
            
            if (std::abs(current_pos_rad - ready_positions_rad[i]) > 0.05) {
              all_at_ready = false;
            }
          } catch (...) {}
        }
      }
      
      if (!all_at_ready) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        attempt++;
        if (attempt % 10 == 0) {
          RCLCPP_INFO(getLogger(), "  ... waiting (%.1f sec)", attempt * 0.1);
        }
      }
    }

    if (all_at_ready) {
      RCLCPP_INFO(getLogger(), "  ✓ All arm motors at ready position (singularity-free)");
    } else {
      RCLCPP_WARN(getLogger(), "  ⚠ Timeout waiting for ready position");
    }
  } else {
    RCLCPP_INFO(getLogger(), "[2/4] Auto-home disabled - keeping current positions");
  }

  // 3. Initialize gripper
  if (gripper_enabled_) {
    RCLCPP_INFO(getLogger(), "[3/4] Initializing gripper...");
    // Send initial position (open)
    int initial_pos = positionToGripperDxl(gripper_.position_state);
    sendGripperCommand(initial_pos);
    RCLCPP_INFO(getLogger(), "  ✓ Gripper initialized");
  } else {
    RCLCPP_INFO(getLogger(), "[3/4] Gripper not enabled - skipping");
  }

  // 4. Initialize command values
  RCLCPP_INFO(getLogger(), "[4/4] Initializing command values...");
  for (auto & motor : motors_) {
    motor.position_command = motor.position_state;
    motor.velocity_command = 0.0;
    motor.effort_command = 0.0;
    RCLCPP_INFO(getLogger(), "  → '%s': ready at %.3f rad", 
      motor.joint_name.c_str(), motor.position_state);
  }
  
  gripper_.position_command = gripper_.position_state;

  RCLCPP_INFO(getLogger(), " ");
  RCLCPP_INFO(getLogger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  RCLCPP_INFO(getLogger(), "  ✓ Activation complete");
  RCLCPP_INFO(getLogger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  RCLCPP_INFO(getLogger(), " ");

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// on_deactivate
// ============================================================================
hardware_interface::CallbackReturn ArmHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(getLogger(), " ");
  RCLCPP_INFO(getLogger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  RCLCPP_INFO(getLogger(), "  Deactivating Arm + Gripper");
  RCLCPP_INFO(getLogger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");

  if (auto_home_) {
    RCLCPP_INFO(getLogger(), "[1/2] Moving arm joints to home position (0.0 rad)...");
    
    for (auto & motor : motors_) {
      if (motor.actuator) {
        try {
          motor.position_command = 0.0;
          float target_deg = 0.0f;
          float max_speed = 100.0f;
          motor.actuator->sendPositionAbsoluteSetpoint(target_deg, max_speed);
          RCLCPP_INFO(getLogger(), "  → '%s': moving to home (current: %.2f rad)", 
            motor.joint_name.c_str(), motor.position_state);
        } catch (const std::exception & e) {
          RCLCPP_WARN(getLogger(), "  → '%s': home command failed (%s)", 
            motor.joint_name.c_str(), e.what());
        }
      }
    }

    RCLCPP_INFO(getLogger(), "  Waiting for motors to reach home position...");
    
    bool all_at_home = false;
    int max_attempts = 100;
    int attempt = 0;
    
    while (!all_at_home && attempt < max_attempts) {
      all_at_home = true;
      
      for (auto & motor : motors_) {
        if (motor.actuator) {
          try {
            auto feedback = motor.actuator->getMotorStatus2();
            double current_pos_rad = static_cast<double>(feedback.shaft_angle) * M_PI / 180.0;
            motor.position_state = current_pos_rad;
            
            if (std::abs(current_pos_rad) > 0.05) {
              all_at_home = false;
            }
          } catch (...) {}
        }
      }
      
      if (!all_at_home) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        attempt++;
        if (attempt % 10 == 0) {
          RCLCPP_INFO(getLogger(), "  ... waiting (%.1f sec)", attempt * 0.1);
        }
      }
    }

    if (all_at_home) {
      RCLCPP_INFO(getLogger(), "  ✓ All motors at home position");
    } else {
      RCLCPP_WARN(getLogger(), "  ⚠ Timeout waiting for home position");
    }
  } else {
    RCLCPP_INFO(getLogger(), "[1/2] Auto-home disabled - skipping home movement");
  }

  RCLCPP_INFO(getLogger(), "[2/2] Starting position holding thread...");
  
  stopHoldingThread();
  startHoldingThread();
  
  RCLCPP_INFO(getLogger(), "  ✓ Position holding thread started");

  RCLCPP_INFO(getLogger(), " ");
  RCLCPP_INFO(getLogger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  RCLCPP_INFO(getLogger(), "  ✓ Deactivation complete - motors holding at home");
  RCLCPP_INFO(getLogger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  RCLCPP_INFO(getLogger(), " ");
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// on_error
// ============================================================================
hardware_interface::CallbackReturn ArmHardwareInterface::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  printArmErrorBox(getLogger(), "HARDWARE ERROR",
    {"Emergency stop triggered",
     "All motors will be stopped"});

  stopAsyncThread();

  for (auto & motor : motors_) {
    if (motor.actuator) {
      try {
        motor.actuator->stopMotor();
        motor.actuator->reset();
      } catch (...) {}
    }
  }

  RCLCPP_INFO(getLogger(), "✓ Hardware in safe state");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// export_state_interfaces
// ============================================================================
std::vector<hardware_interface::StateInterface>
ArmHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Arm motor state interfaces
  for (auto & motor : motors_) {
    state_interfaces.emplace_back(
      motor.joint_name, hardware_interface::HW_IF_POSITION, &motor.position_state);
    state_interfaces.emplace_back(
      motor.joint_name, hardware_interface::HW_IF_VELOCITY, &motor.velocity_state);
    state_interfaces.emplace_back(
      motor.joint_name, hardware_interface::HW_IF_EFFORT, &motor.effort_state);
  }

  // Gripper state interface
  if (!gripper_.joint_name.empty()) {
    state_interfaces.emplace_back(
      gripper_.joint_name, hardware_interface::HW_IF_POSITION, &gripper_.position_state);
    state_interfaces.emplace_back(
      gripper_.joint_name, hardware_interface::HW_IF_VELOCITY, &gripper_.velocity_state);
  }

  RCLCPP_DEBUG(getLogger(), "Exported %zu state interfaces", state_interfaces.size());
  return state_interfaces;
}

// ============================================================================
// export_command_interfaces
// ============================================================================
std::vector<hardware_interface::CommandInterface>
ArmHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Arm motor command interfaces
  for (auto & motor : motors_) {
    command_interfaces.emplace_back(
      motor.joint_name, hardware_interface::HW_IF_POSITION, &motor.position_command);
    command_interfaces.emplace_back(
      motor.joint_name, hardware_interface::HW_IF_VELOCITY, &motor.velocity_command);
    command_interfaces.emplace_back(
      motor.joint_name, hardware_interface::HW_IF_EFFORT, &motor.effort_command);
  }

  // Gripper command interface
  if (!gripper_.joint_name.empty()) {
    command_interfaces.emplace_back(
      gripper_.joint_name, hardware_interface::HW_IF_POSITION, &gripper_.position_command);
  }

  RCLCPP_DEBUG(getLogger(), "Exported %zu command interfaces", command_interfaces.size());
  return command_interfaces;
}

// ============================================================================
// read: Get motor and gripper feedback
// ============================================================================
hardware_interface::return_type ArmHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::lock_guard<std::mutex> lock(mutex_);

  // Read arm motors
  for (auto & motor : motors_) {
    if (!motor.actuator) continue;

    try {
      auto feedback = motor.actuator->getMotorStatus2();
      
      motor.position_state = feedback.shaft_angle * M_PI / 180.0;
      motor.velocity_state = feedback.shaft_speed * M_PI / 180.0;
      motor.effort_state = feedback.current * motor.torque_constant;

    } catch (const std::exception & e) {
      RCLCPP_WARN(getLogger(),
        "Read failed for '%s': %s", motor.joint_name.c_str(), e.what());
    }
  }

  // Gripper: assume command = state (no feedback from OpenCR)
  // In a real implementation, you could read feedback from OpenCR
  gripper_.position_state = gripper_.position_command;

  return hardware_interface::return_type::OK;
}

// ============================================================================
// write: Send motor and gripper commands
// ============================================================================
hardware_interface::return_type ArmHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::lock_guard<std::mutex> lock(mutex_);

  // Write to arm motors
  for (auto & motor : motors_) {
    if (!motor.actuator) continue;

    try {
      double position_deg = motor.position_command * 180.0 / M_PI;
      double velocity_dps = std::abs(motor.velocity_command) * 180.0 / M_PI;
      
      if (velocity_dps < 5.0) {
        velocity_dps = 360.0;
      } else if (velocity_dps > 720.0) {
        velocity_dps = 720.0;
      }

      motor.actuator->sendPositionAbsoluteSetpoint(position_deg, velocity_dps);
      motor.last_position_command_deg = position_deg;

    } catch (const std::exception & e) {
      RCLCPP_WARN(getLogger(),
        "Write failed for '%s': %s", motor.joint_name.c_str(), e.what());
    }
  }

  // Write to gripper (if enabled and command changed)
  if (gripper_enabled_ && gripper_serial_fd_ >= 0) {
    int dxl_pos = positionToGripperDxl(gripper_.position_command);
    
    // Only send if position changed significantly
    if (std::abs(dxl_pos - gripper_.last_dxl_command) > 10) {
      RCLCPP_INFO(getLogger(), "Gripper: cmd=%.4f m → DXL %d (last=%d)", 
        gripper_.position_command, dxl_pos, gripper_.last_dxl_command);
      if (sendGripperCommand(dxl_pos)) {
        gripper_.last_dxl_command = dxl_pos;
      }
    }
  }

  return hardware_interface::return_type::OK;
}

// ============================================================================
// Async thread management
// ============================================================================
bool ArmHardwareInterface::startAsyncThread()
{
  stop_async_thread_.store(false);
  async_thread_ = std::thread(&ArmHardwareInterface::asyncThreadFunc, this);
  return true;
}

void ArmHardwareInterface::stopAsyncThread()
{
  stop_async_thread_.store(true);
  if (async_thread_.joinable()) {
    async_thread_.join();
  }
}

void ArmHardwareInterface::asyncThreadFunc()
{
  while (!stop_async_thread_.load()) {
    std::this_thread::sleep_for(cycle_time_);
  }
}

// ============================================================================
// Position holding thread management
// ============================================================================
bool ArmHardwareInterface::startHoldingThread()
{
  stop_holding_thread_.store(false);
  holding_thread_ = std::thread(&ArmHardwareInterface::holdingThreadFunc, this);
  return true;
}

void ArmHardwareInterface::stopHoldingThread()
{
  stop_holding_thread_.store(true);
  if (holding_thread_.joinable()) {
    holding_thread_.join();
  }
}

void ArmHardwareInterface::holdingThreadFunc()
{
  RCLCPP_INFO(getLogger(), "Position holding thread started");
  
  while (!stop_holding_thread_.load()) {
    for (auto & motor : motors_) {
      if (motor.actuator) {
        try {
          motor.actuator->sendPositionAbsoluteSetpoint(0.0f, 10.0f);
        } catch (const std::exception & e) {
          RCLCPP_WARN(getLogger(),
            "Holding command failed for '%s': %s", motor.joint_name.c_str(), e.what());
        }
      }
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  
  RCLCPP_INFO(getLogger(), "Position holding thread stopped");
}

}  // namespace myactuator_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  myactuator_hardware::ArmHardwareInterface,
  hardware_interface::SystemInterface)
