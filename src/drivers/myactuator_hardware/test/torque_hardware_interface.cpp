#include "myactuator_hardware/torque_hardware_interface.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <fstream>
#include <cerrno>
#include <cstring>

#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>

namespace myactuator_hardware
{

TorqueHardwareInterface::~TorqueHardwareInterface()
{
  on_cleanup(rclcpp_lifecycle::State());
}

rclcpp::Logger TorqueHardwareInterface::getLogger() const
{
  return rclcpp::get_logger("TorqueHW");
}

// ============================================================================
// on_init
// ============================================================================
hardware_interface::CallbackReturn TorqueHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.hardware_parameters.find("ifname") == info_.hardware_parameters.end()) {
    RCLCPP_ERROR(getLogger(), "Missing required parameter: 'ifname'");
    return hardware_interface::CallbackReturn::ERROR;
  }
  ifname_ = info_.hardware_parameters.at("ifname");

  motors_.resize(info_.joints.size());
  
  RCLCPP_INFO(getLogger(), "Initializing %zu joints for TORQUE control on '%s'",
    info_.joints.size(), ifname_.c_str());

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    if (!initializeMotor(info_.joints[i], i)) {
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(getLogger(), "✓ Torque control configuration ready");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// initializeMotor
// ============================================================================
bool TorqueHardwareInterface::initializeMotor(
  const hardware_interface::ComponentInfo & joint, size_t index)
{
  MotorData & motor = motors_[index];
  motor.joint_name = joint.name;

  if (joint.parameters.find("actuator_id") == joint.parameters.end()) {
    RCLCPP_ERROR(getLogger(), "Joint '%s' missing 'actuator_id'", joint.name.c_str());
    return false;
  }
  motor.actuator_id = std::stoi(joint.parameters.at("actuator_id"));

  if (joint.parameters.find("torque_constant") != joint.parameters.end()) {
    motor.torque_constant = std::stod(joint.parameters.at("torque_constant"));
  }
  
  if (joint.parameters.find("max_current") != joint.parameters.end()) {
    motor.max_current = std::stod(joint.parameters.at("max_current"));
  }
  
  if (joint.parameters.find("max_torque") != joint.parameters.end()) {
    motor.max_torque = std::stod(joint.parameters.at("max_torque"));
  }

  RCLCPP_INFO(getLogger(), "  → Joint[%zu]: '%s' ID=%d, Kt=%.3f Nm/A, Imax=%.1f A, Tmax=%.1f Nm",
    index, motor.joint_name.c_str(), motor.actuator_id, 
    motor.torque_constant, motor.max_current, motor.max_torque);

  return true;
}

// ============================================================================
// checkCANInterface
// ============================================================================
bool TorqueHardwareInterface::checkCANInterface()
{
  int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (sock < 0) {
    RCLCPP_ERROR(getLogger(), "Failed to create CAN socket");
    return false;
  }

  struct ifreq ifr;
  std::strncpy(ifr.ifr_name, ifname_.c_str(), IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';

  if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0) {
    close(sock);
    RCLCPP_ERROR(getLogger(), "CAN interface '%s' not found", ifname_.c_str());
    return false;
  }

  if (ioctl(sock, SIOCGIFFLAGS, &ifr) >= 0) {
    if (!(ifr.ifr_flags & IFF_UP)) {
      close(sock);
      RCLCPP_ERROR(getLogger(), "CAN interface '%s' is DOWN", ifname_.c_str());
      return false;
    }
  }

  close(sock);
  RCLCPP_INFO(getLogger(), "✓ CAN interface '%s' is UP", ifname_.c_str());
  return true;
}

// ============================================================================
// on_configure
// ============================================================================
hardware_interface::CallbackReturn TorqueHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(getLogger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  RCLCPP_INFO(getLogger(), "  Configuring Torque Hardware Interface");
  RCLCPP_INFO(getLogger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");

  if (!checkCANInterface()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  try {
    driver_ = std::make_unique<myactuator_rmd::CanDriver>(ifname_);
    RCLCPP_INFO(getLogger(), "✓ CAN driver created");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(getLogger(), "CAN driver error: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (auto & motor : motors_) {
    RCLCPP_INFO(getLogger(), "  → Connecting to '%s' (ID=%d)...", 
      motor.joint_name.c_str(), motor.actuator_id);
    
    try {
      motor.actuator = std::make_unique<myactuator_rmd::ActuatorInterface>(
        *driver_, motor.actuator_id);
      
      std::string model = motor.actuator->getMotorModel();
      RCLCPP_INFO(getLogger(), "    ✓ Connected: %s", model.c_str());
      
    } catch (const std::exception & e) {
      RCLCPP_ERROR(getLogger(), "Motor connection failed: %s", e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(getLogger(), "┌─────────────────────────────────────────┐");
  RCLCPP_INFO(getLogger(), "│  TORQUE CONTROL MODE READY              │");
  RCLCPP_INFO(getLogger(), "│  Direct current control enabled         │");
  RCLCPP_INFO(getLogger(), "└─────────────────────────────────────────┘");

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// on_cleanup
// ============================================================================
hardware_interface::CallbackReturn TorqueHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(getLogger(), "Cleaning up...");

  for (auto & motor : motors_) {
    if (motor.actuator) {
      try {
        motor.actuator->sendTorqueSetpoint(0.0, 0.0);  // Zero torque
        motor.actuator->shutdownMotor();
      } catch (...) {}
      motor.actuator.reset();
    }
  }
  driver_.reset();

  RCLCPP_INFO(getLogger(), "✓ Cleanup complete");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// on_activate
// ============================================================================
hardware_interface::CallbackReturn TorqueHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(getLogger(), "Activating torque control...");

  for (auto & motor : motors_) {
    motor.effort_command = 0.0;
    RCLCPP_INFO(getLogger(), "  → '%s': torque control active", motor.joint_name.c_str());
  }

  RCLCPP_INFO(getLogger(), "✓ Torque control activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// on_deactivate
// ============================================================================
hardware_interface::CallbackReturn TorqueHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(getLogger(), "Deactivating...");

  for (auto & motor : motors_) {
    if (motor.actuator) {
      try {
        motor.actuator->sendTorqueSetpoint(0.0, 0.0);
        motor.actuator->stopMotor();
      } catch (...) {}
    }
  }

  RCLCPP_INFO(getLogger(), "✓ Deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// on_error
// ============================================================================
hardware_interface::CallbackReturn TorqueHardwareInterface::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_ERROR(getLogger(), "ERROR - Stopping all motors!");

  for (auto & motor : motors_) {
    if (motor.actuator) {
      try {
        motor.actuator->sendTorqueSetpoint(0.0, 0.0);
        motor.actuator->stopMotor();
      } catch (...) {}
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// export_state_interfaces
// ============================================================================
std::vector<hardware_interface::StateInterface>
TorqueHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;

  for (auto & motor : motors_) {
    interfaces.emplace_back(
      motor.joint_name, hardware_interface::HW_IF_POSITION, &motor.position_state);
    interfaces.emplace_back(
      motor.joint_name, hardware_interface::HW_IF_VELOCITY, &motor.velocity_state);
    interfaces.emplace_back(
      motor.joint_name, hardware_interface::HW_IF_EFFORT, &motor.effort_state);
  }

  return interfaces;
}

// ============================================================================
// export_command_interfaces
// ============================================================================
std::vector<hardware_interface::CommandInterface>
TorqueHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;

  for (auto & motor : motors_) {
    interfaces.emplace_back(
      motor.joint_name, hardware_interface::HW_IF_POSITION, &motor.position_command);
    interfaces.emplace_back(
      motor.joint_name, hardware_interface::HW_IF_VELOCITY, &motor.velocity_command);
    interfaces.emplace_back(
      motor.joint_name, hardware_interface::HW_IF_EFFORT, &motor.effort_command);
  }

  return interfaces;
}

// ============================================================================
// read: Get motor feedback
// ============================================================================
hardware_interface::return_type TorqueHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::lock_guard<std::mutex> lock(mutex_);

  for (auto & motor : motors_) {
    if (!motor.actuator) continue;

    try {
      auto feedback = motor.actuator->getMotorStatus2();
      
      motor.position_state = feedback.shaft_angle * M_PI / 180.0;
      motor.velocity_state = feedback.shaft_speed * M_PI / 180.0;
      motor.effort_state = feedback.current * motor.torque_constant;

    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(getLogger(), *rclcpp::Clock::make_shared(), 5000,
        "Read failed for '%s': %s", motor.joint_name.c_str(), e.what());
    }
  }

  return hardware_interface::return_type::OK;
}

// ============================================================================
// write: Send TORQUE commands directly
// ============================================================================
hardware_interface::return_type TorqueHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::lock_guard<std::mutex> lock(mutex_);

  for (auto & motor : motors_) {
    if (!motor.actuator) continue;

    try {
      // Send torque command directly (Nm)
      // sendTorqueSetpoint takes (torque_Nm, torque_constant)
      double torque_Nm = motor.effort_command;
      
      // Clamp to max_torque limit (25Nm by default)
      torque_Nm = std::clamp(torque_Nm, -motor.max_torque, motor.max_torque);
      
      // Send torque setpoint
      motor.actuator->sendTorqueSetpoint(
        static_cast<float>(torque_Nm), 
        static_cast<float>(motor.torque_constant));

    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(getLogger(), *rclcpp::Clock::make_shared(), 5000,
        "Write failed for '%s': %s", motor.joint_name.c_str(), e.what());
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace myactuator_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  myactuator_hardware::TorqueHardwareInterface,
  hardware_interface::SystemInterface)
