#ifndef MYACTUATOR_HARDWARE__TORQUE_HARDWARE_INTERFACE_HPP_
#define MYACTUATOR_HARDWARE__TORQUE_HARDWARE_INTERFACE_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <myactuator_rmd/actuator_interface.hpp>
#include <myactuator_rmd/driver/can_driver.hpp>

namespace myactuator_hardware
{

/**
 * @brief Torque control hardware interface for MyActuator RMD series
 * 
 * This interface supports direct torque (current) control for immediate response.
 * Unlike position control, torque commands are sent directly to the motor.
 */
class TorqueHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(TorqueHardwareInterface)

  TorqueHardwareInterface() = default;
  ~TorqueHardwareInterface() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct MotorData
  {
    std::string joint_name;
    int actuator_id{0};
    double torque_constant{0.32};  // Nm/A
    double max_current{10.0};      // A
    double max_torque{25.0};       // Nm - 최대 토크 제한
    
    // State
    double position_state{0.0};
    double velocity_state{0.0};
    double effort_state{0.0};
    
    // Command
    double position_command{0.0};
    double velocity_command{0.0};
    double effort_command{0.0};
    
    // Actuator
    std::unique_ptr<myactuator_rmd::ActuatorInterface> actuator;
  };

  rclcpp::Logger getLogger() const;
  bool initializeMotor(const hardware_interface::ComponentInfo & joint, size_t index);
  bool checkCANInterface();

  std::string ifname_;
  std::unique_ptr<myactuator_rmd::CanDriver> driver_;
  std::vector<MotorData> motors_;
  std::mutex mutex_;
};

}  // namespace myactuator_hardware

#endif  // MYACTUATOR_HARDWARE__TORQUE_HARDWARE_INTERFACE_HPP_
