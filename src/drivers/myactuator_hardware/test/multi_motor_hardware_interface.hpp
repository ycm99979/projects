#ifndef MYACTUATOR_HARDWARE__MULTI_MOTOR_HARDWARE_INTERFACE_HPP_
#define MYACTUATOR_HARDWARE__MULTI_MOTOR_HARDWARE_INTERFACE_HPP_

#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <unordered_map>

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
 * @brief Multi-motor hardware interface for MyActuator RMD series
 * 
 * This class implements ros2_control SystemInterface to support multiple motors
 * on a single CAN bus. It is designed to work with MoveIt2's JointTrajectoryController.
 */
class MultiMotorHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MultiMotorHardwareInterface)

  /// @brief Constructor
  MultiMotorHardwareInterface() = default;

  /// @brief Destructor - ensures proper cleanup
  ~MultiMotorHardwareInterface() override;

  /// @brief Initialize the hardware interface from URDF
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  /// @brief Configure the hardware (establish CAN connection)
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  /// @brief Cleanup hardware resources
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  /// @brief Activate the hardware (enable motors)
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  /// @brief Deactivate the hardware (disable motors)
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  /// @brief Handle errors
  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  /// @brief Export state interfaces for all joints
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  /// @brief Export command interfaces for all joints
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  /// @brief Read current state from all motors
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /// @brief Write commands to all motors
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  /// @brief Structure to hold per-motor data
  struct MotorData
  {
    std::string joint_name;
    int actuator_id{0};
    double torque_constant{0.0};
    
    // State
    double position_state{0.0};
    double velocity_state{0.0};
    double effort_state{0.0};
    
    // Command
    double position_command{0.0};
    double velocity_command{0.0};
    double effort_command{0.0};
    
    // Last sent command (for filtering)
    double last_position_command_deg{0.0};
    
    // Actuator interface
    std::unique_ptr<myactuator_rmd::ActuatorInterface> actuator;
  };

  /// @brief Get logger
  rclcpp::Logger getLogger() const;

  /// @brief Initialize motor from joint configuration
  bool initializeMotor(
    const hardware_interface::ComponentInfo & joint,
    size_t index);

  /// @brief Check if CAN interface is available and UP
  bool checkCANInterface();

  /// @brief Start async communication thread
  bool startAsyncThread();

  /// @brief Stop async communication thread
  void stopAsyncThread();

  /// @brief Async thread function
  void asyncThreadFunc();

  /// @brief Start position holding thread (for deactivated state)
  bool startHoldingThread();

  /// @brief Stop position holding thread
  void stopHoldingThread();

  /// @brief Position holding thread function
  void holdingThreadFunc();

  // CAN driver (shared across all motors on same bus)
  std::string ifname_;
  std::unique_ptr<myactuator_rmd::CanDriver> driver_;

  // Motor data for each joint
  std::vector<MotorData> motors_;

  // Thread for async communication
  std::thread async_thread_;
  std::atomic<bool> stop_async_thread_{false};
  std::mutex mutex_;

  // Thread for position holding (when deactivated)
  std::thread holding_thread_;
  std::atomic<bool> stop_holding_thread_{false};

  // Configuration
  std::chrono::milliseconds cycle_time_{2};
  std::chrono::milliseconds timeout_{0};
  bool auto_home_{true};  // Move to home position on activate/deactivate

  // Control mode tracking
  enum class ControlMode { POSITION, VELOCITY, EFFORT };
  ControlMode active_control_mode_{ControlMode::POSITION};
};

}  // namespace myactuator_hardware

#endif  // MYACTUATOR_HARDWARE__MULTI_MOTOR_HARDWARE_INTERFACE_HPP_
