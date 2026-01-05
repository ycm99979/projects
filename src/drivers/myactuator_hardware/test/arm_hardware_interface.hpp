#ifndef MYACTUATOR_HARDWARE__ARM_HARDWARE_INTERFACE_HPP_
#define MYACTUATOR_HARDWARE__ARM_HARDWARE_INTERFACE_HPP_

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
 * @brief Arm hardware interface with integrated gripper control
 * 
 * This class extends the multi-motor interface to include gripper control
 * via serial communication to OpenCR (Dynamixel gripper).
 * 
 * - Arm joints: MyActuator RMD motors via CAN
 * - Gripper joint: Dynamixel via OpenCR serial bridge
 */
class ArmHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ArmHardwareInterface)

  ArmHardwareInterface() = default;
  ~ArmHardwareInterface() override;

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
  /// @brief Structure for arm motor data (MyActuator RMD)
  struct MotorData
  {
    std::string joint_name;
    int actuator_id{0};
    double torque_constant{0.0};
    
    double position_state{0.0};
    double velocity_state{0.0};
    double effort_state{0.0};
    
    double position_command{0.0};
    double velocity_command{0.0};
    double effort_command{0.0};
    
    double last_position_command_deg{0.0};
    
    std::unique_ptr<myactuator_rmd::ActuatorInterface> actuator;
  };

  /// @brief Structure for gripper data (Dynamixel via OpenCR)
  struct GripperData
  {
    std::string joint_name;
    
    double position_state{0.0};
    double velocity_state{0.0};
    
    double position_command{0.0};
    
    // Gripper range mapping (matches URDF limits: -0.01 ~ 0.019)
    double min_position{-0.01};    // meters (closed)
    double max_position{0.019};    // meters (open)
    int min_dxl_position{2048};    // Dynamixel position (closed)
    int max_dxl_position{0};       // Dynamixel position (open)
    
    int last_dxl_command{-1};
  };

  rclcpp::Logger getLogger() const;

  bool initializeMotor(const hardware_interface::ComponentInfo & joint, size_t index);
  bool checkCANInterface();

  bool startAsyncThread();
  void stopAsyncThread();
  void asyncThreadFunc();

  bool startHoldingThread();
  void stopHoldingThread();
  void holdingThreadFunc();

  // ========== Gripper serial communication ==========
  bool openGripperSerial();
  void closeGripperSerial();
  bool sendGripperCommand(int position);
  int positionToGripperDxl(double position_meters);

  // CAN driver for arm motors
  std::string ifname_;
  std::unique_ptr<myactuator_rmd::CanDriver> driver_;

  // Arm motor data
  std::vector<MotorData> motors_;

  // Gripper data
  GripperData gripper_;
  bool gripper_enabled_{false};
  std::string gripper_port_;
  int gripper_baudrate_{115200};
  int gripper_serial_fd_{-1};

  // Threads
  std::thread async_thread_;
  std::atomic<bool> stop_async_thread_{false};
  std::mutex mutex_;

  std::thread holding_thread_;
  std::atomic<bool> stop_holding_thread_{false};

  // Configuration
  std::chrono::milliseconds cycle_time_{2};
  std::chrono::milliseconds timeout_{0};
  bool auto_home_{true};

  enum class ControlMode { POSITION, VELOCITY, EFFORT };
  ControlMode active_control_mode_{ControlMode::POSITION};
};

}  // namespace myactuator_hardware

#endif  // MYACTUATOR_HARDWARE__ARM_HARDWARE_INTERFACE_HPP_
