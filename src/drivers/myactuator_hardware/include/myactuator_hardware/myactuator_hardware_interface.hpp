#ifndef MYACTUATOR_HARDWARE__MYACTUATOR_HARDWARE_INTERFACE_HPP_
#define MYACTUATOR_HARDWARE__MYACTUATOR_HARDWARE_INTERFACE_HPP_

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

namespace myactuator_hardware
{

// ============================================================================
// USBCAN Library Types (from libusbcan.so)
// ============================================================================

// Device types
constexpr uint32_t USBCAN_I = 3;
constexpr uint32_t USBCAN_II = 4;

// CAN board info structure
struct ZCAN_CAN_BOARD_INFO {
  uint16_t hw_Version;
  uint16_t fw_Version;
  uint16_t dr_Version;
  uint16_t in_Version;
  uint16_t irq_Num;
  uint8_t can_Num;
  uint8_t str_Serial_Num[20];
  uint8_t str_hw_Type[40];
  uint8_t Reserved[4];
};

// CAN init config structure
struct ZCAN_CAN_INIT_CONFIG {
  int AccCode;
  int AccMask;
  int Reserved;
  uint8_t Filter;
  uint8_t Timing0;
  uint8_t Timing1;
  uint8_t Mode;
};

// CAN object structure (for TX/RX)
struct ZCAN_CAN_OBJ {
  uint32_t ID;
  uint32_t TimeStamp;
  uint8_t TimeFlag;
  int8_t SendType;
  int8_t RemoteFlag;
  int8_t ExternFlag;
  int8_t DataLen;
  uint8_t Data[8];
  uint8_t Reserved[3];
};

// Function pointer types for libusbcan.so
typedef int (*VCI_OpenDevice_t)(uint32_t DeviceType, uint32_t DeviceInd, uint32_t Reserved);
typedef int (*VCI_CloseDevice_t)(uint32_t DeviceType, uint32_t DeviceInd);
typedef int (*VCI_InitCAN_t)(uint32_t DeviceType, uint32_t DeviceInd, uint32_t CANInd, ZCAN_CAN_INIT_CONFIG* pInitConfig);
typedef int (*VCI_StartCAN_t)(uint32_t DeviceType, uint32_t DeviceInd, uint32_t CANInd);
typedef int (*VCI_ResetCAN_t)(uint32_t DeviceType, uint32_t DeviceInd, uint32_t CANInd);
typedef int (*VCI_Transmit_t)(uint32_t DeviceType, uint32_t DeviceInd, uint32_t CANInd, ZCAN_CAN_OBJ* pSend, uint32_t Len);
typedef int (*VCI_Receive_t)(uint32_t DeviceType, uint32_t DeviceInd, uint32_t CANInd, ZCAN_CAN_OBJ* pReceive, uint32_t Len, int WaitTime);
typedef int (*VCI_GetReceiveNum_t)(uint32_t DeviceType, uint32_t DeviceInd, uint32_t CANInd);
typedef int (*VCI_ReadBoardInfo_t)(uint32_t DeviceType, uint32_t DeviceInd, ZCAN_CAN_BOARD_INFO* pInfo);

/**
 * @brief MyActuator RMD hardware interface using USBCAN-UC12 (libusbcan.so)
 * 
 * This interface communicates with MyActuator RMD motors via USBCAN-UC12
 * adapter using the manufacturer's libusbcan.so library.
 */
class MyActuatorHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MyActuatorHardwareInterface)

  MyActuatorHardwareInterface() = default;
  ~MyActuatorHardwareInterface() override;

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
  // ========== Motor Data Structure ==========
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
    
    double last_position_command_deg{0.0};
  };

  // ========== Gripper Data Structure ==========
  struct GripperData
  {
    std::string joint_name;
    
    double position_state{0.0};
    double velocity_state{0.0};
    
    double position_command{0.0};
    
    double min_position{-0.01};
    double max_position{0.019};
    int min_dxl_position{2048};
    int max_dxl_position{0};
    
    int last_dxl_command{-1};
  };

  rclcpp::Logger getLogger() const;

  // ========== USBCAN Library Functions ==========
  bool loadUSBCANLibrary();
  void unloadUSBCANLibrary();
  bool openUSBCAN();
  void closeUSBCAN();
  bool initCANChannel(uint32_t channel);
  bool startCANChannel(uint32_t channel);
  bool resetCANChannel(uint32_t channel);
  
  // ========== CAN Communication ==========
  bool sendCANFrame(uint32_t id, const uint8_t* data, uint8_t len);
  bool receiveCANFrame(uint32_t& id, uint8_t* data, uint8_t& len, int timeout_ms = 100);
  int getReceiveCount();

  // ========== Motor Communication ==========
  bool initializeMotor(const hardware_interface::ComponentInfo & joint, size_t index);
  bool readMotorState(MotorData & motor);
  bool writeMotorCommand(MotorData & motor);
  bool setMotorAcceleration(int actuator_id, uint32_t accel_dps2);

  // ========== Batch Communication (Optimized) ==========
  void readAllMotorStates();
  void writeAllMotorCommands();
  void clearReceiveBuffer();

  // ========== Async Thread ==========
  bool startAsyncThread();
  void stopAsyncThread();
  void asyncThreadFunc();

  // ========== Holding Thread ==========
  bool startHoldingThread();
  void stopHoldingThread();
  void holdingThreadFunc();

  // ========== Gripper Functions ==========
  bool openGripperSerial();
  void closeGripperSerial();
  bool sendGripperCommand(int position);
  int positionToGripperDxl(double position_meters);

  // ========== USBCAN Library Handle ==========
  void* lib_handle_{nullptr};
  VCI_OpenDevice_t VCI_OpenDevice_{nullptr};
  VCI_CloseDevice_t VCI_CloseDevice_{nullptr};
  VCI_InitCAN_t VCI_InitCAN_{nullptr};
  VCI_StartCAN_t VCI_StartCAN_{nullptr};
  VCI_ResetCAN_t VCI_ResetCAN_{nullptr};
  VCI_Transmit_t VCI_Transmit_{nullptr};
  VCI_Receive_t VCI_Receive_{nullptr};
  VCI_GetReceiveNum_t VCI_GetReceiveNum_{nullptr};
  VCI_ReadBoardInfo_t VCI_ReadBoardInfo_{nullptr};

  // ========== USBCAN Configuration ==========
  uint32_t device_type_{USBCAN_II};
  uint32_t device_index_{0};
  uint32_t can_channel_{0};
  bool device_opened_{false};
  
  // Baudrate timing values
  // 0x1400 = 1M, 0x1c00 = 500k, 0x1c01 = 250k, 0x1c03 = 125k
  uint16_t baudrate_timing_{0x1400};

  // ========== Motor Members ==========
  std::vector<MotorData> motors_;

  // ========== Gripper Members ==========
  GripperData gripper_;
  bool gripper_enabled_{false};
  std::string gripper_port_;
  int gripper_baudrate_{115200};
  int gripper_serial_fd_{-1};

  // ========== Thread Members ==========
  std::thread async_thread_;
  std::atomic<bool> stop_async_thread_{false};
  std::mutex mutex_;

  std::thread holding_thread_;
  std::atomic<bool> stop_holding_thread_{false};

  // ========== Configuration ==========
  std::chrono::milliseconds cycle_time_{2};
  bool auto_home_{true};

  // Motor dynamics parameters
  uint32_t motor_acceleration_{1000};  // deg/s^2
  uint32_t motor_deceleration_{1000};  // deg/s^2
  double max_velocity_dps_{720.0};     // deg/s
  double default_velocity_dps_{360.0}; // deg/s

  // Performance monitoring
  double last_read_time_ms_{0.0};
  double last_write_time_ms_{0.0};
  double avg_read_time_ms_{0.0};
  double avg_write_time_ms_{0.0};
  uint64_t comm_count_{0};
  bool enable_timing_log_{false};

  enum class ControlMode { POSITION, VELOCITY, EFFORT };
  ControlMode active_control_mode_{ControlMode::POSITION};
};

}  // namespace myactuator_hardware

#endif  // MYACTUATOR_HARDWARE__MYACTUATOR_HARDWARE_INTERFACE_HPP_
