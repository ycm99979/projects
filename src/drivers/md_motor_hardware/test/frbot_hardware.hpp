#ifndef FRBOT_HARDWARE__FRBOT_HARDWARE_HPP_
#define FRBOT_HARDWARE__FRBOT_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "geometry_msgs/msg/twist.hpp"

namespace frbot_hardware
{

class FrbotHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(FrbotHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(
  const rclcpp_lifecycle::State & previous_state) override;

  // on_activate / on_deactivate 는 필요하면 나중에 override 해도 되고, 
  // 일단 기본 구현으로 두고 싶으면 안 적어도 됨.

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // 이 하드웨어용 노드
  rclcpp::Node::SharedPtr hardware_node_;

  // 명령(컨트롤러 → 하드웨어) Twist
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  // 상태(하드웨어 → 컨트롤러) Twist
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr state_sub_;

  void state_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  std::mutex mutex_;
  double latest_linear_x_  = 0.0;  // Twist.linear.x
  double latest_angular_z_ = 0.0;  // Twist.angular.z

  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
};

}  // namespace frbot_hardware

#endif  // FRBOT_HARDWARE__FRBOT_HARDWARE_HPP_
