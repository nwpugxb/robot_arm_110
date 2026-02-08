#ifndef SO101_HARDWARE__SO101_INTERFACE_HPP_
#define SO101_HARDWARE__SO101_INTERFACE_HPP_

#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <vector>
#include <string>

namespace so101_hardware
{
class SO101HardwareInterface : public hardware_interface::SystemInterface
{
public:
  // 使用 hardware_interface::CallbackReturn 以确保类型绝对匹配
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  // 注意：复数形式 interfaces
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;
  std::vector<uint8_t> joint_ids_;
};
}  // namespace so101_hardware
#endif
