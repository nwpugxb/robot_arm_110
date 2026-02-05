#include "so101_hardware/so101_interface.hpp"
#include "SCServo.h"
#include <pluginlib/class_list_macros.hpp>
#include <cmath>
static rclcpp::Clock steady_clock(RCL_STEADY_TIME);

namespace so101_hardware
{
SMS_STS sms_sts;

hardware_interface::CallbackReturn SO101HardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  hw_states_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);
  std::string device = info_.hardware_parameters.count("device") ? info_.hardware_parameters.at("device") : "/dev/ttyUSB0";
  if (!sms_sts.begin(1000000, device.c_str())) {
    RCLCPP_ERROR(rclcpp::get_logger("SO101HardwareInterface"), "Failed to init STS servos!");
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SO101HardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("SO101HardwareInterface"), "Activating hardware... Synchronizing positions.");

  for (size_t i = 0; i < info_.joints.size(); i++) {
    int id = std::stoi(info_.joints[i].parameters.at("id"));
    sms_sts.writeWord(id, 16, 300); // 设置安全扭矩

    int pos = sms_sts.ReadPos(id);
    if (pos != -1) {
      // 算出物理弧度
      double physical_radian = (static_cast<double>(pos) - 2048.0) * (M_PI / 2048.0);
      
      // --- 关键修正：同步时也必须考虑 joint_6 的逻辑偏移 ---
      if (id == 6) {
        hw_states_[i] = physical_radian + (M_PI / 2.0); // 物理 -90 变为 逻辑 0
      } else {
        hw_states_[i] = physical_radian;
      }
      
      // 将修正后的逻辑弧度同步给指令变量
      hw_commands_[i] = hw_states_[i];
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> SO101HardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> SO101HardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::return_type SO101HardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (size_t i = 0; i < hw_states_.size(); i++) {
    int id = std::stoi(info_.joints[i].parameters.at("id"));
    int pos = sms_sts.ReadPos(id);
    if (pos != -1) {
      double physical_radian = (static_cast<double>(pos) - 2048.0) * (M_PI / 2048.0);
      
      // --- 核心修改：对 Joint 6 读回的值加回 90度 ---
      if (id == 6) {
        hw_states_[i] = physical_radian + (M_PI / 2.0);
      } else {
        hw_states_[i] = physical_radian;
      }
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SO101HardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  static rclcpp::Clock steady_clock(RCL_STEADY_TIME);

  const size_t n = info_.joints.size();
  std::vector<uint8_t> ids(n);
  std::vector<int16_t> positions(n);
  std::vector<uint16_t> speeds(n, 100);
  std::vector<uint8_t> accels(n, 10);

  for (size_t i = 0; i < n; i++) {
    const int id = std::stoi(info_.joints[i].parameters.at("id"));
    ids[i] = static_cast<uint8_t>(id);

    double cmd_radian = hw_commands_[i];
    if (id == 6) cmd_radian -= (M_PI / 2.0);

    const double target_pos = (cmd_radian * 2048.0 / M_PI) + 2048.0;
    const double clamped = std::fmax(0.0, std::fmin(4095.0, target_pos));
    positions[i] = static_cast<int16_t>(clamped);

    RCLCPP_INFO_THROTTLE(
      rclcpp::get_logger("SO101HardwareInterface"),
      steady_clock,
      1000,
      "write joint[%zu] id=%d cmd(rad)=%.3f pos=%d",
      i, id, hw_commands_[i], positions[i]);
  }

  sms_sts.SyncWritePosEx(ids.data(), n, positions.data(), speeds.data(), accels.data());
  return hardware_interface::return_type::OK;
}

}

PLUGINLIB_EXPORT_CLASS(so101_hardware::SO101HardwareInterface, hardware_interface::SystemInterface)
