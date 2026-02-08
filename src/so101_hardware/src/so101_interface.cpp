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

  joint_ids_.resize(info_.joints.size());
  for (size_t i = 0; i < info_.joints.size(); i++) {
    // 提前解析字符串 ID，避免在 write 循环中重复解析
    joint_ids_[i] = static_cast<uint8_t>(std::stoi(info_.joints[i].parameters.at("id")));
  }

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

// hardware_interface::return_type SO101HardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
// {
//   static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
  
//   const uint16_t speed = 100;
//   const uint8_t accel = 10;

//   const size_t n = info_.joints.size();
//   std::vector<uint8_t> ids(n);
//   std::vector<int16_t> positions(n);
//   std::vector<uint16_t> speeds(n, speed);
//   std::vector<uint8_t> accels(n, accel);

//   for (size_t i = 0; i < n; i++) {
//     const int id = std::stoi(info_.joints[i].parameters.at("id"));
//     ids[i] = static_cast<uint8_t>(id);

//     double cmd_radian = hw_commands_[i];
//     if (id == 6) cmd_radian -= (M_PI / 2.0);

//     const double target_pos = (cmd_radian * 2048.0 / M_PI) + 2048.0;
//     const double clamped = std::fmax(0.0, std::fmin(4095.0, target_pos));
//     positions[i] = static_cast<int16_t>(clamped);

//     RCLCPP_INFO_THROTTLE(
//       rclcpp::get_logger("SO101HardwareInterface"),
//       steady_clock,
//       1000,
//       "write joint[%zu] id=%d cmd(rad)=%.3f pos=%d",
//       i, id, hw_commands_[i], positions[i]);
//   }

//   sms_sts.SyncWritePosEx(ids.data(), n, positions.data(), speeds.data(), accels.data());
//   return hardware_interface::return_type::OK;
// }

// 建议：在类成员中增加 std::vector<uint8_t> joint_ids_ 并由 on_init 初始化

hardware_interface::return_type SO101HardwareInterface::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
  const size_t n = info_.joints.size();
  
  // 预分配内存，避免每帧分配
  static std::vector<uint8_t> ids(n);
  static std::vector<int16_t> positions(n);
  static std::vector<uint16_t> speeds(n);
  static std::vector<uint8_t> accels(n);

  double dt = period.seconds();
  if (dt < 1e-6) dt = 0.02;

  // 这里的参数建议根据手感调整
  const double Kp = 15.0;            // 适中的增益，不要直接用 1/dt
  const uint16_t SPEED_MIN = 20;     
  const uint16_t SPEED_MAX = 1000;   // STS3215 建议不要超过 2400
  const uint8_t  ACC_VAL   = 20;     // 加速度建议写死，轨迹更平滑

  for (size_t i = 0; i < n; i++) {
    // 假设你已经在 on_init 缓存了 ID
    uint8_t id = joint_ids_[i];
    ids[i] = id;

    //RCLCPP_INFO_THROTTLE(rclcpp::get_logger("SO101HardwareInterface"), "id set :%d ", ids[i]);

    // 1. 目标位置转换
    double cmd_rad = hw_commands_[i];
    if (id == 6) cmd_rad -= (M_PI / 2.0);
    double target_cnt = (cmd_rad * 2048.0 / M_PI) + 2048.0;
    positions[i] = static_cast<int16_t>(std::clamp(target_cnt, 0.0, 4095.0));

    // 2. 速度计算：使用 P 控制逻辑
    double cur_rad = hw_states_[i];
    if (id == 6) cur_rad -= (M_PI / 2.0);
    double cur_cnt = (cur_rad * 2048.0 / M_PI) + 2048.0;

    double error = std::fabs(target_cnt - cur_cnt);
    
    // 核心修改：使用 Kp 增益
    double v = error * Kp; 

    // 死区处理：防止原地抖动
    if (error < 5.0) v = 0.0; 

    speeds[i] = (v <= 0.0) ? 0 : static_cast<uint16_t>(std::clamp(v, (double)SPEED_MIN, (double)SPEED_MAX));
    accels[i] = ACC_VAL;

    // 调试打印：建议仅在排查时开启，或加大间隔
    RCLCPP_INFO_THROTTLE(rclcpp::get_logger("SO101HardwareInterface"), steady_clock, 2000,
      "ID:%d Err:%.1f Spd:%u", id, error, speeds[i]);
  }

  sms_sts.SyncWritePosEx(ids.data(), n, positions.data(), speeds.data(), accels.data());
  return hardware_interface::return_type::OK;
}

}

PLUGINLIB_EXPORT_CLASS(so101_hardware::SO101HardwareInterface, hardware_interface::SystemInterface)
