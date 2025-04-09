#include "icart_mini_driver/icart_mini_driver_node.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <cmath>
#include <iostream>

extern "C" {
  #include <ypspur.h>
}

namespace icart_mini_driver
{

hardware_interface::CallbackReturn IcartMiniDriver::on_init(const hardware_interface::HardwareInfo & info)
{
  // hardware_info を保存
  info_ = info;

  // ジョイント確認ログ
  for (const auto & joint : info_.joints) {
    std::cout << "Joint found: " << joint.name << std::endl;
  }

  // YP-Spur 初期化
  if (YPSpur_init() < 0) {
    std::cerr << "[IcartMiniDriver] ERROR: Failed to initialize YPSpur" << std::endl;
    return CallbackReturn::ERROR;
  }

  std::cout << "[IcartMiniDriver] YPSpur initialized successfully." << std::endl;

  // 初期状態リセット
  cmd_left_ = 0.0;
  cmd_right_ = 0.0;
  pos_left_ = 0.0;
  pos_right_ = 0.0;
  vel_left_ = 0.0;
  vel_right_ = 0.0;

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> IcartMiniDriver::export_state_interfaces()
{
  return {
    {info_.joints[0].name, hardware_interface::HW_IF_POSITION, &pos_left_},
    {info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &vel_left_},
    {info_.joints[1].name, hardware_interface::HW_IF_POSITION, &pos_right_},
    {info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &vel_right_}
  };
}

std::vector<hardware_interface::CommandInterface> IcartMiniDriver::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> cmd_interfaces;

  cmd_interfaces.emplace_back(hardware_interface::CommandInterface(
    info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &cmd_left_));
  cmd_interfaces.emplace_back(hardware_interface::CommandInterface(
    info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &cmd_right_));

  return cmd_interfaces;
}


hardware_interface::CallbackReturn IcartMiniDriver::on_activate(const rclcpp_lifecycle::State &)
{
  std::cout << "[IcartMiniDriver] Activated." << std::endl;
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IcartMiniDriver::on_deactivate(const rclcpp_lifecycle::State &)
{
  std::cout << "[IcartMiniDriver] Deactivated." << std::endl;
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type IcartMiniDriver::read(const rclcpp::Time &, const rclcpp::Duration & period)
{
  double v, w;
  YPSpur_get_vel(&v, &w);

  double wheel_sep = 0.4615;  // fallback default

  if (info_.joints[0].parameters.count("wheel_separation")) {
    wheel_sep = std::stod(info_.joints[0].parameters.at("wheel_separation"));
  } else {
    std::cerr << "[IcartMiniDriver] WARNING: 'wheel_separation' parameter not found, using default: " << wheel_sep << std::endl;
  }

  vel_left_ = v - w * 0.5 * wheel_sep;
  vel_right_ = v + w * 0.5 * wheel_sep;

  pos_left_ += vel_left_ * period.seconds();
  pos_right_ += vel_right_ * period.seconds();

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type IcartMiniDriver::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  double wheel_sep = 0.4615;  // fallback default

  if (info_.joints[0].parameters.count("wheel_separation")) {
    wheel_sep = std::stod(info_.joints[0].parameters.at("wheel_separation"));
  } else {
    std::cerr << "[IcartMiniDriver] WARNING: 'wheel_separation' parameter not found, using default: " << wheel_sep << std::endl;
  }

  double v = (cmd_left_ + cmd_right_) * 0.5;
  double w = (cmd_right_ - cmd_left_) / wheel_sep;

  YPSpur_vel(v, w);
  return hardware_interface::return_type::OK;
}

}  // namespace icart_mini_driver

PLUGINLIB_EXPORT_CLASS(icart_mini_driver::IcartMiniDriver, hardware_interface::SystemInterface)
