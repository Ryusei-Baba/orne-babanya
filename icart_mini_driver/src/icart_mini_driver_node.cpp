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
  info_ = info;

  for (const auto & joint : info_.joints) {
    std::cout << "Joint found: " << joint.name << std::endl;
  }

  auto get_param = [&](const std::string & name, double & out, double def) {
    if (info_.hardware_parameters.count(name)) {
      out = std::stod(info_.hardware_parameters.at(name));
    } else {
      out = def;
    }
  };

  get_param("linear_vel_limit", linear_vel_limit_, 1.5);
  get_param("linear_accel_limit", linear_accel_limit_, 1.5);
  get_param("angular_vel_limit", angular_vel_limit_, 3.14);
  get_param("angular_accel_limit", angular_accel_limit_, 3.14);
  get_param("wheel_separation", wheel_separation_, 0.4615);

  cmd_left_ = cmd_right_ = 0.0;
  pos_left_ = pos_right_ = 0.0;
  vel_left_ = vel_right_ = 0.0;

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

  cmd_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &cmd_left_));

  cmd_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &cmd_right_));

  return cmd_interfaces;
}

hardware_interface::CallbackReturn IcartMiniDriver::on_activate(const rclcpp_lifecycle::State &)
{
  if (YPSpur_init() < 0) {
    std::cerr << "[IcartMiniDriver] ERROR: Failed to initialize YPSpur" << std::endl;
    return CallbackReturn::ERROR;
  }

  Spur_set_vel(linear_vel_limit_);
  Spur_set_accel(linear_accel_limit_);
  Spur_set_angvel(angular_vel_limit_);
  Spur_set_angaccel(angular_accel_limit_);
  Spur_stop();

  std::cout << "[IcartMiniDriver] YPSpur activated." << std::endl;
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IcartMiniDriver::on_deactivate(const rclcpp_lifecycle::State &)
{
  Spur_free();
  std::cout << "[IcartMiniDriver] YPSpur deactivated." << std::endl;
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type IcartMiniDriver::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  double theta_r = 0.0, theta_l = 0.0;
  if (YP_get_wheel_ang(&theta_r, &theta_l) < 0) {
    std::cerr << "[IcartMiniDriver] ERROR: Failed to get wheel angles" << std::endl;
    return hardware_interface::return_type::ERROR;
  }

  pos_left_ = theta_l;
  pos_right_ = theta_r;

  double v, w;
  YPSpur_get_vel(&v, &w);
  double wheel_sep = wheel_separation_;

  vel_left_ = v - w * 0.5 * wheel_sep;
  vel_right_ = v + w * 0.5 * wheel_sep;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type IcartMiniDriver::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  double wheel_sep = wheel_separation_;

  double v = (cmd_left_ + cmd_right_) * 0.5;
  double w = (cmd_right_ - cmd_left_) / wheel_sep;
  
  Spur_set_vel(linear_vel_limit_);
  Spur_set_accel(linear_accel_limit_);
  Spur_set_angvel(angular_vel_limit_);
  Spur_set_angaccel(angular_accel_limit_);
  
  Spur_vel(v, w);

  return hardware_interface::return_type::OK;
}

}  // namespace icart_mini_driver

PLUGINLIB_EXPORT_CLASS(icart_mini_driver::IcartMiniDriver, hardware_interface::SystemInterface)