#pragma once

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/handle.hpp"
#include "icart_mini_driver/visibility_control.h"

extern "C" {
  #include <ypspur.h>
}

namespace icart_mini_driver
{

class ICART_MINI_DRIVER_PUBLIC IcartMiniDriver : public hardware_interface::SystemInterface
{
public:
  IcartMiniDriver() = default;

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  hardware_interface::HardwareInfo info_;
  double cmd_left_ = 0.0;
  double cmd_right_ = 0.0;
  double pos_left_ = 0.0;
  double pos_right_ = 0.0;
  double vel_left_ = 0.0;
  double vel_right_ = 0.0;

  double wheel_separation_ = 0.4615;
  double linear_vel_limit_ = 1.5;
  double linear_accel_limit_ = 1.5;
  double angular_vel_limit_ = 3.14;
  double angular_accel_limit_ = 3.14;
};

}  // namespace icart_mini_driver