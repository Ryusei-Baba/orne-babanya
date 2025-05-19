#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <chrono>
#include <ypspur.h>
#include "icart_mini_driver/visibility_control.h"

namespace icart_mini_driver {

class IcartMiniDriver : public rclcpp::Node
{
public:
    IcartMiniDriver();
    explicit IcartMiniDriver(const rclcpp::NodeOptions & options);
    void read_param();
    void reset_param();
    void bringup_ypspur();
    void joint_states();
    void odometry();
    bool loop();

private:
    // Initialize
    void initialize();

    // Callback
    void cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg);

    // Parameters
    std::string odom_frame_id;
    std::string base_frame_id;
    std::string left_wheel_joint;
    std::string right_wheel_joint;
    int loop_hz;
    double liner_vel_lim, liner_accel_lim, angular_vel_lim, angular_accel_lim;
    bool odom_from_ypspur, debug_mode = false;

    // ROS interfaces
    geometry_msgs::msg::Twist::SharedPtr cmd_vel_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;
    rclcpp::TimerBase::SharedPtr loop_timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Internal state
    sensor_msgs::msg::JointState js;
    nav_msgs::msg::Odometry odom;
    geometry_msgs::msg::TransformStamped odom_trans;
    tf2::Vector3 z_axis_;
    float dt;
    double tf_time_offset_;
};

} // namespace icart_mini_driver