#include "icart_mini_driver/icart_mini_driver_node.hpp"

namespace icart_mini_driver {

IcartMiniDriver::IcartMiniDriver() 
: Node("icart_mini_driver")
{
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 1);
    js_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&IcartMiniDriver::cmd_vel_cb, this, std::placeholders::_1));
    loop_timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&IcartMiniDriver::loop, this));
}

IcartMiniDriver::IcartMiniDriver(const rclcpp::NodeOptions & options)
: Node("icart_mini_driver", options)
{
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 1);
    js_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&IcartMiniDriver::cmd_vel_cb, this, std::placeholders::_1));
    loop_timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&IcartMiniDriver::loop, this));
}

// cmd_vel callback
void IcartMiniDriver::cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    cmd_vel_ = msg;
    Spur_vel(msg->linear.x, msg->angular.z);
}

// Read parameters
void IcartMiniDriver::read_param()
{
    declare_parameter("odom_frame_id", "odom");
    declare_parameter("base_frame_id", "base_footprint");
    declare_parameter("Hz", 40);
    declare_parameter("left_wheel_joint", "left_wheel_joint");
    declare_parameter("right_wheel_joint", "right_wheel_joint");
    declare_parameter("liner_vel_lim", 1.5);
    declare_parameter("liner_accel_lim", 1.5);
    declare_parameter("angular_vel_lim", 3.14);
    declare_parameter("angular_accel_lim", 3.14);
    declare_parameter("calculate_odom_from_ypspur", true);
    declare_parameter("debug_mode", false);

    get_parameter("odom_frame_id", odom_frame_id);
    get_parameter("base_frame_id", base_frame_id);
    get_parameter("left_wheel_joint", left_wheel_joint);
    get_parameter("right_wheel_joint", right_wheel_joint);
    get_parameter("liner_vel_lim", liner_vel_lim);
    get_parameter("liner_accel_lim", liner_accel_lim);
    get_parameter("angular_vel_lim", angular_vel_lim);
    get_parameter("angular_accel_lim", angular_accel_lim);
    get_parameter("Hz", loop_hz);
    get_parameter("calculate_odom_from_ypspur", odom_from_ypspur);
    get_parameter("debug_mode", debug_mode);
    RCLCPP_INFO(this->get_logger(), "Set param!!");
}

// Reset parameters
void IcartMiniDriver::reset_param()
{
    z_axis_.setX(0);
    z_axis_.setY(0);
    z_axis_.setZ(1);
    cmd_vel_->linear.x = 0.0;
    cmd_vel_->angular.z = 0.0;
    odom.pose.pose.position.x = 0;
    odom.pose.pose.position.y = 0;
    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(z_axis_, 0));
    odom.twist.twist.linear.x = 0;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = 0;

    js.name.push_back(left_wheel_joint);
    js.name.push_back(right_wheel_joint);
    js.position.resize(2);
    js.velocity.resize(2);
}

// Bring up YP-Spur
void IcartMiniDriver::bringup_ypspur()
{
    if (Spur_init() > 0)
    {
        RCLCPP_INFO(this->get_logger(), "Bringup ypspur!!");
        Spur_stop();
        Spur_free();
        Spur_set_pos_GL(0, 0, 0);
        Spur_set_vel(liner_vel_lim);
        Spur_set_accel(liner_accel_lim);
        Spur_set_angvel(angular_vel_lim);
        Spur_set_angaccel(angular_accel_lim);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Disconnected ypspur");
    }
}

// Publish joint states
void IcartMiniDriver::joint_states()
{
    rclcpp::Time js_t = this->now();
    const rclcpp::Time current_stamp_js(js_t);
    double l_ang_pos{}, r_ang_pos{}, l_wheel_vel{}, r_wheel_vel{};
    YP_get_wheel_ang(&l_ang_pos, &r_ang_pos);
    YP_get_wheel_vel(&l_wheel_vel, &r_wheel_vel);
    js.header.stamp = current_stamp_js;
    js.header.frame_id = "base_link";
    js.position[0] = -l_ang_pos;
    js.position[1] = -r_ang_pos;
    js.velocity[0] = l_wheel_vel;
    js.velocity[1] = r_wheel_vel;
    js_pub_->publish(js);
}

// Compute and publish odometry
void IcartMiniDriver::odometry()
{
    double x, y, yaw, v, w;
    z_axis_.setX(0);
    z_axis_.setY(0);
    z_axis_.setZ(1);
    rclcpp::Time t = this->now();
    const rclcpp::Time current_stamp(t);

    if (odom_from_ypspur)
    {
        Spur_get_pos_GL(&x, &y, &yaw);
        Spur_get_vel(&v, &w);
    }
    else
    {
        v = cmd_vel_->linear.x;
        w = cmd_vel_->angular.z;
        yaw = tf2::getYaw(odom.pose.pose.orientation) + dt * w;
        x = odom.pose.pose.position.x + dt * v * cosf(yaw);
        y = odom.pose.pose.position.y + dt * v * sinf(yaw);
    }

    odom.header.stamp = current_stamp;
    odom.header.frame_id = odom_frame_id;
    odom.child_frame_id = base_frame_id;
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(z_axis_, yaw));
    odom.twist.twist.linear.x = v;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = w;
    odom_pub_->publish(odom);

    odom_trans.header.stamp = current_stamp + rclcpp::Duration::from_seconds(tf_time_offset_);
    odom_trans.header.frame_id = odom_frame_id;
    odom_trans.child_frame_id = base_frame_id;
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0;
    odom_trans.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(odom_trans);
}

// Main loop
bool IcartMiniDriver::loop()
{
    if (!YP_get_error_state())
    {
        odometry();
        joint_states();
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Disconnected ypspur reconnect ypspur");
        bringup_ypspur();
        return false;
    }
    return true;
}
} // namespace icart_mini_driver