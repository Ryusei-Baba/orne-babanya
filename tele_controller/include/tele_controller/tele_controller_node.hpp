#ifndef TELE_CONTROLLER__TELE_CONTROLLER_NODE_HPP_
#define TELE_CONTROLLER__TELE_CONTROLLER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>

namespace tele_controller {

enum class Buttons : int {
    Circle = 1,
    Triangle = 2,
    Square = 3,
    L1 = 4,
    R1 = 5,
    L2 = 6,
    R2 = 7,
    Share = 8,
    Options = 9,
    PS = 10,
    L_PRESS = 11
};

enum class Axes : int {
    L_Y = 1,
    R_X = 3
};

class TeleController : public rclcpp::Node {
public:
    explicit TeleController(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    explicit TeleController(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void _subscriber_callback_joy(const sensor_msgs::msg::Joy::SharedPtr msg);

    bool is_pressed(int button_index, const std::vector<int>& buttons);
    void _publish_route_command(const std::string& command);

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _subscription_joy;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_vel;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_stop;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_restart;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_save;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_autonomous;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_nav_start;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_action_command;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_save_image;

    rclcpp::QoS _qos{10};

    const double linear_max_vel;
    const double angular_max_vel;
    bool is_autonomous;
    bool is_save;
    bool is_autorun;

    std::vector<int> prev_buttons;
    std::map<int, std::chrono::steady_clock::time_point> last_pressed_time;
    std::chrono::milliseconds debounce_interval;
    std::string prev_command;

    double dtor(double deg) const { return deg * M_PI / 180.0; }
};

}  // namespace tele_controller

#endif  // TELE_CONTROLLER__TELE_CONTROLLER_NODE_HPP_