#include "tele_controller/tele_controller_node.hpp"

using namespace std;

namespace tele_controller {

TeleController::TeleController(const rclcpp::NodeOptions& options) : TeleController("", options) {}

TeleController::TeleController(const std::string& name_space, const rclcpp::NodeOptions& options)
: rclcpp::Node("controller_node", name_space, options),
linear_max_vel(get_parameter("linear_max_vel").as_double()),
angular_max_vel(get_parameter("angular_max_vel").as_double()),
is_autonomous(get_parameter("autonomous_flag").as_bool()),
is_save(false),
is_autorun(false),
prev_buttons(13, 0),
debounce_interval(std::chrono::milliseconds(500))
{
    declare_parameter("linear_max_vel", 0.5);
    declare_parameter("angular_max_vel", 1.57);
    declare_parameter("autonomous_flag", false);

    _subscription_joy = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy",
        _qos,
        std::bind(&TeleController::_subscriber_callback_joy, this, std::placeholders::_1)
    );

    publisher_vel = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", _qos);
    publisher_stop = this->create_publisher<std_msgs::msg::Empty>("stop", _qos);
    publisher_restart = this->create_publisher<std_msgs::msg::Empty>("restart", _qos);
    publisher_save = this->create_publisher<std_msgs::msg::Bool>("save", _qos);
    publisher_autonomous = this->create_publisher<std_msgs::msg::Bool>("autonomous", _qos);
    publisher_nav_start = this->create_publisher<std_msgs::msg::Empty>("nav_start", _qos);
    publisher_action_command = this->create_publisher<std_msgs::msg::String>("cmd_route", _qos);
    publisher_save_image = this->create_publisher<std_msgs::msg::Empty>("save_image", _qos);

    publisher_restart->publish(*std::make_shared<std_msgs::msg::Empty>());
}

bool TeleController::is_pressed(int button_index, const std::vector<int>& buttons) {
    auto now = std::chrono::steady_clock::now();
    auto last_time = last_pressed_time[button_index];
    
    if (buttons[button_index] == 1 && prev_buttons[button_index] == 0 && 
        (now - last_time) > debounce_interval) {
        last_pressed_time[button_index] = now;
        return true;
    }
    return false;
}

void TeleController::_publish_route_command(const std::string& command) {
    auto msg = std::make_shared<std_msgs::msg::String>();
    msg->data = command;
    publisher_action_command->publish(*msg);
    RCLCPP_INFO(this->get_logger(), "ルート指令送信: %s", command.c_str());
}

void TeleController::_subscriber_callback_joy(const sensor_msgs::msg::Joy::SharedPtr msg){
    std::vector<int> buttons(msg->buttons.begin(), msg->buttons.end());
    std::vector<double> axes(msg->axes.begin(), msg->axes.end());

    if (is_pressed(static_cast<int>(Buttons::Options), buttons) || 
        is_pressed(static_cast<int>(Buttons::Circle), buttons)) {
        is_save = !is_save;
        auto msg_save = std::make_shared<std_msgs::msg::Bool>();
        msg_save->data = is_save;
        publisher_save->publish(*msg_save);
        RCLCPP_INFO(this->get_logger(), "データ収集フラグ: %d", is_save);
    }

    if (is_pressed(static_cast<int>(Buttons::Share), buttons)) {
        is_autonomous = !is_autonomous;
        auto msg_autonomous = std::make_shared<std_msgs::msg::Bool>();
        msg_autonomous->data = is_autonomous;
        publisher_autonomous->publish(*msg_autonomous);
        RCLCPP_INFO(this->get_logger(), "自動フラグ: %d", is_autonomous);
    }

    bool triangle_pressed = buttons[static_cast<int>(Buttons::Triangle)] == 1;
    bool left_pressed = buttons[static_cast<int>(Buttons::L1)] == 1 || 
                        buttons[static_cast<int>(Buttons::L2)] == 1;
    bool right_pressed = buttons[static_cast<int>(Buttons::R1)] == 1 || 
                         buttons[static_cast<int>(Buttons::R2)] == 1;

    std::string current_command = "roadside";
    
    if (triangle_pressed) {
        current_command = "straight";
    } else if (left_pressed) {
        current_command = "left";
    } else if (right_pressed) {
        current_command = "right";
    }
    
    if (prev_command != current_command) {
        _publish_route_command(current_command);
        publisher_save_image->publish(*std::make_shared<std_msgs::msg::Empty>());
        prev_command = current_command;
    }

    if (is_pressed(static_cast<int>(Buttons::L_PRESS), buttons)) {
        is_autorun = !is_autorun;
        RCLCPP_INFO(this->get_logger(), "オートラン: %d", is_autorun);
    }

    if (!is_autonomous) {
        auto twist = std::make_shared<geometry_msgs::msg::Twist>();
        if (is_autorun) {
            twist->linear.x = linear_max_vel;
        } else {
            twist->linear.x = linear_max_vel * axes[static_cast<int>(Axes::L_Y)];
        }
        twist->angular.z = angular_max_vel * axes[static_cast<int>(Axes::R_X)];
        publisher_vel->publish(*twist);
    }

    prev_buttons = buttons;
}

}  // namespace tele_controller