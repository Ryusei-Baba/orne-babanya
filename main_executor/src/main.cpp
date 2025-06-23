#include <rclcpp/rclcpp.hpp>

#include "icart_mini_driver/icart_mini_driver_node.hpp"
#include "tele_controller/tele_controller.hpp"


int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);
    rclcpp::executors::MultiThreadedExecutor exec;

    rclcpp::NodeOptions nodes_option;
    nodes_option.allow_undeclared_parameters(true);
    nodes_option.automatically_declare_parameters_from_overrides(true);

    auto icart_mini_driver_node = std::make_shared<icart_mini_driver::IcartMiniDriver>(nodes_option);
    auto tele_controller_node = std::make_shared<tele_controller::TeleController>(nodes_option);
    
    exec.add_node(icart_mini_driver_node);
    exec.add_node(tele_controller_node)

    exec.spin();
    rclcpp::shutdown();
    return 0;
}