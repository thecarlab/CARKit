#include <rclcpp/rclcpp.hpp>
#include "carkit_pure_pursuit/carkit_pure_pursuit.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<carkit_pure_pursuit::PurePursuitController>();
    
    RCLCPP_INFO(node->get_logger(), "Starting Pure Pursuit Controller...");
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
} 