#include "tortoisebot_waypoints/tortoisebot_ac.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointClient>();
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}