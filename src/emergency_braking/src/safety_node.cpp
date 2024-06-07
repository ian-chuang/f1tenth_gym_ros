// used https://github.com/CL2-UWaterloo/f1tenth_ws/tree/main/src/safety_node as reference

#include "emergency_braking/safety.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Safety>());
    rclcpp::shutdown();
    return 0;
}
