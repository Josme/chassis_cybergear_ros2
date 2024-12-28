#include <rclcpp/rclcpp.hpp>
#include "chassis_control_m5/chassis_control.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ChassisControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

