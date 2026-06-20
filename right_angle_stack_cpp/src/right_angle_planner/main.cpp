#include "right_angle_planner.hpp"

int main(const int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RightAnglePlanner>());
    rclcpp::shutdown();
    return 0;
}
