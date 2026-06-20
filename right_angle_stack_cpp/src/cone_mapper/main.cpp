#include "cone_mapper.hpp"
#include <rclcpp/rclcpp.hpp>

int main(const int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ConeMapper>());
    rclcpp::shutdown();
    return 0;
}