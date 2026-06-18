// 定位融合节点入口

#include <rclcpp/rclcpp.hpp>
#include "localization_fusion.hpp"

int main(const int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalizationFusion>());
    rclcpp::shutdown();
    return 0;
}