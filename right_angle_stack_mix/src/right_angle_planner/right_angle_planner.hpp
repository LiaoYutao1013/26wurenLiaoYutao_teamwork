#pragma once

#include <rclcpp/rclcpp.hpp>
#include <fsd_common_msgs/msg/map.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <string>
#include <vector>

class RightAnglePlanner : public rclcpp::Node {
public:
    RightAnglePlanner();

private:
    // 回调函数
    void on_pose(const geometry_msgs::msg::PoseStamped::SharedPtr &msg);

    void on_map(const fsd_common_msgs::msg::Map::SharedPtr &msg);

    void on_timer();

    // 前方中心规划线
    std::vector<std::pair<double, double> > cone_centerline();

    // 发布RViz可视化标记
    void publish_markers(const std_msgs::msg::Header &header,
                         const std::vector<std::pair<double, double> > &points) const;

    // 构建并发布规划路径
    void publish_path(const std_msgs::msg::Header &header,
                     const std::vector<std::pair<double, double> > &points) const;

    // 最大匹配距离参数
    double pair_distance_max_;

    // 位姿缓存
    geometry_msgs::msg::PoseStamped::SharedPtr car_pose_;

    // 锥桶地图
    std::vector<std::pair<double, double>> blue_cones_;
    std::vector<std::pair<double, double>> yellow_cones_;

    // 订阅器、发布器、定时器
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<fsd_common_msgs::msg::Map>::SharedPtr map_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};