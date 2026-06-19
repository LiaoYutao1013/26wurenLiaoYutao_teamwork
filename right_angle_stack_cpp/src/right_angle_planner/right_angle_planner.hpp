#pragma once

#include <rclcpp/rclcpp.hpp>
#include <fsd_common_msgs/msg/map.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <string>
#include <vector>

// 锥桶记录：稳定ID + 世界坐标 + 颜色
struct ConeRecord {
    int id;
    double x, y;
    std::string color;
};

class RightAnglePlanner : public rclcpp::Node {
public:
    RightAnglePlanner();

private:
    // 回调函数
    void on_pose(const geometry_msgs::msg::PoseStamped::SharedPtr &msg);

    void on_map(const fsd_common_msgs::msg::Map::SharedPtr &msg);

    void on_timer();

    // 中心线生成：蓝黄锥桶配对 → 只取前方点 → 按距离排序
    std::vector<std::pair<double, double> > cone_centerline();

    // 发布RViz可视化标记
    void publish_markers(const std_msgs::msg::Header &header,
                         const std::vector<std::pair<double, double> > &points) const;

    // 参数
    double pair_distance_max_;

    // 位姿缓存
    geometry_msgs::msg::PoseStamped::SharedPtr car_pose_;

    // 锥桶地图
    std::vector<ConeRecord> blue_cones_;
    std::vector<ConeRecord> yellow_cones_;
    int next_cone_id_ = 0;

    // 订阅器、发布器、定时器
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<fsd_common_msgs::msg::Map>::SharedPtr map_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};