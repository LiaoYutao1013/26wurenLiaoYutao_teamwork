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

    // 锥桶地图维护：匹配新消息到已有索引的锥桶，保持ID不变
    void update_cone_map(const fsd_common_msgs::msg::Map &msg);

    // 匹配更新一组锥桶：新锥桶与已有记录最近邻匹配，更新位置或追加新记录
    void match_cones(std::vector<ConeRecord> &existing,
                     const std::vector<fsd_common_msgs::msg::Cone> &new_cones,
                     const std::string &color);

    // 中心线生成：蓝黄锥桶配对 → 中点排序
    std::vector<std::pair<double, double> > cone_centerline();

    // 路径合并：冻结全部已发布路径，拼接新中心线后缀
    std::vector<std::pair<double, double> > merge_path
    (const std::vector<std::pair<double, double> > &new_centerline);

    // 发布RViz可视化标记
    void publish_markers(const std_msgs::msg::Header &header,
                         const std::vector<std::pair<double, double> > &points) const;

    // 参数
    double pair_distance_max_;
    double cone_match_distance_;
    double passed_filter_distance_;

    // 地图与位姿缓存
    fsd_common_msgs::msg::Map::SharedPtr map_;
    geometry_msgs::msg::PoseStamped::SharedPtr car_pose_;

    // 锥桶地图：带稳定索引的蓝/黄锥桶列表
    std::vector<ConeRecord> blue_cones_;
    std::vector<ConeRecord> yellow_cones_;
    int next_cone_id_ = 0;

    // 路径状态：已发布路径（冻结，永不修改）
    std::vector<std::pair<double, double> > published_path_;

    // 订阅器、发布器、定时器
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<fsd_common_msgs::msg::Map>::SharedPtr map_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};