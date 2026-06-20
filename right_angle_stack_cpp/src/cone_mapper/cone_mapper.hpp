// 锥桶建图节点头文件：将局部感知锥桶变换到世界坐标系，合并邻近锥桶，发布全局地图
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <fsd_common_msgs/msg/cone.hpp>
#include <fsd_common_msgs/msg/map.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <map>
#include <string>
#include <vector>

// 路标：世界坐标 + 被观测次数 + 置信度
struct Landmark {
    double x, y;
    int count;
    double pose_confidence;
    double color_confidence;
};

class ConeMapper : public rclcpp::Node {
public:
    ConeMapper();

private:
    // 位姿回调：记录车辆当前位置
    void on_pose(const geometry_msgs::msg::PoseStamped::SharedPtr &msg);

    // 感知回调（Map 格式）
    void on_map_detection(const fsd_common_msgs::msg::Map::SharedPtr &msg);

    // 处理一组锥桶：坐标变换 → 合并
    void process_cones(const std::string &frame_id,
                       const std::vector<fsd_common_msgs::msg::Cone> &cones,
                       const std::string &fallback_color);

    // 将锥桶坐标转换到世界坐标系
    bool to_world(const std::string &frame_id,
                  const fsd_common_msgs::msg::Cone &cone,
                  double &wx, double &wy) const;

    // 合并路标：最近邻匹配，距离小于阈值则加权平均更新，否则新增
    void merge_landmark(const std::string &color,
                        double x, double y,
                        double pose_confidence,
                        double color_confidence);

    // 创建 cone 消息
    static fsd_common_msgs::msg::Cone make_cone(const std::string &color, const Landmark &lm);

    // 发布全局地图
    void publish_map();

    // 发布 RViz 可视化
    void publish_markers();

    // 参数
    double merge_distance_;
    double marker_scale_x_;
    double marker_scale_y_;
    double marker_scale_z_;

    // 状态
    double pose_x_ = 0.0;
    double pose_y_ = 0.0;
    double pose_yaw_ = 0.0;
    bool has_pose_ = false;
    std::map<std::string, std::vector<Landmark>> landmarks_;

    // 发布器
    rclcpp::Publisher<fsd_common_msgs::msg::Map>::SharedPtr map_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    // 订阅器
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<fsd_common_msgs::msg::Map>::SharedPtr map_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};