#include "right_angle_planner.hpp"
#include <cmath>
#include <limits>
#include "right_angle_stack_mix/utils.hpp"

RightAnglePlanner::RightAnglePlanner()
    : Node("right_angle_planner") {
    // 锥桶配对最大距离
    this->declare_parameter("pair_distance_max", 6.0);
    pair_distance_max_ = this->get_parameter("pair_distance_max").as_double();

    // 订阅器：地图、位姿
    this->declare_parameter("map_topic", "/estimation/slam/map");
    this->declare_parameter("pose_topic", "/localization/pose");
    map_sub_ = bind_sub(this, "map_topic", 10, &RightAnglePlanner::on_map);
    pose_sub_ = bind_sub(this, "pose_topic", 10, &RightAnglePlanner::on_pose);

    // 发布器：规划路径、可视化标记
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planning/centerline", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/visualization/planning", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                     [this]() { on_timer(); });

    RCLCPP_INFO(this->get_logger(), "路径规划模块启动");
}

// 位姿回调：更新车体位置
void RightAnglePlanner::on_pose(const geometry_msgs::msg::PoseStamped::SharedPtr &msg) {
    car_pose_ = msg;
}

// 地图回调：更新锥桶位置
void RightAnglePlanner::on_map(const fsd_common_msgs::msg::Map::SharedPtr &msg) {
    blue_cones_.clear();
    yellow_cones_.clear();
    for (const auto &cone: msg->cone_blue) blue_cones_.emplace_back(cone.position.x, cone.position.y);
    for (const auto &cone: msg->cone_yellow) yellow_cones_.emplace_back(cone.position.x, cone.position.y);
}

// 前方中心线规划
std::vector<std::pair<double, double> > RightAnglePlanner::cone_centerline() {
    if (!car_pose_) return {};

    std::vector<std::pair<double, double> > midpoints;

    // 对每个蓝色锥体，寻找最接近的黄色锥体，以这对锥桶的中点作为路径点
    std::vector<bool> used_yellow(yellow_cones_.size(), false);
    for (const auto &[blue_x, blue_y]: blue_cones_) {
        int best_index = -1;
        double best_dist = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < yellow_cones_.size(); ++i) {
            // 忽略已匹配的锥体
            if (used_yellow[i]) continue;
            // 计算最近锥桶对距离
            const double dist = std::hypot(blue_x - yellow_cones_[i].first,
                                           blue_y - yellow_cones_[i].second);
            if (dist < best_dist) {
                best_dist = dist;
                best_index = static_cast<int>(i);
            }
        }
        // 忽略相距太远的一对锥体
        if (best_dist > pair_distance_max_) continue;

        // 记录匹配，计算中点
        used_yellow[best_index] = true;
        midpoints.emplace_back((blue_x + yellow_cones_[best_index].first) * 0.5,
                               (blue_y + yellow_cones_[best_index].second) * 0.5);
    }

    // 过滤车体后方的路径点
    const double x = car_pose_->pose.position.x;
    const double y = car_pose_->pose.position.y;
    const double yaw = quaternion_to_yaw(car_pose_->pose.orientation);

    std::vector<std::pair<double, double> > filtered;
    for (const auto &mp: midpoints) {
        // 保留前方的路径点
        if ((mp.first - x) * std::cos(yaw) + (mp.second - y) * std::sin(yaw) > 0.0)
            filtered.push_back(mp);
    }

    // 按距离从近到远排序
    std::sort(filtered.begin(), filtered.end(),
              [&](const auto &a, const auto &b) {
                  const double da = std::hypot(a.first - x, a.second - y);
                  const double db = std::hypot(b.first - x, b.second - y);
                  return da < db;
              });

    return filtered;
}

// 定时器回调：发布路径、可视化标记
void RightAnglePlanner::on_timer() {
    const auto points = cone_centerline();

    const auto stamp = this->now();
    std_msgs::msg::Header header;
    header.stamp = stamp;
    header.frame_id = "world";
    publish_path(header, points);
    publish_markers(header, points);
}

// 发布规划路径
void RightAnglePlanner::publish_path(const std_msgs::msg::Header &header,
                                     const std::vector<std::pair<double, double> > &points) const {
    nav_msgs::msg::Path path;
    path.header = header;

    for (size_t i = 0; i < points.size(); ++i) {
        auto [x, y] = points[i];
        geometry_msgs::msg::PoseStamped pose;
        pose.header = header;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        path.poses.push_back(pose);
    }
    path_pub_->publish(path);
}

// 发布Rviz可视化标记
void RightAnglePlanner::publish_markers(const std_msgs::msg::Header &header,
                                        const std::vector<std::pair<double, double> > &points) const {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.header = header;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);

    visualization_msgs::msg::Marker line;
    line.header = header;
    line.ns = "centerline";
    line.id = 1;
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.scale.x = 0.12;
    line.color.r = 0.0;
    line.color.g = 0.9;
    line.color.b = 0.35;
    line.color.a = 1.0;

    for (const auto &[x, y]: points) {
        geometry_msgs::msg::Point p;
        p.x = x;
        p.y = y;
        p.z = 0.05;
        line.points.push_back(p);
    }
    markers.markers.push_back(line);
    marker_pub_->publish(markers);
}