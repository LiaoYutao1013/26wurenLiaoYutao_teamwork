#include "right_angle_planner.hpp"
#include <cmath>
#include <limits>
#include "right_angle_stack_cpp/utils.hpp"

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

void RightAnglePlanner::on_pose(const geometry_msgs::msg::PoseStamped::SharedPtr &msg) {
    car_pose_ = msg;
}

void RightAnglePlanner::on_map(const fsd_common_msgs::msg::Map::SharedPtr &msg) {
    blue_cones_.clear();
    yellow_cones_.clear();
    for (const auto &cone: msg->cone_blue)
        blue_cones_.push_back({next_cone_id_++, cone.position.x, cone.position.y, "blue"});
    for (const auto &cone: msg->cone_yellow)
        yellow_cones_.push_back({next_cone_id_++, cone.position.x, cone.position.y, "yellow"});
}

std::vector<std::pair<double, double> > RightAnglePlanner::cone_centerline() {
    if (!car_pose_) return {};

    std::vector<std::pair<double, double> > midpoints;

    // 对每个蓝色锥体，寻找最接近的黄色锥体，以这对锥桶的中点作为路径点
    std::vector<bool> used_yellow(yellow_cones_.size(), false);
    for (const auto &blue_cone: blue_cones_) {
        int best_index = -1;
        double best_dist = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < yellow_cones_.size(); ++i) {
            // 忽略已匹配的锥体
            if (used_yellow[i]) continue;
            // 计算最近锥桶对距离
            const double dist = std::hypot(blue_cone.x - yellow_cones_[i].x,
                                           blue_cone.y - yellow_cones_[i].y);
            if (dist < best_dist) {
                best_dist = dist;
                best_index = static_cast<int>(i);
            }
        }
        // 忽略相距太远的一对锥体
        if (best_dist > pair_distance_max_) continue;

        // 记录匹配，计算中点
        used_yellow[best_index] = true;
        midpoints.emplace_back((blue_cone.x + yellow_cones_[best_index].x) * 0.5,
                               (blue_cone.y + yellow_cones_[best_index].y) * 0.5);
    }

    // 过滤：只保留车体前方（投影>0）的点，避免U型弯道回头
    const double vx = car_pose_->pose.position.x;
    const double vy = car_pose_->pose.position.y;
    const double vyaw = quaternion_to_yaw(car_pose_->pose.orientation);
    const double dir_x = std::cos(vyaw);
    const double dir_y = std::sin(vyaw);

    std::vector<std::pair<double, double> > filtered;
    for (const auto &mp: midpoints) {
        const double proj = (mp.first - vx) * dir_x + (mp.second - vy) * dir_y;
        if (proj > 0.0) filtered.push_back(mp);
    }
    if (filtered.empty()) return midpoints;
    midpoints = std::move(filtered);

    // 按距离从近到远排序
    std::sort(midpoints.begin(), midpoints.end(),
              [&](const auto &a, const auto &b) {
                  const double da = std::hypot(a.first - vx, a.second - vy);
                  const double db = std::hypot(b.first - vx, b.second - vy);
                  return da < db;
              });

    return midpoints;
}

void RightAnglePlanner::on_timer() {
    const auto points = cone_centerline();
    if (points.empty()) return;

    const auto stamp = this->now();
    nav_msgs::msg::Path path;
    path.header.stamp = stamp;
    path.header.frame_id = "world";

    for (size_t i = 0; i < points.size(); ++i) {
        auto [x, y] = points[i];
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = x;
        pose.pose.position.y = y;

        double yaw = 0.0;
        if (i + 1 < points.size()) {
            yaw = std::atan2(points[i + 1].second - y, points[i + 1].first - x);
        } else if (i > 0) {
            yaw = std::atan2(y - points[i - 1].second, x - points[i - 1].first);
        }
        pose.pose.orientation = yaw_to_quaternion(yaw);
        path.poses.push_back(pose);
    }
    path_pub_->publish(path);
    publish_markers(path.header, points);
}

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
