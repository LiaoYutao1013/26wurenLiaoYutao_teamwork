#include "cone_mapper.hpp"
#include <cmath>
#include <limits>
#include <utility>
#include "right_angle_stack_cpp/utils.hpp"

ConeMapper::ConeMapper()
    : Node("cone_mapper") {
    // 订阅感知话题
    this->declare_parameter("perception_map_topic", "/perception/cones");
    map_sub_ = bind_sub(this, "perception_map_topic", 10, &ConeMapper::on_map_detection);

    // 订阅位姿话题
    this->declare_parameter("pose_topic", "/localization/pose");
    pose_sub_ = bind_sub(this, "pose_topic", 10, &ConeMapper::on_pose);

    // 锥桶合并距离阈值
    this->declare_parameter("merge_distance", 0.75);
    merge_distance_ = this->get_parameter("merge_distance").as_double();

    // 可视化锥桶参数
    this->declare_parameter("marker_scale_x", 0.35);
    this->declare_parameter("marker_scale_y", 0.35);
    this->declare_parameter("marker_scale_z", 0.56);
    marker_scale_x_ = this->get_parameter("marker_scale_x").as_double();
    marker_scale_y_ = this->get_parameter("marker_scale_y").as_double();
    marker_scale_z_ = this->get_parameter("marker_scale_z").as_double();

    // 地图发布器（规划）
    map_pub_ = this->create_publisher<fsd_common_msgs::msg::Map>("/estimation/slam/map", 10);
    // 可视化锥桶发布器（rviz）
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/visualization/cone_map", 10);

    // 定时器：发布地图可视化
    timer_ = this->create_wall_timer(std::chrono::milliseconds(200),
                                     [this]() {
                                         publish_map();
                                         publish_markers();
                                     });

    RCLCPP_INFO(this->get_logger(), "锥桶建图节点启动：将局部感知结果转换到世界坐标系");
}

// 位姿回调：记录车辆当前位置
void ConeMapper::on_pose(const geometry_msgs::msg::PoseStamped::SharedPtr &msg) {
    pose_x_ = msg->pose.position.x;
    pose_y_ = msg->pose.position.y;
    pose_yaw_ = quaternion_to_yaw(msg->pose.orientation);
    has_pose_ = true;
}

// 感知回调（Map 格式）
void ConeMapper::on_map_detection(const fsd_common_msgs::msg::Map::SharedPtr &msg) {
    process_cones(msg->header.frame_id, msg->cone_red, "red");
    process_cones(msg->header.frame_id, msg->cone_blue, "blue");
    process_cones(msg->header.frame_id, msg->cone_yellow, "yellow");
    process_cones(msg->header.frame_id, msg->cone_unknown, "unknown");
    publish_map();
    publish_markers();
}

// 处理一组锥桶：坐标变换 → 合并
void ConeMapper::process_cones(const std::string &frame_id,
                               const std::vector<fsd_common_msgs::msg::Cone> &cones,
                               const std::string &fallback_color) {
    for (const auto &cone: cones) {
        std::string color = cone.color.empty() ? fallback_color : cone.color;
        double wx, wy;
        // 跳过无法转换的锥桶
        if (!to_world(frame_id, cone, wx, wy)) continue;
        // 合并或新增路标
        merge_landmark(color, wx, wy, cone.pose_confidence, cone.color_confidence);
    }
}

// 将锥桶坐标转换到世界坐标系
bool ConeMapper::to_world(const std::string &frame_id,
                          const fsd_common_msgs::msg::Cone &cone,
                          double &wx, double &wy) const {
    // 世界坐标系无需转换
    if (frame_id == "world") {
        wx = cone.position.x;
        wy = cone.position.y;
        return true;
    }
    // 无位姿信息，无法转换
    if (!has_pose_) return false;
    // 从局部坐标系转换到世界坐标系
    body_to_world(cone.position.x, cone.position.y, pose_x_, pose_y_, pose_yaw_, wx, wy);
    return true;
}

// 合并路标：最近邻匹配，距离小于阈值则加权平均更新，否则新增
void ConeMapper::merge_landmark(const std::string &color,
                                const double x, const double y,
                                const double pose_confidence,
                                const double color_confidence) {
    auto &bucket = landmarks_[color];
    Landmark *closest = nullptr;
    double best_distance = std::numeric_limits<double>::infinity();

    // 遍历所有路标，寻找最近邻匹配项
    for (auto &lm: bucket) {
        if (const double dist = std::hypot(lm.x - x, lm.y - y); dist < best_distance) {
            best_distance = dist;
            closest = &lm;
        }
    }

    // 无最近邻或距离超过阈值，合并新增锥桶
    if (closest == nullptr || best_distance > merge_distance_) {
        bucket.push_back({x, y, 1, pose_confidence, color_confidence});
        return;
    }

    // 加权平均更新已有路标
    const int count = std::min(closest->count + 1, 30);
    const double alpha = 1.0 / count;
    closest->x = (1.0 - alpha) * closest->x + alpha * x;
    closest->y = (1.0 - alpha) * closest->y + alpha * y;
    closest->count = count;
    closest->pose_confidence = std::max(closest->pose_confidence, pose_confidence);
    closest->color_confidence = std::max(closest->color_confidence, color_confidence);
}

// 创建 cone 消息
fsd_common_msgs::msg::Cone ConeMapper::make_cone(const std::string &color, const Landmark &lm) {
    fsd_common_msgs::msg::Cone cone;
    cone.position.x = lm.x;
    cone.position.y = lm.y;
    cone.position.z = 0.0;
    cone.color = color;
    cone.pose_confidence = lm.pose_confidence;
    cone.color_confidence = lm.color_confidence;
    return cone;
}

// 发布全局地图
void ConeMapper::publish_map() {
    fsd_common_msgs::msg::Map msg;
    // 构建全局地图：遍历所有颜色的锥桶，按颜色归类填充
    for (const auto &[color, bucket]: landmarks_) {
        for (const auto &lm: bucket) {
            // clang-format off
            if      (color == "red")    msg.cone_red    .push_back(make_cone("red", lm));
            else if (color == "blue")   msg.cone_blue   .push_back(make_cone("blue", lm));
            else if (color == "yellow") msg.cone_yellow .push_back(make_cone("yellow", lm));
            else                        msg.cone_unknown.push_back(make_cone("unknown", lm));
            // clang-format on
        }
    }
    map_pub_->publish(msg);
}

// 发布 RViz 可视化
void ConeMapper::publish_markers() {
    const auto stamp = this->now();
    visualization_msgs::msg::MarkerArray markers;

    visualization_msgs::msg::Marker clear;
    clear.header.stamp = stamp;
    clear.header.frame_id = "world";
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);

    int marker_id = 0;
    for (const auto &[color, landmarks]: landmarks_) {
        for (const auto &landmark: landmarks) {
            visualization_msgs::msg::Marker marker;
            marker.header.stamp = stamp;
            marker.header.frame_id = "world";
            marker.ns = color + "_cones";
            marker.id = marker_id++;
            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = landmark.x;
            marker.pose.position.y = landmark.y;
            marker.pose.position.z = marker_scale_z_ / 2.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = marker_scale_x_;
            marker.scale.y = marker_scale_y_;
            marker.scale.z = marker_scale_z_;
            set_marker_color(marker, color);
            markers.markers.push_back(marker);
        }
    }
    marker_pub_->publish(markers);
}
