#pragma once

#include <cmath>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <visualization_msgs/msg/marker.hpp>

// 角度归一化到 [-π, π]
inline double normalize_angle(double angle) {
    return std::fmod(angle + M_PI, 2.0 * M_PI) - M_PI;
}

// 偏航角转四元数（仅绕Z轴旋转）
inline geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw) {
    geometry_msgs::msg::Quaternion q;
    q.w = std::cos(yaw * 0.5);
    q.z = std::sin(yaw * 0.5);
    return q;
}

// 四元数转偏航角
inline double quaternion_to_yaw(const geometry_msgs::msg::Quaternion &q) {
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

// 世界坐标系下的向量转换到车体坐标系
inline void world_to_body(double dx, double dy, double yaw, double &local_x, double &local_y) {
    double c = std::cos(yaw);
    double s = std::sin(yaw);
    local_x = c * dx + s * dy;
    local_y = -s * dx + c * dy;
}

// 车体坐标系下的向量转换到世界坐标系
inline void body_to_world(const double local_x,
                          const double local_y,
                          const double origin_x,
                          const double origin_y,
                          const double yaw,
                          double &wx, double &wy) {
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    wx = origin_x + c * local_x - s * local_y;
    wy = origin_y + s * local_x + c * local_y;
}


// 根据颜色设置 Marker 的 RGBA 颜色
inline void set_marker_color(visualization_msgs::msg::Marker &marker, const std::string &color) {
    marker.color.a = 1.0;
    if (color == "blue") {
        marker.color.r = 0.02;
        marker.color.g = 0.16;
        marker.color.b = 1.0;
    } else if (color == "yellow") {
        marker.color.r = 1.0;
        marker.color.g = 0.78;
        marker.color.b = 0.02;
    } else if (color == "red") {
        marker.color.r = 1.0;
        marker.color.g = 0.04;
        marker.color.b = 0.02;
    } else {
        marker.color.r = 0.8;
        marker.color.g = 0.8;
        marker.color.b = 0.8;
    }
}

// 订阅模板（const 引用版本，避免不必要的 shared_ptr 拷贝）
template<typename NodeT, typename MsgT>
std::shared_ptr<rclcpp::Subscription<MsgT> >
bind_sub(NodeT *node, const std::string &param_name, int qos,
         void (NodeT::*handler)(const std::shared_ptr<MsgT> &)) {
    const std::string topic = node->get_parameter(param_name).as_string();
    auto callback = [node, handler](std::shared_ptr<MsgT> msg) { (node->*handler)(std::move(msg)); };
    return node->template create_subscription<MsgT>(topic, qos, callback);
}

// 订阅模板（值传递版本）
template<typename NodeT, typename MsgT>
std::shared_ptr<rclcpp::Subscription<MsgT> >
bind_sub(NodeT *node, const std::string &param_name, int qos,
         void (NodeT::*handler)(std::shared_ptr<MsgT>)) {
    const std::string topic = node->get_parameter(param_name).as_string();
    auto callback = [node, handler](std::shared_ptr<MsgT> msg) { (node->*handler)(std::move(msg)); };
    return node->template create_subscription<MsgT>(topic, qos, callback);
}