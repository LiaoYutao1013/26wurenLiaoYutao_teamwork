#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include <vector>

class PurePursuitController : public rclcpp::Node {
public:
    PurePursuitController();

private:
    // 路径订阅回调函数
    void on_path(const nav_msgs::msg::Path::SharedPtr &msg);

    // 里程计订阅回调函数
    void on_odom(const nav_msgs::msg::Odometry::SharedPtr &msg);

    // 定时器回调函数：定时发布控制
    void on_timer() const;

    // 发布停止指令
    void publish_stop() const;

    // 车辆参数
    double wheelbase_;
    double target_speed_;
    double max_steer_angle_;

    // 车辆位姿状态
    bool has_state_ = false;
    double state_x_ = 0.0;
    double state_y_ = 0.0;
    double state_yaw_ = 0.0;

    // 前向预测距离
    double lookahead_distance_;

    // 路径点
    std::vector<std::pair<double, double> > path_;

    // 订阅器、发布器、定时器
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
