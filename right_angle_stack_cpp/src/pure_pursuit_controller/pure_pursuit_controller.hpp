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
    void on_path(const nav_msgs::msg::Path::SharedPtr &msg);
    void on_odom(const nav_msgs::msg::Odometry::SharedPtr &msg);
    void on_timer();
    void publish_stop() const;

    double wheelbase_;
    double target_speed_;
    double lookahead_distance_;
    double max_steer_angle_;
    double stop_distance_;

    std::vector<std::pair<double, double>> path_;
    size_t current_index_ = 0;
    double state_x_ = 0.0, state_y_ = 0.0, state_yaw_ = 0.0;
    bool has_state_ = false;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};