// 定位融合节点头文件：GPS + IMU陀螺 + 轮速计 + 磁力计 → 融合位姿
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tf2_ros/transform_broadcaster.h>

static constexpr double EARTH_RADIUS_M = 6378137.0;

class LocalizationFusion : public rclcpp::Node {
public:
    explicit LocalizationFusion(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
    // GPS 经纬度转本地 ENU 坐标
    void gps_to_local_xy(double latitude_deg, double longitude_deg, double &x, double &y) const;

    // GPS 回调：低通滤波融合位置
    void on_gps(sensor_msgs::msg::NavSatFix::SharedPtr msg);

    // IMU 回调：记录角速度
    void on_imu(sensor_msgs::msg::Imu::SharedPtr msg);

    // 轮速计回调：记录前进速度
    void on_wheel_odom(nav_msgs::msg::Odometry::SharedPtr msg);

    // 磁力计回调：修正航向角
    void on_magnetic_field(sensor_msgs::msg::MagneticField::SharedPtr msg);

    // 定时器回调：发布位姿
    void on_timer();

    // 发布位姿、里程计和 TF 变换
    void publish_state(rclcpp::Time stamp) const;

    // 模板辅助：从话题参数创建订阅
    template<typename MsgT>
    std::shared_ptr<rclcpp::Subscription<MsgT> >
    bind_sub(const std::string &topic_param, int qos,
             void (LocalizationFusion::*handler)(std::shared_ptr<MsgT>));

    // 参数
    double gps_gain_, mag_gain_, magnetic_declination_;
    bool use_first_gps_as_origin_;
    double gps_reject_distance_, mag_reject_angle_;
    double default_dt_, max_dt_;
    double initial_x_;
    double initial_y_;
    double initial_yaw_;

    // 状态
    double x_, y_, yaw_;
    double forward_speed_ = 0.0;
    double yaw_rate_ = 0.0;
    double gps_ref_lat_ = 0.0, gps_ref_lon_ = 0.0;
    bool gps_reference_ready_ = false;
    int rejected_gps_count_ = 0;
    int rejected_mag_count_ = 0;
    rclcpp::Time last_time_;

    // 订阅器
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;

    // 发布器
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;
};