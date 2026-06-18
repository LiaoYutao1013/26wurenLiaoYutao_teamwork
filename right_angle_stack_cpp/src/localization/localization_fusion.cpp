#include "localization_fusion.hpp"
#include <cmath>
#include "right_angle_stack_cpp/utils.hpp"

LocalizationFusion::LocalizationFusion(const rclcpp::NodeOptions &options)
    : Node("localization_fusion", options) {
    // 初始位姿：(0, -15, 朝北)
    this->declare_parameter("initial_x", 0.0);
    this->declare_parameter("initial_y", -15.0);
    this->declare_parameter("initial_yaw", 0.5 * M_PI);
    initial_x_ = this->get_parameter("initial_x").as_double();
    initial_y_ = this->get_parameter("initial_y").as_double();
    initial_yaw_ = this->get_parameter("initial_yaw").as_double();

    // 定位融合参数
    this->declare_parameter("gps_gain", 0.12); // GPS
    this->declare_parameter("mag_gain", 0.0); // 磁力计
    this->declare_parameter("magnetic_declination", 0.0); // 磁偏角
    gps_gain_ = this->get_parameter("gps_gain").as_double();
    mag_gain_ = this->get_parameter("mag_gain").as_double();
    magnetic_declination_ = this->get_parameter("magnetic_declination").as_double();

    // 是否使用第一个GPS数据作为参考点
    this->declare_parameter("use_first_gps_as_origin", true);
    use_first_gps_as_origin_ = this->get_parameter("use_first_gps_as_origin").as_bool();

    // 跳变拒绝阈值
    this->declare_parameter("mag_reject_angle", 1.2); // 磁力计
    this->declare_parameter("gps_reject_distance", 8.0); // GPS
    mag_reject_angle_ = this->get_parameter("mag_reject_angle").as_double();
    gps_reject_distance_ = this->get_parameter("gps_reject_distance").as_double();

    // 订阅传感器话题：与model.sdf中的传感器话题一致
    this->declare_parameter("gps_topic", "/sensors/gps/fix"); // GPS
    this->declare_parameter("imu_topic", "/sensors/imu/data_raw"); // IMU
    this->declare_parameter("wheel_odom_topic", "/sensors/wheel_odom"); // 轮速计
    this->declare_parameter("magnetic_field_topic", "/sensors/magnetic_field"); // 磁力计
    gps_sub_ = bind_sub(this, "gps_topic", 10, &LocalizationFusion::on_gps);
    imu_sub_ = bind_sub(this, "imu_topic", 50, &LocalizationFusion::on_imu);
    wheel_sub_ = bind_sub(this, "wheel_odom_topic", 20, &LocalizationFusion::on_wheel_odom);
    mag_sub_ = bind_sub(this, "magnetic_field_topic", 20, &LocalizationFusion::on_magnetic_field);

    // 初始化状态
    x_ = initial_x_;
    y_ = initial_y_;
    yaw_ = initial_yaw_;
    gps_reference_ready_ = !use_first_gps_as_origin_; // 如果不使用第一个GPS数据作为参考点，默认已准备就绪

    // 位姿发布器
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/localization/pose", 10);
    // 里程计发布器
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/localization/odom", 10);
    // TF 广播器
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    // 定时器：每 20ms 发布位姿
    timer_ = this->create_wall_timer(std::chrono::milliseconds(20),
                                     [this]() { on_timer(); });
    last_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "定位融合节点启动：GPS，轮速计，IMU，磁力计");
}

// GPS 经纬度转本地 ENU 坐标
void LocalizationFusion::gps_to_local_xy(const double latitude_deg,
                                         const double longitude_deg,
                                         double &x,
                                         double &y) const {
    const double lat = latitude_deg * M_PI / 180.0;
    const double lon = longitude_deg * M_PI / 180.0;
    x = initial_x_ + EARTH_RADIUS_M * std::cos(gps_ref_lat_) * (lon - gps_ref_lon_);
    y = initial_y_ + EARTH_RADIUS_M * (lat - gps_ref_lat_);
}

// GPS 回调：低通滤波融合位置，带跳变拒绝
void LocalizationFusion::on_gps(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    // 跳过无效数据
    if (std::isnan(msg->latitude) || std::isnan(msg->longitude)) return;

    // 首次 GPS 作为参考原点
    if (!gps_reference_ready_) {
        gps_ref_lat_ = msg->latitude * M_PI / 180.0;
        gps_ref_lon_ = msg->longitude * M_PI / 180.0;
        gps_reference_ready_ = true;
        RCLCPP_INFO(this->get_logger(),
                    "GPS 参考点已从首次定位初始化；首次定位映射到初始位姿 (%.2f, %.2f)。",
                    initial_x_, initial_y_);
    }

    // 获取 ENU 坐标
    double gps_x, gps_y;
    gps_to_local_xy(msg->latitude, msg->longitude, gps_x, gps_y);

    // 跳变拒绝
    if (std::hypot(gps_x - x_, gps_y - y_) > gps_reject_distance_) return;

    // 低通滤波更新位置
    x_ = (1.0 - gps_gain_) * x_ + gps_gain_ * gps_x;
    y_ = (1.0 - gps_gain_) * y_ + gps_gain_ * gps_y;
}

// IMU 回调：记录角速度
void LocalizationFusion::on_imu(const sensor_msgs::msg::Imu::SharedPtr msg) {
    yaw_rate_ = msg->angular_velocity.z;
}

// 轮速计回调：记录前进速度
void LocalizationFusion::on_wheel_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    forward_speed_ = msg->twist.twist.linear.x;
}

// 磁力计回调：修正航向角
void LocalizationFusion::on_magnetic_field(const sensor_msgs::msg::MagneticField::SharedPtr msg) {
    // 磁力计未启用，直接返回
    if (mag_gain_ <= 0.0) return;

    const double mx = msg->magnetic_field.x;
    const double my = msg->magnetic_field.y;
    // 返回无效的磁力计数据
    if (std::abs(mx) + std::abs(my) < 1e-9) return;

    // 测量的偏航角
    const double measured_yaw = std::atan2(mx, my) - magnetic_declination_;
    const double yaw_error = normalize_angle(measured_yaw - yaw_);

    // 跳变拒绝
    if (std::abs(yaw_error) > mag_reject_angle_) return;
    // 低通滤波
    yaw_ += mag_gain_ * yaw_error;
    yaw_ = normalize_angle(yaw_);
}

// 定时器回调：发布位姿
void LocalizationFusion::on_timer() {
    // 时间计算
    const auto now = this->now();
    double dt = (now - last_time_).seconds();
    last_time_ = now;

    // 推算
    dt = std::max(dt, 0.0); // 确保时间步长非负
    yaw_ = normalize_angle(yaw_ + yaw_rate_ * dt);
    x_ += forward_speed_ * std::cos(yaw_) * dt;
    y_ += forward_speed_ * std::sin(yaw_) * dt;

    // 发布位姿、里程计和 TF 变换
    publish_state(now);
}

// 发布位姿、里程计和 TF 变换
void LocalizationFusion::publish_state(rclcpp::Time stamp) const {
    auto quat = yaw_to_quaternion(yaw_);

    std_msgs::msg::Header header;
    header.stamp = stamp;
    header.frame_id = "world";

    // 位姿发布
    geometry_msgs::msg::PoseStamped pose;
    pose.header = header;
    pose.pose.position.x = x_;
    pose.pose.position.y = y_;
    pose.pose.orientation = quat;
    pose_pub_->publish(pose);

    // 里程计发布
    nav_msgs::msg::Odometry odom;
    odom.header = header;
    odom.child_frame_id = "base_link";
    odom.pose.pose = pose.pose;
    odom.twist.twist.linear.x = forward_speed_;
    odom.twist.twist.angular.z = yaw_rate_;
    odom_pub_->publish(odom);

    // TF 变换发布
    geometry_msgs::msg::TransformStamped transform;
    transform.header = header;
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = x_;
    transform.transform.translation.y = y_;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation = quat;
    tf_broadcaster_->sendTransform(transform);
}