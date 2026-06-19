#include "pure_pursuit_controller.hpp"
#include <cmath>
#include <limits>
#include "right_angle_stack_cpp/utils.hpp"

PurePursuitController::PurePursuitController()
    : Node("pure_pursuit_controller") {
    // 车辆参数
    this->declare_parameter("wheelbase", 1.04); // 与right_angle_car.urdf.xacro的轴距一致
    this->declare_parameter("target_speed", 3.0);
    this->declare_parameter("max_steer_angle", 0.8);
    wheelbase_ = this->get_parameter("wheelbase").as_double();
    target_speed_ = this->get_parameter("target_speed").as_double();
    max_steer_angle_ = this->get_parameter("max_steer_angle").as_double();

    // 策略参数
    this->declare_parameter("stop_distance", 1.2);
    this->declare_parameter("lookahead_distance", 3.0);
    stop_distance_ = this->get_parameter("stop_distance").as_double();
    lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();

    // 订阅话题：与right_angle_planner的节点约定一致
    this->declare_parameter("path_topic", "/planning/centerline");
    this->declare_parameter("odom_topic", "/localization/odom");
    path_sub_ = bind_sub(this, "path_topic", 10, &PurePursuitController::on_path);
    odom_sub_ = bind_sub(this, "odom_topic", 20, &PurePursuitController::on_odom);

    // 发布话题：控制gazebo中的车辆指令
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(50),
                                     [this]() { on_timer(); });

    RCLCPP_INFO(this->get_logger(), "车辆控制器启动");
}

void PurePursuitController::on_path(const nav_msgs::msg::Path::SharedPtr &msg) {
    path_.clear();
    for (const auto &pose: msg->poses)
        path_.emplace_back(pose.pose.position.x,
                           pose.pose.position.y);
    if (has_state_ && !path_.empty()) {
        current_index_ = 0;
        double best = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < path_.size(); ++i) {
            const double d = std::hypot(path_[i].first - state_x_,
                                        path_[i].second - state_y_);
            if (d < best) {
                best = d;
                current_index_ = i;
            }
        }
    }
}

void PurePursuitController::on_odom(const nav_msgs::msg::Odometry::SharedPtr &msg) {
    state_x_ = msg->pose.pose.position.x;
    state_y_ = msg->pose.pose.position.y;
    state_yaw_ = quaternion_to_yaw(msg->pose.pose.orientation);
    has_state_ = true;
}

void PurePursuitController::publish_stop() const {
    const geometry_msgs::msg::Twist cmd;
    cmd_pub_->publish(cmd);
}

void PurePursuitController::on_timer() {
    if (!has_state_ || path_.size() < 2) {
        publish_stop();
        return;
    }

    const double x = state_x_;
    const double y = state_y_;
    const double yaw = state_yaw_;

    const auto &[goal_x, goal_y] = path_.back();
    if (std::hypot(goal_x - x, goal_y - y) < stop_distance_) {
        publish_stop();
        return;
    }

    if (current_index_ >= path_.size()) current_index_ = 0;
    size_t nearest_idx = current_index_;
    double nearest_dist = std::numeric_limits<double>::infinity();
    for (size_t i = current_index_; i < path_.size(); ++i) {
        const double dist = std::hypot(path_[i].first - x,
                                       path_[i].second - y);
        if (dist < nearest_dist) {
            nearest_dist = dist;
            nearest_idx = i;
        }
    }
    current_index_ = nearest_idx;

    size_t target_idx = nearest_idx;
    double accumulated = 0.0;
    for (size_t i = nearest_idx; i + 1 < path_.size(); ++i) {
        accumulated += std::hypot(path_[i + 1].first - path_[i].first,
                                  path_[i + 1].second - path_[i].second);
        if (accumulated >= lookahead_distance_) {
            target_idx = i + 1;
            break;
        }
    }
    if (target_idx == nearest_idx && nearest_idx < path_.size() - 1) {
        target_idx = path_.size() - 1;
    }

    const double tx = path_[target_idx].first;
    const double ty = path_[target_idx].second;

    const double dx = tx - x;
    const double dy = ty - y;
    const double e = dy * std::cos(yaw) - dx * std::sin(yaw);

    double sigma = std::atan(2.0 * wheelbase_ * e / (lookahead_distance_ * lookahead_distance_));
    sigma = std::max(-max_steer_angle_, std::min(max_steer_angle_, sigma));

    const double yaw_rate = target_speed_ * std::tan(sigma) / wheelbase_;

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = target_speed_;
    cmd.angular.z = yaw_rate;
    cmd_pub_->publish(cmd);
}
