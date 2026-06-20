#include "pure_pursuit_controller.hpp"
#include <cmath>
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

    // 前视距离
    this->declare_parameter("lookahead_distance", 3.0);
    lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();

    // 订阅话题：与right_angle_planner的节点约定一致
    this->declare_parameter("path_topic", "/planning/centerline");
    this->declare_parameter("odom_topic", "/localization/odom");
    path_sub_ = bind_sub(this, "path_topic", 10, &PurePursuitController::on_path);
    odom_sub_ = bind_sub(this, "odom_topic", 20, &PurePursuitController::on_odom);

    // 发布话题：控制gazebo中的车辆指令
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // 定时器：发布控制指令
    timer_ = this->create_wall_timer(std::chrono::milliseconds(50),
                                     [this]() { on_timer(); });

    RCLCPP_INFO(this->get_logger(), "车辆控制器启动");
}

// 订阅规划话题回调：更新目标路径点
void PurePursuitController::on_path(const nav_msgs::msg::Path::SharedPtr &msg) {
    path_.clear();
    for (const auto &pose: msg->poses)
        path_.emplace_back(pose.pose.position.x,
                           pose.pose.position.y);
}


// 订阅定位话题回调：更新车辆位姿
void PurePursuitController::on_odom(const nav_msgs::msg::Odometry::SharedPtr &msg) {
    state_x_ = msg->pose.pose.position.x;
    state_y_ = msg->pose.pose.position.y;
    state_yaw_ = quaternion_to_yaw(msg->pose.pose.orientation);
    has_state_ = true;
}

// 发布停车指令
void PurePursuitController::publish_stop() const {
    const geometry_msgs::msg::Twist cmd;
    cmd_pub_->publish(cmd);
}

// 定时器回调：发布控制指令
void PurePursuitController::on_timer() const {
    // 没有定位信息、规划路径，停车
    if (!has_state_ || path_.empty()) {
        publish_stop();
        return;
    }

    // 选取下一个前视距离附近的点
    size_t target_idx = path_.size() - 1;
    for (size_t i = 0; i < path_.size(); ++i) {
        const double distance = std::hypot(state_x_ - path_[i].first,
                                           state_y_ - path_[i].second);
        if (distance >= lookahead_distance_) {
            target_idx = i;
            break;
        }
    }

    // 路径点跟踪
    const double tx = path_[target_idx].first;
    const double ty = path_[target_idx].second;

    const double dx = tx - state_x_;
    const double dy = ty - state_y_;
    const double e = dy * std::cos(state_yaw_) - dx * std::sin(state_yaw_);

    double sigma = std::atan(2.0 * wheelbase_ * e / (lookahead_distance_ * lookahead_distance_));
    sigma = std::max(-max_steer_angle_, sigma);
    sigma = std::min(+max_steer_angle_, sigma);

    const double yaw_rate = target_speed_ * std::tan(sigma) / wheelbase_;

    // 发布控制指令
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = target_speed_;
    cmd.angular.z = yaw_rate;
    cmd_pub_->publish(cmd);
}
