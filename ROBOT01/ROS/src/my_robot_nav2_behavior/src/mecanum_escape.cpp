#include "my_robot_nav2_behavior/mecanum_escape.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"

namespace my_robot_nav2_behavior
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr std::size_t kDirectionCount = 8;
}  // namespace

MecanumEscape::MecanumEscape()
: nav2_behaviors::TimedBehavior<Action>(),
  feedback_(std::make_shared<Action::Feedback>())
{
}

template<typename T>
void MecanumEscape::declareAndGet(
    const std::string & name, const T & default_value, T & value)
{
    auto node = node_.lock();
    if (!node) {
        throw std::runtime_error{"Failed to lock behavior server node"};
    }

    const std::string parameter_name = behavior_name_ + "." + name;
    nav2_util::declare_parameter_if_not_declared(
        node, parameter_name, rclcpp::ParameterValue(default_value));
    node->get_parameter(parameter_name, value);
}

void MecanumEscape::onConfigure()
{
    auto node = node_.lock();
    if (!node) {
        throw std::runtime_error{"Failed to lock behavior server node"};
    }

    declareAndGet("odom_topic", std::string("odom"), odom_topic_);
    declareAndGet("projection_step", 0.05, projection_step_);
    declareAndGet("search_distance", 0.60, search_distance_);
    declareAndGet("clearance_margin", 0.05, clearance_margin_);
    declareAndGet("simulate_ahead_time", 2.0, simulate_ahead_time_);
    declareAndGet("min_remembered_speed", 0.03, min_remembered_speed_);
    declareAndGet("velocity_memory_duration", 30.0, velocity_memory_duration_);
    declareAndGet("direction_preference_weight", 0.35, direction_preference_weight_);
    declareAndGet("clearance_weight", 0.65, clearance_weight_);

    if (projection_step_ <= 0.0 || search_distance_ <= 0.0 ||
        clearance_margin_ < 0.0 || simulate_ahead_time_ <= 0.0 ||
        min_remembered_speed_ < 0.0 || velocity_memory_duration_ < 0.0)
    {
        throw std::runtime_error{"MecanumEscape parameters must be positive"};
    }

    odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::SystemDefaultsQoS(),
        std::bind(&MecanumEscape::odomCallback, this, std::placeholders::_1));
}

void MecanumEscape::onCleanup()
{
    behavior_running_ = false;
    odom_sub_.reset();
}

void MecanumEscape::onActionCompletion(std::shared_ptr<Result> /*result*/)
{
    behavior_running_ = false;
}

void MecanumEscape::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Do not learn the recovery command itself as the direction that caused the robot to get stuck.
    if (behavior_running_) {
        return;
    }

    const double vx = msg->twist.twist.linear.x;
    const double vy = msg->twist.twist.linear.y;
    if (std::hypot(vx, vy) < min_remembered_speed_) {
        return;
    }

    std::lock_guard<std::mutex> lock(velocity_mutex_);
    last_velocity_x_ = vx;
    last_velocity_y_ = vy;
    last_velocity_time_ = clock_->now();
}

std::pair<double, double> MecanumEscape::preferredDirection()
{
    std::lock_guard<std::mutex> lock(velocity_mutex_);
    const double speed = std::hypot(last_velocity_x_, last_velocity_y_);
    const double age = (clock_->now() - last_velocity_time_).seconds();
    if (speed >= min_remembered_speed_ && age <= velocity_memory_duration_) {
        return {-last_velocity_x_ / speed, -last_velocity_y_ / speed};
    }

    RCLCPP_WARN(
        logger_,
        "No recent non-zero odometry velocity; using rearward as the preferred escape direction");
    return {-1.0, 0.0};
}

geometry_msgs::msg::Pose2D MecanumEscape::projectPose(
    const geometry_msgs::msg::PoseStamped & pose,
    const double direction_x, const double direction_y, const double distance) const
{
    geometry_msgs::msg::Pose2D projected;
    const double yaw = tf2::getYaw(pose.pose.orientation);
    const double world_x = std::cos(yaw) * direction_x - std::sin(yaw) * direction_y;
    const double world_y = std::sin(yaw) * direction_x + std::cos(yaw) * direction_y;
    projected.x = pose.pose.position.x + distance * world_x;
    projected.y = pose.pose.position.y + distance * world_y;
    projected.theta = yaw;
    return projected;
}

double MecanumEscape::measureClearance(
    const geometry_msgs::msg::PoseStamped & pose,
    const double direction_x, const double direction_y, const double max_distance)
{
    bool fetch_data = true;
    double last_clear_distance = 0.0;
    for (double distance = projection_step_; distance <= max_distance;
        distance += projection_step_)
    {
        const auto projected = projectPose(pose, direction_x, direction_y, distance);
        if (!local_collision_checker_->isCollisionFree(projected, fetch_data)) {
            break;
        }
        fetch_data = false;
        last_clear_distance = distance;
    }
    return last_clear_distance;
}

bool MecanumEscape::selectDirection(const geometry_msgs::msg::PoseStamped & pose)
{
    const auto preferred = preferredDirection();
    const double preferred_angle = std::atan2(preferred.second, preferred.first);
    const double required_clearance = command_distance_ + clearance_margin_;
    const double max_search = std::max(search_distance_, required_clearance);

    DirectionScore best{0.0, 0.0, 0.0, 0.0, -std::numeric_limits<double>::infinity()};
    bool found = false;

    for (std::size_t index = 0; index < kDirectionCount; ++index) {
        const double angle = static_cast<double>(index) * kPi / 4.0;
        const double direction_x = std::cos(angle);
        const double direction_y = std::sin(angle);
        const double clearance = measureClearance(
            pose, direction_x, direction_y, max_search);
        if (clearance + 1e-6 < required_clearance) {
            continue;
        }

        const double preference = 0.5 * (1.0 + std::cos(angle - preferred_angle));
        const double normalized_clearance = std::min(clearance / max_search, 1.0);
        const double score = direction_preference_weight_ * preference +
            clearance_weight_ * normalized_clearance;
        if (!found || score > best.score) {
            best = {direction_x, direction_y, angle, clearance, score};
            found = true;
        }
    }

    if (!found) {
        RCLCPP_WARN(
            logger_, "No escape direction has %.2f m of required clearance",
            required_clearance);
        return false;
    }

    direction_x_ = best.x;
    direction_y_ = best.y;
    RCLCPP_INFO(
        logger_,
        "Selected mecanum escape direction %.0f deg (vx %.2f, vy %.2f), clearance %.2f m",
        best.angle * 180.0 / kPi, direction_x_ * command_speed_,
        direction_y_ * command_speed_, best.clearance);
    return true;
}

nav2_behaviors::ResultStatus MecanumEscape::onRun(
    const std::shared_ptr<const Goal> command)
{
    command_distance_ = std::fabs(command->target.x);
    command_speed_ = std::fabs(command->speed);
    command_time_allowance_ = command->time_allowance;

    if (command_distance_ <= 0.0 || command_speed_ <= 0.0) {
        RCLCPP_ERROR(logger_, "Escape distance and speed must be non-zero");
        return {nav2_behaviors::Status::FAILED, Result::INVALID_INPUT};
    }

    if (!nav2_util::getCurrentPose(
        initial_pose_, *tf_, local_frame_, robot_base_frame_, transform_tolerance_))
    {
        RCLCPP_ERROR(logger_, "Initial robot pose is not available");
        return {nav2_behaviors::Status::FAILED, Result::TF_ERROR};
    }

    if (!selectDirection(initial_pose_)) {
        return {nav2_behaviors::Status::FAILED, Result::COLLISION_AHEAD};
    }

    end_time_ = clock_->now() + command_time_allowance_;
    behavior_running_ = true;
    return {nav2_behaviors::Status::SUCCEEDED, Result::NONE};
}

bool MecanumEscape::trajectoryIsClear(
    const geometry_msgs::msg::PoseStamped & pose, const double distance)
{
    bool fetch_data = true;
    for (double projection = 0.0; projection <= distance; projection += projection_step_) {
        const auto projected = projectPose(pose, direction_x_, direction_y_, projection);
        if (!local_collision_checker_->isCollisionFree(projected, fetch_data)) {
            return false;
        }
        fetch_data = false;
    }
    return true;
}

nav2_behaviors::ResultStatus MecanumEscape::onCycleUpdate()
{
    const auto time_remaining = end_time_ - clock_->now();
    if (time_remaining.seconds() < 0.0 && command_time_allowance_.seconds() > 0.0) {
        stopRobot();
        RCLCPP_WARN(logger_, "Mecanum escape exceeded its time allowance");
        return {nav2_behaviors::Status::FAILED, Result::TIMEOUT};
    }

    geometry_msgs::msg::PoseStamped current_pose;
    if (!nav2_util::getCurrentPose(
        current_pose, *tf_, local_frame_, robot_base_frame_, transform_tolerance_))
    {
        stopRobot();
        RCLCPP_ERROR(logger_, "Current robot pose is not available");
        return {nav2_behaviors::Status::FAILED, Result::TF_ERROR};
    }

    const double distance_traveled = std::hypot(
        current_pose.pose.position.x - initial_pose_.pose.position.x,
        current_pose.pose.position.y - initial_pose_.pose.position.y);
    feedback_->distance_traveled = distance_traveled;
    action_server_->publish_feedback(feedback_);

    if (distance_traveled >= command_distance_) {
        stopRobot();
        return {nav2_behaviors::Status::SUCCEEDED, Result::NONE};
    }

    const double remaining_distance = command_distance_ - distance_traveled;
    const double lookahead = std::min(
        remaining_distance, std::max(projection_step_, command_speed_ * simulate_ahead_time_));
    if (!trajectoryIsClear(current_pose, lookahead)) {
        stopRobot();
        RCLCPP_WARN(logger_, "Collision detected along selected mecanum escape direction");
        return {nav2_behaviors::Status::FAILED, Result::COLLISION_AHEAD};
    }

    auto velocity = std::make_unique<geometry_msgs::msg::TwistStamped>();
    velocity->header.stamp = clock_->now();
    velocity->header.frame_id = robot_base_frame_;
    velocity->twist.linear.x = command_speed_ * direction_x_;
    velocity->twist.linear.y = command_speed_ * direction_y_;
    velocity->twist.angular.z = 0.0;
    vel_pub_->publish(std::move(velocity));

    return {nav2_behaviors::Status::RUNNING, Result::NONE};
}

nav2_core::CostmapInfoType MecanumEscape::getResourceInfo()
{
    return nav2_core::CostmapInfoType::LOCAL;
}

}  // namespace my_robot_nav2_behavior

PLUGINLIB_EXPORT_CLASS(my_robot_nav2_behavior::MecanumEscape, nav2_core::Behavior)
