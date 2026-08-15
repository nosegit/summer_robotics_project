#ifndef MY_ROBOT_NAV2_BEHAVIOR__MECANUM_ESCAPE_HPP_
#define MY_ROBOT_NAV2_BEHAVIOR__MECANUM_ESCAPE_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace my_robot_nav2_behavior
{

class MecanumEscape : public nav2_behaviors::TimedBehavior<nav2_msgs::action::BackUp>
{
public:
    using Action = nav2_msgs::action::BackUp;
    using Goal = Action::Goal;
    using Result = Action::Result;

    MecanumEscape();
    ~MecanumEscape() override = default;

    nav2_behaviors::ResultStatus onRun(
        const std::shared_ptr<const Goal> command) override;
    nav2_behaviors::ResultStatus onCycleUpdate() override;
    nav2_core::CostmapInfoType getResourceInfo() override;

protected:
    void onConfigure() override;
    void onCleanup() override;
    void onActionCompletion(std::shared_ptr<Result> result) override;

private:
    struct DirectionScore
    {
        double x;
        double y;
        double angle;
        double clearance;
        double score;
    };

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    std::pair<double, double> preferredDirection();
    bool selectDirection(const geometry_msgs::msg::PoseStamped & pose);
    double measureClearance(
        const geometry_msgs::msg::PoseStamped & pose,
        double direction_x, double direction_y, double max_distance);
    bool trajectoryIsClear(
        const geometry_msgs::msg::PoseStamped & pose, double distance);
    geometry_msgs::msg::Pose2D projectPose(
        const geometry_msgs::msg::PoseStamped & pose,
        double direction_x, double direction_y, double distance) const;

    template<typename T>
    void declareAndGet(const std::string & name, const T & default_value, T & value);

    Action::Feedback::SharedPtr feedback_;
    geometry_msgs::msg::PoseStamped initial_pose_;
    rclcpp::Duration command_time_allowance_{0, 0};
    rclcpp::Time end_time_;
    double command_distance_{0.0};
    double command_speed_{0.0};
    double direction_x_{-1.0};
    double direction_y_{0.0};

    std::string odom_topic_{"odom"};
    double projection_step_{0.05};
    double search_distance_{0.60};
    double clearance_margin_{0.05};
    double simulate_ahead_time_{2.0};
    double min_remembered_speed_{0.03};
    double velocity_memory_duration_{30.0};
    double direction_preference_weight_{0.35};
    double clearance_weight_{0.65};

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    std::atomic_bool behavior_running_{false};
    std::mutex velocity_mutex_;
    double last_velocity_x_{0.0};
    double last_velocity_y_{0.0};
    rclcpp::Time last_velocity_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace my_robot_nav2_behavior

#endif  // MY_ROBOT_NAV2_BEHAVIOR__MECANUM_ESCAPE_HPP_
