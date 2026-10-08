#include "wheeltec_nav2/imu_spin.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"

namespace wheeltec_nav2
{

ImuSpin::ImuSpin()
: nav2_behaviors::TimedBehavior<nav2_msgs::action::Spin>()
{
  feedback_ = std::make_shared<nav2_msgs::action::Spin::Feedback>();
}

void ImuSpin::onConfigure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock behavior server node");
  }

  nav2_util::declare_parameter_if_not_declared(
    node, "imu_topic", rclcpp::ParameterValue("/imu/data"));
  nav2_util::declare_parameter_if_not_declared(
    node, "imu_min_angular_speed", rclcpp::ParameterValue(0.55));
  nav2_util::declare_parameter_if_not_declared(
    node, "imu_max_angular_speed", rclcpp::ParameterValue(0.60));
  nav2_util::declare_parameter_if_not_declared(
    node, "imu_proportional_gain", rclcpp::ParameterValue(1.8));
  nav2_util::declare_parameter_if_not_declared(
    node, "imu_slow_down_angle", rclcpp::ParameterValue(0.45));
  nav2_util::declare_parameter_if_not_declared(
    node, "imu_angle_tolerance", rclcpp::ParameterValue(0.045));
  nav2_util::declare_parameter_if_not_declared(
    node, "imu_timeout_sec", rclcpp::ParameterValue(0.2));
  nav2_util::declare_parameter_if_not_declared(
    node, "imu_yaw_sign", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, "imu_gyro_deadband_rad_s", rclcpp::ParameterValue(0.03));
  nav2_util::declare_parameter_if_not_declared(
    node, "imu_max_delta_rad", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node, "imu_settle_time_sec", rclcpp::ParameterValue(0.3));
  nav2_util::declare_parameter_if_not_declared(
    node, "imu_stopped_rate_rad_s", rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(
    node, "imu_pulse_period_sec", rclcpp::ParameterValue(0.24));
  nav2_util::declare_parameter_if_not_declared(
    node, "imu_pulse_on_sec", rclcpp::ParameterValue(0.08));
  nav2_util::declare_parameter_if_not_declared(
    node, "simulate_ahead_time", rclcpp::ParameterValue(1.5));

  std::string imu_topic;
  node->get_parameter("imu_topic", imu_topic);
  node->get_parameter("imu_min_angular_speed", min_angular_speed_);
  node->get_parameter("imu_max_angular_speed", max_angular_speed_);
  node->get_parameter("imu_proportional_gain", proportional_gain_);
  node->get_parameter("imu_slow_down_angle", slow_down_angle_);
  node->get_parameter("imu_angle_tolerance", angle_tolerance_);
  node->get_parameter("imu_timeout_sec", imu_timeout_sec_);
  node->get_parameter("imu_yaw_sign", imu_yaw_sign_);
  node->get_parameter("imu_gyro_deadband_rad_s", imu_gyro_deadband_);
  node->get_parameter("imu_max_delta_rad", imu_max_delta_);
  node->get_parameter("imu_settle_time_sec", settle_time_sec_);
  node->get_parameter("imu_stopped_rate_rad_s", stopped_rate_);
  node->get_parameter("imu_pulse_period_sec", pulse_period_sec_);
  node->get_parameter("imu_pulse_on_sec", pulse_on_sec_);
  node->get_parameter("simulate_ahead_time", simulate_ahead_time_);

  for (const double value : {min_angular_speed_, max_angular_speed_, proportional_gain_,
      angle_tolerance_, slow_down_angle_, imu_timeout_sec_, imu_yaw_sign_, imu_gyro_deadband_,
      imu_max_delta_, settle_time_sec_, stopped_rate_, pulse_period_sec_, pulse_on_sec_,
      simulate_ahead_time_})
  {
    if (!std::isfinite(value)) {
      throw std::runtime_error("Non-finite IMU spin parameter");
    }
  }
  if (min_angular_speed_ < 0.55 || max_angular_speed_ > 0.60 ||
    min_angular_speed_ > max_angular_speed_ || imu_timeout_sec_ <= 0.0 ||
    std::abs(imu_yaw_sign_) != 1.0 || imu_gyro_deadband_ < 0.0 ||
    imu_max_delta_ <= 0.0 || settle_time_sec_ <= 0.0 || stopped_rate_ <= 0.0 ||
    slow_down_angle_ <= angle_tolerance_ || simulate_ahead_time_ <= 0.0 ||
    proportional_gain_ <= 0.0 || angle_tolerance_ <= 0.0 ||
    pulse_period_sec_ <= 0.0 || pulse_on_sec_ <= 0.0 ||
    pulse_on_sec_ > pulse_period_sec_)
  {
    throw std::runtime_error("Invalid IMU spin controller parameters");
  }

  imu_subscription_ = node->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic, rclcpp::SensorDataQoS(),
    std::bind(&ImuSpin::imuCallback, this, std::placeholders::_1));
  RCLCPP_INFO(
    logger_, "Configured timestamped gyro spin: min=%.2f max=%.2f tolerance=%.3f rad",
    min_angular_speed_, max_angular_speed_, angle_tolerance_);
}

void ImuSpin::onCleanup()
{
  imu_subscription_.reset();
  std::lock_guard<std::mutex> lock(imu_mutex_);
  gyro_yaw_.invalidate();
}

void ImuSpin::imuCallback(const sensor_msgs::msg::Imu::SharedPtr message)
{
  std::lock_guard<std::mutex> lock(imu_mutex_);
  gyro_yaw_.update(
    static_cast<int64_t>(message->header.stamp.sec) * 1000000000LL +
    message->header.stamp.nanosec, clock_->now().nanoseconds(),
    message->angular_velocity.z, imu_yaw_sign_, imu_gyro_deadband_,
    imu_timeout_sec_, imu_max_delta_);
}

nav2_behaviors::Status ImuSpin::onRun(
  const std::shared_ptr<const nav2_msgs::action::Spin::Goal> command)
{
  std::lock_guard<std::mutex> lock(imu_mutex_);
  if (!gyro_yaw_.fresh(clock_->now().nanoseconds(), imu_timeout_sec_) ||
    !std::isfinite(command->target_yaw) || command->time_allowance.sec < 0 ||
    command->time_allowance.nanosec >= 1000000000u)
  {
    stopRobot();
    RCLCPP_ERROR(logger_, "IMU data is stale or spin goal is invalid");
    return nav2_behaviors::Status::FAILED;
  }

  start_yaw_ = gyro_yaw_.yaw();
  goal_imu_generation_ = gyro_yaw_.generation();
  requested_yaw_ = command->target_yaw;
  target_yaw_ = start_yaw_ + requested_yaw_;
  command_time_allowance_ = command->time_allowance;
  if (command_time_allowance_.seconds() <= 0.0) {
    command_time_allowance_ = rclcpp::Duration::from_seconds(15.0);
  }
  started_ = std::chrono::steady_clock::now();
  settling_ = false;

  RCLCPP_INFO(
    logger_, "IMU spin target %.1f degrees", requested_yaw_ * 180.0 / M_PI);
  return nav2_behaviors::Status::SUCCEEDED;
}

nav2_behaviors::Status ImuSpin::onCycleUpdate()
{
  const double elapsed = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - started_).count();
  if (elapsed > command_time_allowance_.seconds()) {
    stopRobot();
    RCLCPP_WARN(logger_, "IMU spin exceeded time allowance");
    return nav2_behaviors::Status::FAILED;
  }

  double current_yaw;
  double current_rate;
  {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    if (!gyro_yaw_.fresh(clock_->now().nanoseconds(), imu_timeout_sec_) ||
      gyro_yaw_.generation() != goal_imu_generation_)
    {
      stopRobot();
      RCLCPP_ERROR(logger_, "IMU stale or discontinuous during spin; aborting");
      return nav2_behaviors::Status::FAILED;
    }
    current_yaw = gyro_yaw_.yaw();
    current_rate = gyro_yaw_.rate();
  }

  const double error = target_yaw_ - current_yaw;
  const double remaining = std::abs(error);
  feedback_->angular_distance_traveled = static_cast<float>(current_yaw - start_yaw_);
  action_server_->publish_feedback(feedback_);

  if (remaining <= angle_tolerance_) {
    stopRobot();
    if (std::abs(current_rate) > stopped_rate_) {
      settling_ = false;
      return nav2_behaviors::Status::RUNNING;
    }
    if (!settling_) {
      settled_since_ = std::chrono::steady_clock::now();
      settling_ = true;
    }
    if (std::chrono::duration<double>(
        std::chrono::steady_clock::now() - settled_since_).count() < settle_time_sec_)
    {
      return nav2_behaviors::Status::RUNNING;
    }
    RCLCPP_INFO(logger_, "IMU spin complete, error %.2f degrees", error * 180.0 / M_PI);
    return nav2_behaviors::Status::SUCCEEDED;
  }
  settling_ = false;

  double speed = std::min(max_angular_speed_, proportional_gain_ * remaining);
  speed = std::max(min_angular_speed_, speed);
  if (remaining <= slow_down_angle_) {
    const double phase = std::fmod(elapsed, pulse_period_sec_);
    if (phase >= pulse_on_sec_) {
      speed = 0.0;
    }
  }

  auto command = std::make_unique<geometry_msgs::msg::Twist>();
  command->angular.z = std::copysign(speed, error);

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_))
  {
    stopRobot();
    RCLCPP_ERROR(logger_, "Current robot pose is not available");
    return nav2_behaviors::Status::FAILED;
  }
  geometry_msgs::msg::Pose2D pose;
  pose.x = current_pose.pose.position.x;
  pose.y = current_pose.pose.position.y;
  pose.theta = tf2::getYaw(current_pose.pose.orientation);
  if (!isCollisionFree(remaining, command.get(), pose)) {
    stopRobot();
    RCLCPP_WARN(logger_, "Collision predicted during IMU spin");
    return nav2_behaviors::Status::FAILED;
  }

  vel_pub_->publish(std::move(command));
  return nav2_behaviors::Status::RUNNING;
}

bool ImuSpin::isCollisionFree(
  double remaining_yaw,
  geometry_msgs::msg::Twist * command,
  geometry_msgs::msg::Pose2D & pose)
{
  const int cycles = std::max(1, static_cast<int>(std::ceil(cycle_frequency_ * simulate_ahead_time_)));
  const auto initial_pose = pose;
  for (int cycle = 0; cycle <= cycles; ++cycle) {
    const double change = std::copysign(
      std::min(remaining_yaw, std::abs(command->angular.z) * (cycle / cycle_frequency_)),
      command->angular.z);
    pose.theta = initial_pose.theta + change;
    if (!collision_checker_->isCollisionFree(pose, cycle == 0)) {
      return false;
    }
    if (std::abs(change) >= remaining_yaw) {
      break;
    }
  }
  return true;
}

}  // namespace wheeltec_nav2

PLUGINLIB_EXPORT_CLASS(wheeltec_nav2::ImuSpin, nav2_core::Behavior)
