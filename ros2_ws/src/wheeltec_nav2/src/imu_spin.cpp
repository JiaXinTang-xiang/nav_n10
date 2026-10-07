#include "wheeltec_nav2/imu_spin.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"

namespace
{
double yawFromQuaternion(const geometry_msgs::msg::Quaternion & quaternion)
{
  return std::atan2(
    2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
    1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z));
}

double wrapAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}
}  // namespace

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
    node, "imu_min_angular_speed", rclcpp::ParameterValue(0.60));
  nav2_util::declare_parameter_if_not_declared(
    node, "imu_max_angular_speed", rclcpp::ParameterValue(0.80));
  nav2_util::declare_parameter_if_not_declared(
    node, "imu_proportional_gain", rclcpp::ParameterValue(1.8));
  nav2_util::declare_parameter_if_not_declared(
    node, "imu_slow_down_angle", rclcpp::ParameterValue(0.45));
  nav2_util::declare_parameter_if_not_declared(
    node, "imu_angle_tolerance", rclcpp::ParameterValue(0.045));
  nav2_util::declare_parameter_if_not_declared(
    node, "imu_timeout_sec", rclcpp::ParameterValue(0.25));
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
  node->get_parameter("imu_pulse_period_sec", pulse_period_sec_);
  node->get_parameter("imu_pulse_on_sec", pulse_on_sec_);
  node->get_parameter("simulate_ahead_time", simulate_ahead_time_);

  if (min_angular_speed_ <= 0.0 || min_angular_speed_ > max_angular_speed_ ||
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
    logger_, "Configured IMU spin: min=%.2f max=%.2f tolerance=%.3f rad",
    min_angular_speed_, max_angular_speed_, angle_tolerance_);
}

void ImuSpin::onCleanup()
{
  imu_subscription_.reset();
}

void ImuSpin::imuCallback(const sensor_msgs::msg::Imu::SharedPtr message)
{
  const double raw_yaw = yawFromQuaternion(message->orientation);
  const auto now = clock_->now();
  std::lock_guard<std::mutex> lock(imu_mutex_);
  if (!imu_received_) {
    imu_unwrapped_yaw_ = raw_yaw;
    imu_received_ = true;
  } else {
    imu_unwrapped_yaw_ += wrapAngle(raw_yaw - imu_last_raw_yaw_);
  }
  imu_last_raw_yaw_ = raw_yaw;
  imu_last_time_ = now;
}

nav2_behaviors::Status ImuSpin::onRun(
  const std::shared_ptr<const nav2_msgs::action::Spin::Goal> command)
{
  std::lock_guard<std::mutex> lock(imu_mutex_);
  if (!imu_received_) {
    RCLCPP_ERROR(logger_, "No valid IMU yaw received");
    return nav2_behaviors::Status::FAILED;
  }
  if ((clock_->now() - imu_last_time_).seconds() > imu_timeout_sec_) {
    RCLCPP_ERROR(logger_, "IMU data is stale");
    return nav2_behaviors::Status::FAILED;
  }

  start_yaw_ = imu_unwrapped_yaw_;
  requested_yaw_ = command->target_yaw;
  target_yaw_ = start_yaw_ + requested_yaw_;
  command_time_allowance_ = command->time_allowance;
  end_time_ = clock_->now() + command_time_allowance_;

  RCLCPP_INFO(
    logger_, "IMU spin target %.1f degrees", requested_yaw_ * 180.0 / M_PI);
  return nav2_behaviors::Status::SUCCEEDED;
}

nav2_behaviors::Status ImuSpin::onCycleUpdate()
{
  if (command_time_allowance_.seconds() > 0.0 && clock_->now() > end_time_) {
    stopRobot();
    RCLCPP_WARN(logger_, "IMU spin exceeded time allowance");
    return nav2_behaviors::Status::FAILED;
  }

  double current_yaw;
  rclcpp::Time imu_time;
  {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    if (!imu_received_) {
      stopRobot();
      return nav2_behaviors::Status::FAILED;
    }
    current_yaw = imu_unwrapped_yaw_;
    imu_time = imu_last_time_;
  }
  if ((clock_->now() - imu_time).seconds() > imu_timeout_sec_) {
    stopRobot();
    RCLCPP_ERROR(logger_, "IMU data timeout during spin");
    return nav2_behaviors::Status::FAILED;
  }

  const double error = target_yaw_ - current_yaw;
  const double remaining = std::abs(error);
  feedback_->angular_distance_traveled = static_cast<float>(current_yaw - start_yaw_);
  action_server_->publish_feedback(feedback_);

  if (remaining <= angle_tolerance_) {
    stopRobot();
    RCLCPP_INFO(logger_, "IMU spin complete, error %.2f degrees", error * 180.0 / M_PI);
    return nav2_behaviors::Status::SUCCEEDED;
  }

  double speed = std::min(max_angular_speed_, proportional_gain_ * remaining);
  speed = std::max(min_angular_speed_, speed);
  if (remaining <= slow_down_angle_) {
    const double phase = std::fmod((clock_->now() - (end_time_ - command_time_allowance_)).seconds(),
      pulse_period_sec_);
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
  if (!isCollisionFree(current_yaw - start_yaw_, command.get(), pose)) {
    stopRobot();
    RCLCPP_WARN(logger_, "Collision predicted during IMU spin");
    return nav2_behaviors::Status::FAILED;
  }

  vel_pub_->publish(std::move(command));
  return nav2_behaviors::Status::RUNNING;
}

bool ImuSpin::isCollisionFree(
  double relative_yaw,
  geometry_msgs::msg::Twist * command,
  geometry_msgs::msg::Pose2D & pose)
{
  const int cycles = static_cast<int>(cycle_frequency_ * simulate_ahead_time_);
  const auto initial_pose = pose;
  for (int cycle = 0; cycle < cycles; ++cycle) {
    const double change = command->angular.z * (cycle / cycle_frequency_);
    pose.theta = initial_pose.theta + change;
    if (std::abs(relative_yaw) - std::abs(change) <= 0.0) {
      break;
    }
    if (!collision_checker_->isCollisionFree(pose, cycle == 0)) {
      return false;
    }
  }
  return true;
}

}  // namespace wheeltec_nav2

PLUGINLIB_EXPORT_CLASS(wheeltec_nav2::ImuSpin, nav2_core::Behavior)
