#include "wheeltec_nav2/hysteresis_path_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <utility>

#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"

namespace wheeltec_nav2
{

void HysteresisPathController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  if (!node_) {
    throw std::runtime_error("Failed to lock controller lifecycle node");
  }

  plugin_name_ = std::move(name);
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);

  nav2_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".linear_speed", rclcpp::ParameterValue(0.15));
  nav2_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".turn_speed", rclcpp::ParameterValue(0.55));
  nav2_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".turn_enter_angle", rclcpp::ParameterValue(0.314159));
  nav2_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".turn_exit_angle", rclcpp::ParameterValue(0.139626));
  nav2_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".lookahead_distance", rclcpp::ParameterValue(0.30));
  nav2_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".goal_distance", rclcpp::ParameterValue(0.10));
  nav2_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".collision_check_time", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".collision_check_step", rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.2));

  node_->get_parameter(plugin_name_ + ".linear_speed", configured_linear_speed_);
  node_->get_parameter(plugin_name_ + ".turn_speed", turn_speed_);
  node_->get_parameter(plugin_name_ + ".turn_enter_angle", turn_enter_angle_);
  node_->get_parameter(plugin_name_ + ".turn_exit_angle", turn_exit_angle_);
  node_->get_parameter(plugin_name_ + ".lookahead_distance", lookahead_distance_);
  node_->get_parameter(plugin_name_ + ".goal_distance", goal_distance_);
  node_->get_parameter(plugin_name_ + ".collision_check_time", collision_check_time_);
  node_->get_parameter(plugin_name_ + ".collision_check_step", collision_check_step_);
  node_->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance_);
  linear_speed_ = configured_linear_speed_;

  if (
    configured_linear_speed_ <= 0.0 || turn_speed_ <= 0.0 ||
    turn_exit_angle_ <= 0.0 || turn_enter_angle_ <= turn_exit_angle_ ||
    lookahead_distance_ <= 0.0 || goal_distance_ <= 0.0 ||
    collision_check_time_ <= 0.0 || collision_check_step_ <= 0.0)
  {
    throw std::runtime_error("Invalid hysteresis path controller parameters");
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "Configured hysteresis controller: linear=%.2f turn=%.2f enter=%.1fdeg exit=%.1fdeg",
    linear_speed_, turn_speed_, turn_enter_angle_ * 180.0 / M_PI,
    turn_exit_angle_ * 180.0 / M_PI);
}

void HysteresisPathController::cleanup()
{
  global_plan_.poses.clear();
  motion_state_ = MotionState::DRIVE;
}

void HysteresisPathController::activate()
{
  motion_state_ = MotionState::DRIVE;
}

void HysteresisPathController::deactivate()
{
  motion_state_ = MotionState::DRIVE;
}

void HysteresisPathController::setPlan(const nav_msgs::msg::Path & path)
{
  if (path.poses.empty()) {
    throw nav2_core::PlannerException("Received an empty global path");
  }
  global_plan_ = path;
}

void HysteresisPathController::setSpeedLimit(
  const double & speed_limit, const bool & percentage)
{
  if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {
    linear_speed_ = configured_linear_speed_;
  } else if (percentage) {
    linear_speed_ = configured_linear_speed_ * speed_limit / 100.0;
  } else {
    linear_speed_ = std::min(configured_linear_speed_, speed_limit);
  }
  linear_speed_ = std::max(0.0, linear_speed_);
}

geometry_msgs::msg::TwistStamped HysteresisPathController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist &,
  nav2_core::GoalChecker *)
{
  if (global_plan_.poses.empty()) {
    throw nav2_core::PlannerException("No global path is available");
  }

  geometry_msgs::msg::PoseStamped robot_in_plan;
  if (!nav2_util::transformPoseInTargetFrame(
      pose, robot_in_plan, *tf_, global_plan_.header.frame_id, transform_tolerance_))
  {
    throw nav2_core::PlannerException("Cannot transform robot pose into global path frame");
  }

  const auto & goal = global_plan_.poses.back().pose.position;
  const auto & robot = robot_in_plan.pose.position;
  const double goal_distance = std::hypot(goal.x - robot.x, goal.y - robot.y);

  geometry_msgs::msg::TwistStamped command;
  command.header.stamp = node_->get_clock()->now();
  command.header.frame_id = costmap_ros_->getBaseFrameID();
  if (goal_distance <= goal_distance_) {
    motion_state_ = MotionState::DRIVE;
    return command;
  }

  const auto target = selectLookaheadPose(robot_in_plan);
  const double target_heading = std::atan2(
    target.pose.position.y - robot.y,
    target.pose.position.x - robot.x);
  const double robot_heading = tf2::getYaw(robot_in_plan.pose.orientation);
  const double heading_error = normalizeAngle(target_heading - robot_heading);

  if (motion_state_ == MotionState::DRIVE) {
    if (heading_error >= turn_enter_angle_) {
      motion_state_ = MotionState::TURN_LEFT;
    } else if (heading_error <= -turn_enter_angle_) {
      motion_state_ = MotionState::TURN_RIGHT;
    }
  } else if (motion_state_ == MotionState::TURN_LEFT) {
    if (std::abs(heading_error) <= turn_exit_angle_) {
      motion_state_ = MotionState::DRIVE;
    } else if (heading_error <= -turn_enter_angle_) {
      motion_state_ = MotionState::TURN_RIGHT;
    }
  } else {
    if (std::abs(heading_error) <= turn_exit_angle_) {
      motion_state_ = MotionState::DRIVE;
    } else if (heading_error >= turn_enter_angle_) {
      motion_state_ = MotionState::TURN_LEFT;
    }
  }

  if (motion_state_ == MotionState::DRIVE) {
    command.twist.linear.x = linear_speed_;
  } else if (motion_state_ == MotionState::TURN_LEFT) {
    command.twist.angular.z = turn_speed_;
  } else {
    command.twist.angular.z = -turn_speed_;
  }

  if (!commandIsCollisionFree(pose, command.twist.linear.x, command.twist.angular.z)) {
    throw nav2_core::PlannerException("Hysteresis controller command would collide");
  }

  return command;
}

geometry_msgs::msg::PoseStamped HysteresisPathController::selectLookaheadPose(
  const geometry_msgs::msg::PoseStamped & robot_pose) const
{
  std::size_t nearest_index = 0;
  double nearest_distance = std::numeric_limits<double>::max();
  for (std::size_t index = 0; index < global_plan_.poses.size(); ++index) {
    const auto & point = global_plan_.poses[index].pose.position;
    const double distance = std::hypot(
      point.x - robot_pose.pose.position.x,
      point.y - robot_pose.pose.position.y);
    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest_index = index;
    }
  }

  double accumulated_distance = 0.0;
  for (std::size_t index = nearest_index + 1; index < global_plan_.poses.size(); ++index) {
    const auto & previous = global_plan_.poses[index - 1].pose.position;
    const auto & current = global_plan_.poses[index].pose.position;
    accumulated_distance += std::hypot(current.x - previous.x, current.y - previous.y);
    if (accumulated_distance >= lookahead_distance_) {
      return global_plan_.poses[index];
    }
  }
  return global_plan_.poses.back();
}

bool HysteresisPathController::commandIsCollisionFree(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  double linear_velocity,
  double angular_velocity) const
{
  geometry_msgs::msg::PoseStamped costmap_pose;
  if (!nav2_util::transformPoseInTargetFrame(
      robot_pose, costmap_pose, *tf_, costmap_ros_->getGlobalFrameID(), transform_tolerance_))
  {
    return false;
  }

  auto * costmap = costmap_ros_->getCostmap();
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(costmap);
  const auto footprint = costmap_ros_->getRobotFootprint();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap->getMutex());

  double x = costmap_pose.pose.position.x;
  double y = costmap_pose.pose.position.y;
  double yaw = tf2::getYaw(costmap_pose.pose.orientation);
  for (double elapsed = 0.0; elapsed <= collision_check_time_; elapsed += collision_check_step_) {
    unsigned int map_x = 0;
    unsigned int map_y = 0;
    if (!costmap->worldToMap(x, y, map_x, map_y)) {
      return false;
    }
    if (costmap->getCost(map_x, map_y) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      return false;
    }
    if (checker.footprintCostAtPose(x, y, yaw, footprint) < 0.0) {
      return false;
    }

    if (std::abs(angular_velocity) < 1e-6) {
      x += linear_velocity * std::cos(yaw) * collision_check_step_;
      y += linear_velocity * std::sin(yaw) * collision_check_step_;
    } else {
      const double next_yaw = yaw + angular_velocity * collision_check_step_;
      const double radius = linear_velocity / angular_velocity;
      x += radius * (std::sin(next_yaw) - std::sin(yaw));
      y -= radius * (std::cos(next_yaw) - std::cos(yaw));
      yaw = normalizeAngle(next_yaw);
    }
  }
  return true;
}

double HysteresisPathController::normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace wheeltec_nav2

PLUGINLIB_EXPORT_CLASS(wheeltec_nav2::HysteresisPathController, nav2_core::Controller)
