#ifndef WHEELTEC_NAV2__HYSTERESIS_PATH_CONTROLLER_HPP_
#define WHEELTEC_NAV2__HYSTERESIS_PATH_CONTROLLER_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace wheeltec_nav2
{

class HysteresisPathController : public nav2_core::Controller
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path & path) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

private:
  enum class MotionState
  {
    DRIVE,
    TURN_LEFT,
    TURN_RIGHT
  };

  geometry_msgs::msg::PoseStamped selectLookaheadPose(
    const geometry_msgs::msg::PoseStamped & robot_pose) const;
  bool commandIsCollisionFree(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    double linear_velocity,
    double angular_velocity) const;
  static double normalizeAngle(double angle);

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string plugin_name_;
  nav_msgs::msg::Path global_plan_;
  MotionState motion_state_{MotionState::DRIVE};
  double linear_speed_{0.15};
  double configured_linear_speed_{0.15};
  double turn_speed_{0.55};
  double turn_enter_angle_{0.314159};
  double turn_exit_angle_{0.139626};
  double lookahead_distance_{0.30};
  double goal_distance_{0.10};
  double collision_check_time_{1.0};
  double collision_check_step_{0.05};
  double transform_tolerance_{0.2};
};

}  // namespace wheeltec_nav2

#endif  // WHEELTEC_NAV2__HYSTERESIS_PATH_CONTROLLER_HPP_
