#include "wheeltec_nav2/final_imu_spin_action.hpp"

#include <cmath>
#include <memory>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/robot_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"

namespace
{
double wrapAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}
}  // namespace

namespace wheeltec_nav2
{

FinalImuSpinAction::FinalImuSpinAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & configuration)
: nav2_behavior_tree::BtActionNode<nav2_msgs::action::Spin>(
    xml_tag_name, action_name, configuration)
{
}

BT::PortsList FinalImuSpinAction::providedPorts()
{
  return providedBasicPorts(
    {
      BT::InputPort<geometry_msgs::msg::PoseStamped>(
        "goal", "Final navigation goal pose"),
      BT::InputPort<double>(
        "time_allowance", 15.0, "Allowed time for final rotation")
    });
}

void FinalImuSpinAction::on_tick()
{
  geometry_msgs::msg::PoseStamped goal_pose;
  if (!getInput("goal", goal_pose)) {
    RCLCPP_ERROR(node_->get_logger(), "FinalImuSpin cannot read navigation goal");
    should_send_goal_ = false;
    return;
  }

  auto tf_buffer = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_buffer, goal_pose.header.frame_id, "base_link", 0.5))
  {
    RCLCPP_ERROR(node_->get_logger(), "FinalImuSpin cannot read current robot pose");
    should_send_goal_ = false;
    return;
  }

  const double goal_yaw = tf2::getYaw(goal_pose.pose.orientation);
  const double current_yaw = tf2::getYaw(current_pose.pose.orientation);
  goal_.target_yaw = static_cast<float>(wrapAngle(goal_yaw - current_yaw));

  double time_allowance = 15.0;
  getInput("time_allowance", time_allowance);
  goal_.time_allowance.sec = static_cast<int32_t>(time_allowance);
  goal_.time_allowance.nanosec = static_cast<uint32_t>(
    (time_allowance - static_cast<double>(goal_.time_allowance.sec)) * 1e9);

  RCLCPP_INFO(
    node_->get_logger(), "Final IMU rotation target %.1f degrees",
    static_cast<double>(goal_.target_yaw) * 180.0 / M_PI);
}

}  // namespace wheeltec_nav2

BT_REGISTER_NODES(factory)
{
  factory.registerBuilder(
    BT::CreateManifest<wheeltec_nav2::FinalImuSpinAction>(
      "FinalImuSpin", wheeltec_nav2::FinalImuSpinAction::providedPorts()),
    [](const std::string & name, const BT::NodeConfiguration & configuration) {
      return std::make_unique<wheeltec_nav2::FinalImuSpinAction>(
        name, "/spin", configuration);
    });
}
