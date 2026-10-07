#ifndef WHEELTEC_NAV2__IMU_SPIN_HPP_
#define WHEELTEC_NAV2__IMU_SPIN_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_msgs/action/spin.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace wheeltec_nav2
{

class ImuSpin : public nav2_behaviors::TimedBehavior<nav2_msgs::action::Spin>
{
public:
  ImuSpin();
  ~ImuSpin() override = default;

  nav2_behaviors::Status onRun(
    const std::shared_ptr<const nav2_msgs::action::Spin::Goal> command) override;
  nav2_behaviors::Status onCycleUpdate() override;
  void onConfigure() override;
  void onCleanup() override;

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr message);
  bool isCollisionFree(
    double relative_yaw,
    geometry_msgs::msg::Twist * command,
    geometry_msgs::msg::Pose2D & pose);

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  std::mutex imu_mutex_;
  double imu_unwrapped_yaw_{0.0};
  double imu_last_raw_yaw_{0.0};
  rclcpp::Time imu_last_time_{0, 0, RCL_ROS_TIME};
  bool imu_received_{false};

  double min_angular_speed_{0.60};
  double max_angular_speed_{0.80};
  double proportional_gain_{1.8};
  double slow_down_angle_{0.45};
  double angle_tolerance_{0.045};
  double imu_timeout_sec_{0.25};
  double pulse_period_sec_{0.24};
  double pulse_on_sec_{0.08};
  double simulate_ahead_time_{1.5};

  std::shared_ptr<nav2_msgs::action::Spin::Feedback> feedback_;

  double start_yaw_{0.0};
  double target_yaw_{0.0};
  double requested_yaw_{0.0};
  rclcpp::Duration command_time_allowance_{0, 0};
  rclcpp::Time end_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace wheeltec_nav2

#endif  // WHEELTEC_NAV2__IMU_SPIN_HPP_
