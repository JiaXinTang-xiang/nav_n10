// Nav2 owns all obstacle decisions. This node only supervises the motion boundary.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include "action_msgs/msg/goal_status_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_stm32_bridge/msg/bridge_status.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
namespace robot_navigation {
namespace msg = robot_stm32_bridge::msg;
using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;
constexpr uint16_t kMotionMotorAuthorized = 1U << 7U;
constexpr uint16_t kMotionStbyEnabled = 1U << 8U;
constexpr uint16_t kMotionBodyCommandReady = 1U << 15U;
constexpr int64_t kScanFutureToleranceNs = 50000000LL;
int64_t stamp_nanoseconds(const builtin_interfaces::msg::Time &stamp) {
  return static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}
bool stamp_is_zero(const builtin_interfaces::msg::Time &stamp) {
  return stamp.sec == 0 && stamp.nanosec == 0;
}
double angular_difference(double a, double b) {
  return std::atan2(std::sin(a-b), std::cos(a-b));
}
class NavigationMotionGate final : public rclcpp::Node {
public:
  NavigationMotionGate() : Node("navigation_safety_gate") {
    localization_timeout_ms_ = declare_parameter<int>("localization_timeout_ms", 700);
    scan_timeout_ms_ = declare_parameter<int>("scan_timeout_ms", 400);
    bridge_timeout_ms_ = declare_parameter<int>("bridge_timeout_ms", 300);
    input_cmd_timeout_ms_ = declare_parameter<int>("input_cmd_timeout_ms", 250);
    max_linear_speed_mps_ = declare_parameter<double>("max_linear_speed_mps", 0.20);
    max_angular_speed_radps_ = declare_parameter<double>("max_angular_speed_radps", 1.50);
    if (localization_timeout_ms_ < 100 || scan_timeout_ms_ < 100 ||
        bridge_timeout_ms_ < 100 || input_cmd_timeout_ms_ < 100 || input_cmd_timeout_ms_ > 250 ||
        !std::isfinite(max_linear_speed_mps_) || max_linear_speed_mps_ <= 0 || max_linear_speed_mps_ > 0.30 ||
        !std::isfinite(max_angular_speed_radps_) || max_angular_speed_radps_ <= 0 || max_angular_speed_radps_ > 1.50) {
      throw std::invalid_argument("invalid motion boundary limits/timeouts");
    }
    const auto output = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    const auto input = declare_parameter<std::string>("input_cmd_vel_topic", "/cmd_vel_nav");
    if (input == output) { throw std::invalid_argument("Twist input/output must differ"); }
    command_publisher_ = create_publisher<geometry_msgs::msg::Twist>(output, 1);
    status_publisher_ = create_publisher<std_msgs::msg::String>("~/status", 10);
    plan_clear_publisher_ = create_publisher<nav_msgs::msg::Path>(
        declare_parameter<std::string>("visualized_plan_topic", "/plan"), 1);
    scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
        declare_parameter<std::string>("scan_topic", "/scan"), rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::LaserScan::SharedPtr scan) { on_scan(scan); });
    bridge_subscription_ = create_subscription<msg::BridgeStatus>(
        declare_parameter<std::string>("bridge_status_topic", "/stm32/status"), 10,
        [this](msg::BridgeStatus::SharedPtr status) {
          bridge_status_ = *status;
          have_bridge_status_ = true;
          last_bridge_status_received_ = Clock::now();
          if (have_goal_ && !navigation_health()) { latch_stopped("bridge/system health lost"); }
        });
    input_command_subscription_ = create_subscription<geometry_msgs::msg::Twist>(input, 1,
        [this](geometry_msgs::msg::Twist::SharedPtr command) { on_input_command(command); });
    controller_status_subscription_ = create_subscription<action_msgs::msg::GoalStatusArray>(
        "/follow_path/_action/status", rclcpp::QoS(1).transient_local().reliable(),
        [this](action_msgs::msg::GoalStatusArray::SharedPtr status) { on_controller_status(*status); });
    arm_client_ = create_client<std_srvs::srv::Trigger>(
        declare_parameter<std::string>("arm_service", "/robot_stm32_bridge/arm"));
    disarm_client_ = create_client<std_srvs::srv::Trigger>(
        declare_parameter<std::string>("disarm_service", "/robot_stm32_bridge/disarm"));
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);
    navigation_client_ = rclcpp_action::create_client<Navigate>(this,
        declare_parameter<std::string>("navigation_action", "/navigate_to_pose"));
    goal_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        declare_parameter<std::string>("goal_topic", "/goal_pose"), rclcpp::QoS(1),
        [this](geometry_msgs::msg::PoseStamped::SharedPtr pose) { on_goal(pose); });
    reset_service_ = create_service<std_srvs::srv::Trigger>("~/reset",
        std::bind(&NavigationMotionGate::on_reset, this, std::placeholders::_1, std::placeholders::_2));
    stop_service_ = create_service<std_srvs::srv::Trigger>("~/stop",
        [this](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr response) {
          latch_stopped("operator STOP; manual reset required");
          response->success = true;
          response->message = "STOP latched; goal canceled; reset and NEW goal required";
        });
    latch_file_ = declare_parameter<std::string>("latch_file", "");
    std::error_code latch_error;
    const bool saved_latch = !latch_file_.empty() && std::filesystem::exists(latch_file_, latch_error);
    if (declare_parameter<bool>("startup_locked", false) || saved_latch || latch_error) {
      navigation_phase_ = NavigationPhase::kLocked;
      reason_ = "saved STOP/fault or maintenance lock; manual reset required";
    }
    control_timer_ = create_wall_timer(20ms, [this]() { control_tick(); });
    status_timer_ = create_wall_timer(100ms, [this]() {
      std_msgs::msg::String status;
      status.data = std::string(state_name()) + "; " + reason_;
      status_publisher_->publish(status);
    });
    publish_zero_twist();
    RCLCPP_INFO(get_logger(), "Native Nav2 motion boundary ready; %s", reason_.c_str());
  }
  ~NavigationMotionGate() override { publish_zero_twist(); }
private:
  using Navigate = nav2_msgs::action::NavigateToPose;
  using NavigationHandle = rclcpp_action::ClientGoalHandle<Navigate>;
  enum class NavigationPhase { kIdle, kActive, kLocked };
  enum class State { kStopped, kRunning };
  const char *state_name() const { return state_ == State::kRunning ? "RUNNING" : "STOPPED"; }
  uint32_t
  age_ms(const std::chrono::steady_clock::time_point &time_point) const {
    if (time_point == std::chrono::steady_clock::time_point{}) {
      return std::numeric_limits<uint32_t>::max();
    }
    const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
                         std::chrono::steady_clock::now() - time_point)
                         .count();
    if (age <= 0) {
      return 0U;
    }
    return static_cast<uint32_t>(
        std::min<int64_t>(age, std::numeric_limits<uint32_t>::max()));
  }

  bool scan_fresh() const {
    const auto age = now().nanoseconds() - stamp_nanoseconds(last_scan_source_stamp_);
    return have_scan_ && !stamp_is_zero(last_scan_source_stamp_) &&
        age_ms(last_scan_received_) <= static_cast<uint32_t>(scan_timeout_ms_) &&
        age >= -kScanFutureToleranceNs && age <= static_cast<int64_t>(scan_timeout_ms_) * 1000000;
  }
  bool bridge_status_fresh() const {
    return have_bridge_status_ && age_ms(last_bridge_status_received_) <=
                                      static_cast<uint32_t>(bridge_timeout_ms_);
  }

  bool input_command_fresh() const {
    return (have_input_command_ &&
            age_ms(last_input_command_received_) <=
                static_cast<uint32_t>(input_cmd_timeout_ms_));
  }

  bool bridge_healthy() const {
    return bridge_status_fresh() && bridge_status_.connected &&
           bridge_status_.protocol_valid && bridge_status_.can_healthy &&
           bridge_status_.status_fresh && !bridge_status_.stm32_critical_fault;
  }

  bool motion_path_ready() const {
    const bool state_allows_motion =
        bridge_status_.system_state == msg::BridgeStatus::SYSTEM_SAFE ||
        bridge_status_.system_state == msg::BridgeStatus::SYSTEM_READY ||
        bridge_status_.system_state == msg::BridgeStatus::SYSTEM_ACTIVE;
    return bridge_healthy() && state_allows_motion &&
           (bridge_status_.motion_flags & kMotionBodyCommandReady) != 0U;
  }

  void transition(const State next, const std::string &reason) {
    if (state_ != next || reason_ != reason) {
      RCLCPP_INFO(get_logger(), "Motion gate %s -> %s: %s", state_name(),
                  next == State::kRunning ? "RUNNING" : "STOPPED",
                  reason.c_str());
    }
    state_ = next;
    reason_ = reason;
  }

  bool fully_disarmed() const {
    return bridge_status_fresh() && !bridge_status_.arm_requested &&
        !bridge_status_.authority_armed &&
        (bridge_status_.motion_flags & (kMotionMotorAuthorized | kMotionStbyEnabled)) == 0U;
  }

  bool localization_fresh() const {
    if (!tf_buffer_) {
      return false;
    }
    try {
      const auto transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
      const int64_t age = now().nanoseconds() - stamp_nanoseconds(transform.header.stamp);
      const auto &p = transform.transform.translation;
      const auto &q = transform.transform.rotation;
      return !stamp_is_zero(transform.header.stamp) && age >= -kScanFutureToleranceNs &&
          age <= static_cast<int64_t>(localization_timeout_ms_) * 1000000LL &&
          std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(q.z) && std::isfinite(q.w);
    } catch (const tf2::TransformException &) {
      return false;
    }
  }

  bool navigation_health() const {
    return scan_fresh() && motion_path_ready() && bridge_status_.fault_flags == 0U &&
           localization_fresh();
  }

  void on_goal(const geometry_msgs::msg::PoseStamped::SharedPtr pose) {
    if (navigation_phase_ == NavigationPhase::kLocked) {
      RCLCPP_ERROR(get_logger(), "Goal refused: STOP/fault latch requires ~/reset; a goal cannot unlock it");
      return;
    }
    if (have_goal_ || goal_request_pending_ || navigation_handle_ || arm_request_outstanding_) {
      RCLCPP_WARN(get_logger(), "Goal refused: current goal/cancellation is still active");
      return;
    }
    const auto &p = pose->pose.position;
    const auto &q = pose->pose.orientation;
    const double norm = std::hypot(q.z, q.w);
    // Like native Nav2, an explicit map goal is a destination, not a velocity
    // freshness signal. Volatile ingress never restores a goal on restart.
    if (pose->header.frame_id != "map" ||
        !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
        !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(norm) ||
        norm < 1e-6 || std::abs(p.z) > 0.01 || std::abs(q.x) > 0.001 || std::abs(q.y) > 0.001) {
      RCLCPP_ERROR(get_logger(), "Goal refused: require a finite planar pose in map");
      return;
    }
    if (!navigation_health() || !fully_disarmed() || !navigation_client_->action_server_is_ready()) {
      RCLCPP_ERROR(get_logger(), "Goal refused: require fresh scan/localization, healthy disarmed bridge and Nav2");
      return;
    }
    saved_goal_ = *pose;
    saved_goal_.pose.orientation.z /= norm;
    saved_goal_.pose.orientation.w /= norm;
    have_goal_ = true;
    RCLCPP_INFO(get_logger(), "Explicit goal accepted: x=%.6f y=%.6f yaw=%.6f",
                p.x, p.y, 2.0 * std::atan2(q.z, q.w));
    send_navigation_goal();
  }

  void cancel_navigation() {
    if (!navigation_handle_ || cancel_requested_) {
      return;
    }
    cancel_requested_ = true;
    cancel_sent_at_ = std::chrono::steady_clock::now();
    const auto handle = navigation_handle_;
    try {
      navigation_client_->async_cancel_goal(handle, [this, handle](auto response) {
        if (navigation_handle_ == handle && response->return_code != 0 &&
            navigation_phase_ != NavigationPhase::kLocked) {
          latch_stopped("Nav2 cancellation rejected; manual reset required");
        }
      });
    } catch (const std::exception &error) {
      have_goal_ = false;
      navigation_phase_ = NavigationPhase::kLocked;
      RCLCPP_ERROR(get_logger(), "Nav2 cancellation failed; remaining locked: %s", error.what());
    }
  }

  void send_navigation_goal() {
    if (!have_goal_ || goal_request_pending_ || navigation_handle_ || !fully_disarmed()) {
      return;
    }
    navigation_phase_ = NavigationPhase::kActive;
    have_input_command_ = false;
    have_navigation_feedback_ = false;
    cancel_requested_ = false;
    goal_request_pending_ = true;
    navigation_sent_at_ = Clock::now();
    navigation_request_stamp_ = now().nanoseconds();
    controller_active_ = false;
    controller_stamp_ = 0;
    Navigate::Goal goal;
    goal.pose = saved_goal_;
    goal.pose.header.stamp = now();
    rclcpp_action::Client<Navigate>::SendGoalOptions options;
    options.goal_response_callback = [this](NavigationHandle::SharedPtr handle) {
      goal_request_pending_ = false;
      navigation_handle_ = handle;
      have_input_command_ = false;  // Never arm from a previous execution's command.
      if (!handle) {
        latch_stopped("Nav2 rejected goal");
      } else if (navigation_phase_ != NavigationPhase::kActive) {
        cancel_navigation();  // A true fault/STOP may precede acknowledgement.
      }
    };
    options.feedback_callback = [this](NavigationHandle::SharedPtr handle,
                                      const std::shared_ptr<const Navigate::Feedback>) {
      if (handle == navigation_handle_ &&
          navigation_phase_ != NavigationPhase::kLocked) {
        have_navigation_feedback_ = true;
        last_navigation_feedback_ = std::chrono::steady_clock::now();
      }
    };
    options.result_callback = [this](const NavigationHandle::WrappedResult &result) {
      if (!navigation_handle_ || result.goal_id != navigation_handle_->get_goal_id()) {
        return;
      }
      controller_active_ = false;
      navigation_handle_.reset();
      cancel_requested_ = false;
      publish_zero_twist();
      start_pending_ = false;
      have_input_command_ = false;
      request_disarm();
      clear_visualized_plan();
      if (navigation_phase_ == NavigationPhase::kLocked) {
        return;
      }
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        have_goal_ = false;
        navigation_phase_ = NavigationPhase::kIdle;
        transition(State::kStopped, "GOAL_REACHED; authority withdrawn; waiting for next explicit goal");
        try {
          const auto tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
          const auto &r = tf.transform.rotation;
          const double yaw = 2.0 * std::atan2(r.z, r.w);
          const auto &q = saved_goal_.pose.orientation;
          RCLCPP_INFO(get_logger(), "Goal result SUCCEEDED: position_error=%.4f m yaw_error=%.4f rad",
              std::hypot(tf.transform.translation.x - saved_goal_.pose.position.x,
                         tf.transform.translation.y - saved_goal_.pose.position.y),
              std::abs(angular_difference(yaw, 2.0 * std::atan2(q.z, q.w))));
        } catch (const tf2::TransformException &) {
          RCLCPP_WARN(get_logger(), "Goal SUCCEEDED; final TF error unavailable");
        }
      } else {
        latch_stopped("navigation aborted/canceled; goal discarded; manual reset required");
      }
    };
    transition(State::kStopped, "PLANNING; waiting for accepted action and fresh controller command");
    try {
      navigation_client_->async_send_goal(goal, options);
    } catch (const std::exception &error) {
      goal_request_pending_ = false;
      latch_stopped(std::string("Nav2 goal request failed: ") + error.what());
    }
  }

  void on_reset(const std_srvs::srv::Trigger::Request::SharedPtr,
                std_srvs::srv::Trigger::Response::SharedPtr response) {
    response->success = false;
    if (navigation_phase_ != NavigationPhase::kLocked || !navigation_health() ||
        !fully_disarmed() || navigation_handle_ || goal_request_pending_ || arm_request_outstanding_) {
      response->message = "reset refused: require a fault latch, restored health, confirmed disarm and finished requests";
      return;
    }
    if (!latch_file_.empty()) {
      std::error_code error;
      std::filesystem::remove(latch_file_, error);
      if (error) {
        response->message = "reset refused: cannot clear saved STOP/fault latch";
        return;
      }
    }
    have_goal_ = false;
    have_input_command_ = false;
    navigation_phase_ = NavigationPhase::kIdle;
    transition(State::kStopped, "IDLE; manual reset acknowledged; new goal required, no motion");
    response->success = true;
    response->message = "reset only; no motion; submit a NEW goal";
  }

  void begin_arm() {
    start_pending_ = true;
    arm_request_outstanding_ = true;
    arm_sent_at_ = std::chrono::steady_clock::now();
    arm_response_received_ = false;
    arm_response_success_ = false;
    publish_zero_twist();
    transition(State::kStopped,
               "authorized goal/START; waiting for authority acknowledgement");
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    arm_client_->async_send_request(
        request,
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
          arm_request_outstanding_ = false;
          if (!start_pending_) {
            request_disarm();
            return;
          }
          const auto result = future.get();
          arm_response_received_ = true;
          arm_response_success_ = result->success;
          arm_response_message_ = result->message;
        });
  }

  void clear_visualized_plan() {
    if (!plan_clear_publisher_) {
      return;
    }
    nav_msgs::msg::Path empty;
    empty.header.stamp = now();
    empty.header.frame_id = "map";
    plan_clear_publisher_->publish(empty);
  }

  void request_disarm() {
    last_disarm_attempt_ = std::chrono::steady_clock::now();
    if (!disarm_client_->service_is_ready()) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                            "Bridge disarm service is unavailable");
      return;
    }
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    disarm_client_->async_send_request(
        request,
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
          const auto result = future.get();
          if (!result->success) {
            RCLCPP_ERROR(get_logger(), "Bridge disarm reported: %s",
                         result->message.c_str());
          }
        });
  }

  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    // Sensor health only: no sector, clearance, obstacle threshold, or free-space decision.
    have_scan_ = !scan->ranges.empty() && std::isfinite(scan->angle_min) &&
        std::isfinite(scan->angle_increment) && scan->angle_increment != 0 &&
        std::isfinite(scan->range_min) && std::isfinite(scan->range_max) &&
        scan->range_min >= 0 && scan->range_max > scan->range_min &&
        std::any_of(scan->ranges.begin(), scan->ranges.end(), [&scan](float range) {
          return (std::isinf(range) && range > 0) ||
              (std::isfinite(range) && range >= scan->range_min && range <= scan->range_max);
        });
    last_scan_received_ = Clock::now();
    last_scan_source_stamp_ = scan->header.stamp;
    if (!have_scan_) { latch_stopped("invalid LaserScan metadata or no valid samples"); }
  }

  void on_controller_status(const action_msgs::msg::GoalStatusArray &array) {
    if (!have_goal_ || navigation_phase_ != NavigationPhase::kActive) { return; }
    const action_msgs::msg::GoalStatus *latest = nullptr;
    bool executing = false;
    for (const auto &status : array.status_list) {
      const auto stamp = stamp_nanoseconds(status.goal_info.stamp);
      if (stamp >= navigation_request_stamp_ &&
          status.status == action_msgs::msg::GoalStatus::STATUS_EXECUTING) {
        executing = true;
      }
      if (stamp >= navigation_request_stamp_ &&
          (!latest || stamp > stamp_nanoseconds(latest->goal_info.stamp))) { latest = &status; }
    }
    // Missing status is never treated as a normal stop. Active command watchdog stays in force.
    if (!latest || stamp_nanoseconds(latest->goal_info.stamp) < controller_stamp_) { return; }
    // Humble preempts FollowPath on each path update: the replacement executes
    // before the old handle terminates. UUID replacement is not control inactivity.
    // Keep the existing command deadline; status updates never refresh it.
    if (controller_active_ && !executing) {
      withdraw_controller();
    }
    controller_stamp_ = stamp_nanoseconds(latest->goal_info.stamp);
    if (executing && !controller_active_) {
      controller_started_at_ = Clock::now();
      have_input_command_ = false;
    }
    controller_active_ = executing;
  }

  void withdraw_controller() {
    publish_zero_twist();
    start_pending_ = false;
    have_input_command_ = false;
    controller_active_ = false;
    withdrawal_stamp_ = now().nanoseconds();
    withdrawal_started_at_ = Clock::now();
    request_disarm();
    transition(State::kStopped, "Nav2 control inactive; authority withdrawn; original goal retained");
  }

  void on_input_command(const geometry_msgs::msg::Twist::SharedPtr command) {
    const std::array<double, 6> values{command->linear.x, command->linear.y, command->linear.z,
        command->angular.x, command->angular.y, command->angular.z};
    if (!std::all_of(values.begin(), values.end(), [](double v) { return std::isfinite(v); }) ||
        std::abs(command->linear.y) > 1e-9 || std::abs(command->linear.z) > 1e-9 ||
        std::abs(command->angular.x) > 1e-9 || std::abs(command->angular.y) > 1e-9 ||
        command->linear.x < -1e-9) {
      latch_stopped("invalid/unsupported navigation Twist");
      return;
    }
    if (!controller_active_ || !have_goal_ || navigation_phase_ != NavigationPhase::kActive) { return; }
    input_command_ = *command;
    input_command_.linear.x = std::clamp(command->linear.x, 0.0, max_linear_speed_mps_);
    input_command_.angular.z = std::clamp(command->angular.z, -max_angular_speed_radps_, max_angular_speed_radps_);
    have_input_command_ = true;
    last_input_command_received_ = Clock::now();
  }

  void publish_zero_twist() { command_publisher_->publish(geometry_msgs::msg::Twist{}); }

  void latch_stopped(const std::string &reason) {
    start_pending_ = false;
    controller_active_ = false;
    have_input_command_ = false;
    have_goal_ = false;
    publish_zero_twist();
    request_disarm();
    clear_visualized_plan();
    navigation_phase_ = NavigationPhase::kLocked;
    transition(State::kStopped, reason + "; STOPPED latched");
    cancel_navigation();
    // Persist only host STOP/fault state. Never save or replay a navigation goal.
    if (!latch_file_.empty()) {
      std::ofstream latch(latch_file_, std::ios::trunc);
      if (!(latch << reason << '\n')) {
        RCLCPP_ERROR(get_logger(), "Cannot persist STOP/fault latch; remain locked");
      }
    }
  }

  void control_tick() {
    if (!startup_disarm_requested_ && disarm_client_->service_is_ready()) {
      startup_disarm_requested_ = true;
      request_disarm();
    }
    if (navigation_phase_ == NavigationPhase::kActive) {
      if (!navigation_health()) {
        latch_stopped("scan/localization/bridge health lost");
      } else if (goal_request_pending_ && age_ms(navigation_sent_at_) > 2000) {
        latch_stopped("Nav2 goal acknowledgement timeout");
      } else if (navigation_handle_ &&
          age_ms(have_navigation_feedback_ ? last_navigation_feedback_ : navigation_sent_at_) > 1000) {
        latch_stopped("navigation feedback timeout");
      } else if (controller_active_ &&
          age_ms(have_input_command_ ? last_input_command_received_ : controller_started_at_) >
              static_cast<uint32_t>(input_cmd_timeout_ms_)) {
        latch_stopped("active controller Twist timeout");
      } else if (withdrawal_stamp_ != 0 && !disarm_confirmed() && age_ms(withdrawal_started_at_) > 2000) {
        latch_stopped("authority withdrawal confirmation timeout");
      }
      if (navigation_phase_ == NavigationPhase::kActive && controller_active_ &&
          navigation_handle_ && have_navigation_feedback_ && input_command_fresh() &&
          state_ == State::kStopped && !start_pending_ && !arm_request_outstanding_ &&
          disarm_confirmed() && arm_client_->service_is_ready()) {
        withdrawal_stamp_ = 0;
        begin_arm();
      }
    }
    if (start_pending_) {
      if (age_ms(arm_sent_at_) > 2000) { latch_stopped("bridge authority acknowledgement timeout"); }
      else if (arm_response_received_ && !arm_response_success_) {
        latch_stopped("bridge arm rejected: " + arm_response_message_);
      } else if (arm_response_received_ && bridge_status_.authority_armed) {
        start_pending_ = false;
        transition(State::kRunning, "fresh active Nav2 controller; authority acknowledged");
      }
    } else if (state_ == State::kRunning && !bridge_status_.authority_armed) {
      latch_stopped("motion authority unexpectedly cleared");
    }
    if (state_ == State::kRunning) { command_publisher_->publish(input_command_); }
    else { publish_zero_twist(); }
    if (state_ == State::kStopped && !start_pending_ && have_bridge_status_ &&
        (bridge_status_.authority_armed || bridge_status_.arm_requested) && age_ms(last_disarm_attempt_) >= 100) {
      request_disarm();
    }
  }

  bool disarm_confirmed() const {
    return fully_disarmed() && (withdrawal_stamp_ == 0 ||
        stamp_nanoseconds(bridge_status_.last_disarm_tx_stamp) >= withdrawal_stamp_);
  }
  int localization_timeout_ms_{700}, scan_timeout_ms_{400}, bridge_timeout_ms_{300}, input_cmd_timeout_ms_{250};
  double max_linear_speed_mps_{0.20}, max_angular_speed_radps_{1.50};
  State state_{State::kStopped};
  NavigationPhase navigation_phase_{NavigationPhase::kIdle};
  std::string reason_{"IDLE; waiting for a new explicit map goal"}, arm_response_message_;
  bool have_goal_{false}, goal_request_pending_{false}, cancel_requested_{false};
  bool have_navigation_feedback_{false}, arm_request_outstanding_{false}, controller_active_{false};
  bool have_scan_{false}, have_bridge_status_{false}, startup_disarm_requested_{false};
  bool start_pending_{false}, have_input_command_{false}, arm_response_received_{false}, arm_response_success_{false};
  std::string latch_file_;
  int64_t navigation_request_stamp_{0}, controller_stamp_{0}, withdrawal_stamp_{0};
  geometry_msgs::msg::PoseStamped saved_goal_;
  geometry_msgs::msg::Twist input_command_;
  msg::BridgeStatus bridge_status_;
  builtin_interfaces::msg::Time last_scan_source_stamp_;
  Clock::time_point navigation_sent_at_{}, cancel_sent_at_{}, last_navigation_feedback_{}, arm_sent_at_{};
  Clock::time_point last_scan_received_{}, last_bridge_status_received_{}, last_input_command_received_{};
  Clock::time_point last_disarm_attempt_{}, controller_started_at_{}, withdrawal_started_at_{};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp_action::Client<Navigate>::SharedPtr navigation_client_;
  NavigationHandle::SharedPtr navigation_handle_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_clear_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr input_command_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Subscription<msg::BridgeStatus>::SharedPtr bridge_subscription_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr controller_status_subscription_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr arm_client_, disarm_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_, reset_service_;
  rclcpp::TimerBase::SharedPtr control_timer_, status_timer_;
};
} // namespace robot_navigation
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_navigation::NavigationMotionGate>());
  rclcpp::shutdown();
  return 0;
}
