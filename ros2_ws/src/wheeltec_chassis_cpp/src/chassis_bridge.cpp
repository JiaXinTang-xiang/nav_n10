#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <memory>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <unistd.h>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/int64_multi_array.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "wheeltec_chassis_cpp/gyro_yaw.hpp"

namespace wheeltec_chassis_cpp
{

using geometry_msgs::msg::Twist;
using namespace std::chrono_literals;

constexpr uint8_t kCommandHeader = 0xBB;
constexpr uint8_t kOdomHeader = 0xCC;
constexpr uint8_t kOdomV2Header = 0xCD;
constexpr uint8_t kFooter = 0x55;

double normalize_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

template<typename T>
T read_value(const std::array<uint8_t, 64> &buffer, std::size_t offset)
{
  T value{};
  std::memcpy(&value, buffer.data() + offset, sizeof(T));
  return value;
}

class ChassisBridge final : public rclcpp::Node
{
public:
  ChassisBridge()
  : Node("chassis_bridge"),
    rx_buffer_{},
    fd_(-1),
    rx_index_(0),
    expected_length_(0),
    receiving_(false),
    odom_x_(0.0),
    odom_y_(0.0),
    odom_theta_(0.0),
    have_wheel_sample_(false),
    last_sequence_(0),
    have_sequence_(false),
    last_tick_(0),
    last_left_count_(0),
    last_right_count_(0),
    imu_odom_offset_(0.0),
    have_imu_offset_(false),
    target_linear_(0.0),
    target_angular_(0.0),
    have_command_(false),
    last_command_time_(0, 0, RCL_ROS_TIME)
  {
    serial_port_ = declare_parameter<std::string>("serial_port", "/dev/chassis");
    serial_baud_ = declare_parameter<int>("serial_baud", 115200);
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    serial_poll_rate_hz_ = declare_parameter<double>("serial_poll_rate_hz", 200.0);
    command_rate_hz_ = declare_parameter<double>("command_rate_hz", 50.0);
    command_timeout_sec_ = declare_parameter<double>("command_timeout_sec", 0.5);
    left_meters_per_count_ = declare_parameter<double>("left_meters_per_count", 0.0002613638);
    right_meters_per_count_ = declare_parameter<double>("right_meters_per_count", 0.0002611298);
    wheel_separation_ = declare_parameter<double>("wheel_separation", 0.1431);
    integrate_from_counts_ = declare_parameter<bool>("integrate_odom_from_wheel_counts", true);
    use_imu_yaw_ = declare_parameter<bool>("use_imu_yaw", true);
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu/data");
    imu_yaw_sign_ = declare_parameter<double>("imu_yaw_sign", 1.0);
    imu_timeout_sec_ = declare_parameter<double>("imu_timeout_sec", 0.2);
    imu_max_delta_rad_ = declare_parameter<double>("imu_max_delta_rad", 0.25);
    imu_force_gyro_yaw_ = declare_parameter<bool>("imu_force_gyro_yaw", true);
    imu_gyro_deadband_ = declare_parameter<double>("imu_gyro_deadband_rad_s", 0.03);

    if (!imu_force_gyro_yaw_) {
      throw std::invalid_argument("Only timestamped gyro yaw is supported; set imu_force_gyro_yaw=true");
    }

    for (const double value : {serial_poll_rate_hz_, command_rate_hz_, command_timeout_sec_,
        left_meters_per_count_, right_meters_per_count_, wheel_separation_,
        imu_timeout_sec_, imu_max_delta_rad_, imu_yaw_sign_, imu_gyro_deadband_})
    {
      if (!std::isfinite(value)) {
        throw std::invalid_argument("non-finite chassis bridge parameter");
      }
    }
    if (serial_poll_rate_hz_ <= 0.0 || command_rate_hz_ <= 0.0 ||
      command_timeout_sec_ <= 0.0 || left_meters_per_count_ <= 0.0 ||
      right_meters_per_count_ <= 0.0 || wheel_separation_ <= 0.0 ||
      imu_timeout_sec_ <= 0.0 || imu_max_delta_rad_ <= 0.0 ||
      std::abs(imu_yaw_sign_) != 1.0 || imu_gyro_deadband_ < 0.0)
    {
      throw std::invalid_argument("invalid chassis bridge parameter");
    }

    open_serial();

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", rclcpp::QoS(20));
    wheel_counts_pub_ = create_publisher<std_msgs::msg::Int64MultiArray>(
      "/wheel_counts", rclcpp::QoS(20));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    cmd_sub_ = create_subscription<Twist>(
      "/cmd_vel", rclcpp::QoS(10),
      std::bind(&ChassisBridge::command_callback, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ChassisBridge::imu_callback, this, std::placeholders::_1));

    read_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / serial_poll_rate_hz_)),
      std::bind(&ChassisBridge::read_serial, this));
    command_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / command_rate_hz_)),
      std::bind(&ChassisBridge::send_current_command, this));

    RCLCPP_INFO(
      get_logger(), "C++底盘桥接已启动: %s @ %d bps; 串口轮询 %.0f Hz; 指令 %.0f Hz",
      serial_port_.c_str(), serial_baud_, serial_poll_rate_hz_, command_rate_hz_);
  }

  ~ChassisBridge() override
  {
    send_velocity(0.0, 0.0);
    if (fd_ >= 0) {
      close(fd_);
    }
  }

private:
  void open_serial()
  {
    fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      throw std::runtime_error("cannot open serial port " + serial_port_ + ": " + std::strerror(errno));
    }
    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
      throw std::runtime_error("cannot read serial settings");
    }
    cfmakeraw(&tty);
    speed_t speed = B115200;
    if (serial_baud_ != 115200) {
      throw std::runtime_error("unsupported serial baud; only 115200 is supported");
    }
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;
    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
      throw std::runtime_error("cannot configure serial port");
    }
  }

  void read_serial()
  {
    std::array<uint8_t, 512> bytes{};
    const ssize_t count = read(fd_, bytes.data(), bytes.size());
    if (count <= 0) {
      return;
    }
    for (ssize_t index = 0; index < count; ++index) {
      parse_byte(bytes[static_cast<std::size_t>(index)]);
    }
  }

  void parse_byte(uint8_t byte)
  {
    if (!receiving_) {
      if (byte == kOdomHeader || byte == kOdomV2Header) {
        rx_buffer_[0] = byte;
        rx_index_ = 1;
        expected_length_ = byte == kOdomHeader ? 15 : 30;
        receiving_ = true;
      }
      return;
    }

    rx_buffer_[rx_index_++] = byte;
    if (rx_index_ < expected_length_) {
      return;
    }
    receiving_ = false;
    if (rx_buffer_[expected_length_ - 1] != kFooter) {
      return;
    }
    const std::size_t payload_end = expected_length_ == 15 ? 13 : 28;
    uint8_t checksum = 0;
    for (std::size_t index = 1; index < payload_end; ++index) {
      checksum = static_cast<uint8_t>(checksum + rx_buffer_[index]);
    }
    if (checksum != rx_buffer_[payload_end]) {
      return;
    }
    decode_odom(rx_buffer_[0]);
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    const auto generation = gyro_yaw_.generation();
    gyro_yaw_.update(
      static_cast<int64_t>(msg->header.stamp.sec) * 1000000000LL + msg->header.stamp.nanosec,
      get_clock()->now().nanoseconds(),
      msg->angular_velocity.z, imu_yaw_sign_, imu_gyro_deadband_,
      imu_timeout_sec_, imu_max_delta_rad_);
    if (generation != gyro_yaw_.generation()) {
      have_imu_offset_ = false;
    }
  }

  void decode_odom(uint8_t header)
  {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    double linear_velocity = 0.0;
    double angular_velocity = 0.0;

    if (header == kOdomHeader) {
      x = read_value<float>(rx_buffer_, 1);
      y = read_value<float>(rx_buffer_, 5);
      theta = read_value<float>(rx_buffer_, 9);
      odom_x_ = x / 1000.0;
      odom_y_ = y / 1000.0;
      odom_theta_ = theta;
    } else {
      const uint8_t version = rx_buffer_[1];
      if (version != 1) {
        return;
      }
      const uint16_t sequence = read_value<uint16_t>(rx_buffer_, 2);
      const uint32_t tick = read_value<uint32_t>(rx_buffer_, 4);
      x = read_value<float>(rx_buffer_, 8);
      y = read_value<float>(rx_buffer_, 12);
      theta = read_value<float>(rx_buffer_, 16);
      const int32_t left_count = read_value<int32_t>(rx_buffer_, 20);
      const int32_t right_count = read_value<int32_t>(rx_buffer_, 24);

      std_msgs::msg::Int64MultiArray counts;
      counts.data = {
        static_cast<int64_t>(sequence), static_cast<int64_t>(tick),
        static_cast<int64_t>(left_count), static_cast<int64_t>(right_count)};
      wheel_counts_pub_->publish(counts);

      if (have_wheel_sample_) {
        const uint32_t tick_delta = tick - last_tick_;
        const double dt = static_cast<double>(tick_delta) * 0.0001;
        if (dt >= 0.001 && dt <= 0.2) {
          const double dl = static_cast<double>(left_count - last_left_count_) * left_meters_per_count_;
          const double dr = static_cast<double>(right_count - last_right_count_) * right_meters_per_count_;
          const double ds = 0.5 * (dl + dr);
          const double wheel_dtheta = (dr - dl) / wheel_separation_;
          double dtheta = wheel_dtheta;
          const rclcpp::Time now = get_clock()->now();
          const bool imu_fresh = use_imu_yaw_ &&
            gyro_yaw_.fresh(now.nanoseconds(), imu_timeout_sec_);
          if (imu_fresh) {
            if (!have_imu_offset_) {
              imu_odom_offset_ = odom_theta_ - gyro_yaw_.yaw();
              have_imu_offset_ = true;
            }
            const double imu_theta = normalize_angle(gyro_yaw_.yaw() + imu_odom_offset_);
            dtheta = normalize_angle(imu_theta - odom_theta_);
          } else {
            have_imu_offset_ = false;
          }
          linear_velocity = ds / dt;
          angular_velocity = dtheta / dt;
          if (integrate_from_counts_) {
            const double theta_mid = odom_theta_ + 0.5 * dtheta;
            odom_x_ += ds * std::cos(theta_mid);
            odom_y_ += ds * std::sin(theta_mid);
            odom_theta_ = normalize_angle(odom_theta_ + dtheta);
          }
        }
      } else {
        odom_x_ = x / 1000.0;
        odom_y_ = y / 1000.0;
        odom_theta_ = theta;
      }
      if (!integrate_from_counts_) {
        odom_x_ = x / 1000.0;
        odom_y_ = y / 1000.0;
        odom_theta_ = theta;
      }
      last_sequence_ = sequence;
      have_sequence_ = true;
      last_tick_ = tick;
      last_left_count_ = left_count;
      last_right_count_ = right_count;
      have_wheel_sample_ = true;
    }

    const rclcpp::Time stamp = get_clock()->now();
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = odom_x_;
    odom.pose.pose.position.y = odom_y_;
    odom.pose.pose.orientation.z = std::sin(odom_theta_ * 0.5);
    odom.pose.pose.orientation.w = std::cos(odom_theta_ * 0.5);
    odom.twist.twist.linear.x = linear_velocity;
    odom.twist.twist.angular.z = angular_velocity;
    odom.pose.covariance[0] = 0.01;
    odom.pose.covariance[7] = 0.01;
    odom.pose.covariance[35] = 0.01;
    odom.twist.covariance[0] = 0.1;
    odom.twist.covariance[7] = 0.1;
    odom.twist.covariance[35] = 0.1;
    odom_pub_->publish(odom);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = odom.header;
      transform.child_frame_id = base_frame_;
      transform.transform.translation.x = odom_x_;
      transform.transform.translation.y = odom_y_;
      transform.transform.rotation = odom.pose.pose.orientation;
      tf_broadcaster_->sendTransform(transform);
    }
  }

  void command_callback(const Twist::SharedPtr msg)
  {
    target_linear_ = msg->linear.x;
    target_angular_ = msg->angular.z;
    last_command_time_ = get_clock()->now();
    have_command_ = true;
  }

  void send_current_command()
  {
    double linear = target_linear_;
    double angular = target_angular_;
    if (!have_command_ || (get_clock()->now() - last_command_time_).seconds() > command_timeout_sec_) {
      linear = 0.0;
      angular = 0.0;
    }
    send_velocity(linear, angular);
  }

  void send_velocity(double linear, double angular)
  {
    const int16_t linear_mm = static_cast<int16_t>(std::clamp(linear * 1000.0, -32767.0, 32767.0));
    const int16_t angular_mrad = static_cast<int16_t>(std::clamp(angular * 1000.0, -32767.0, 32767.0));
    std::array<uint8_t, 7> frame{};
    frame[0] = kCommandHeader;
    frame[1] = static_cast<uint8_t>((static_cast<uint16_t>(linear_mm) >> 8) & 0xFF);
    frame[2] = static_cast<uint8_t>(static_cast<uint16_t>(linear_mm) & 0xFF);
    frame[3] = static_cast<uint8_t>((static_cast<uint16_t>(angular_mrad) >> 8) & 0xFF);
    frame[4] = static_cast<uint8_t>(static_cast<uint16_t>(angular_mrad) & 0xFF);
    frame[5] = static_cast<uint8_t>(frame[1] + frame[2] + frame[3] + frame[4]);
    frame[6] = kFooter;
    if (fd_ >= 0) {
      (void)write(fd_, frame.data(), frame.size());
    }
  }

  std::string serial_port_;
  int serial_baud_;
  std::string odom_frame_;
  std::string base_frame_;
  bool publish_tf_;
  double serial_poll_rate_hz_;
  double command_rate_hz_;
  double command_timeout_sec_;
  double left_meters_per_count_;
  double right_meters_per_count_;
  double wheel_separation_;
  bool integrate_from_counts_;
  bool use_imu_yaw_;
  std::string imu_topic_;
  double imu_yaw_sign_;
  double imu_timeout_sec_;
  double imu_max_delta_rad_;
  bool imu_force_gyro_yaw_;
  double imu_gyro_deadband_;

  std::array<uint8_t, 64> rx_buffer_;
  int fd_;
  std::size_t rx_index_;
  std::size_t expected_length_;
  bool receiving_;

  double odom_x_;
  double odom_y_;
  double odom_theta_;
  bool have_wheel_sample_;
  uint16_t last_sequence_;
  bool have_sequence_;
  uint32_t last_tick_;
  int32_t last_left_count_;
  int32_t last_right_count_;

  GyroYaw gyro_yaw_;
  double imu_odom_offset_;
  bool have_imu_offset_;

  double target_linear_;
  double target_angular_;
  bool have_command_;
  rclcpp::Time last_command_time_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr wheel_counts_pub_;
  rclcpp::Subscription<Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr read_timer_;
  rclcpp::TimerBase::SharedPtr command_timer_;
};

}  // namespace wheeltec_chassis_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<wheeltec_chassis_cpp::ChassisBridge>());
  } catch (const std::exception & error) {
    fprintf(stderr, "chassis_bridge_cpp: %s\n", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
