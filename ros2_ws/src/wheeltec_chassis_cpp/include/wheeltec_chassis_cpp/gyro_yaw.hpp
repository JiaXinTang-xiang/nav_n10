#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>

namespace wheeltec_chassis_cpp
{

class GyroYaw
{
public:
  bool update(
    int64_t stamp_ns, int64_t now_ns, double gyro_z, double sign,
    double deadband, double timeout, double max_delta)
  {
    const double age = static_cast<double>(now_ns - stamp_ns) * 1e-9;
    if (stamp_ns <= 0 || !std::isfinite(gyro_z) || age < -0.02 || age > timeout) {
      invalidate();
      return false;
    }
    if (initialized_ && stamp_ns <= stamp_ns_) {
      return false;
    }
    const double rate = gyro_z * sign;
    const double dt = static_cast<double>(stamp_ns - stamp_ns_) * 1e-9;
    const double delta = std::abs(rate) >= deadband ? rate * dt : 0.0;
    const auto received = std::chrono::steady_clock::now();
    const double receive_gap = std::chrono::duration<double>(received - received_).count();
    const bool continuous = initialized_ && dt > 0.0 && dt <= timeout &&
      receive_gap <= timeout && std::abs(delta) <= max_delta;
    if (continuous) {
      yaw_ += delta;
    } else {
      ++generation_;
    }
    stamp_ns_ = stamp_ns;
    received_ = received;
    rate_ = rate;
    initialized_ = true;
    ready_ = continuous;
    return ready_;
  }

  bool fresh(int64_t now_ns, double timeout) const
  {
    const double age = static_cast<double>(now_ns - stamp_ns_) * 1e-9;
    return ready_ && age >= -0.02 && age <= timeout &&
      std::chrono::duration<double>(std::chrono::steady_clock::now() - received_).count()
      <= timeout;
  }

  void invalidate()
  {
    initialized_ = false;
    ready_ = false;
    ++generation_;
  }

  double yaw() const {return yaw_;}
  double rate() const {return rate_;}
  int64_t stamp() const {return stamp_ns_;}
  uint64_t generation() const {return generation_;}

private:
  double yaw_{0.0};
  double rate_{0.0};
  int64_t stamp_ns_{0};
  uint64_t generation_{0};
  bool initialized_{false};
  bool ready_{false};
  std::chrono::steady_clock::time_point received_{};
};

}
