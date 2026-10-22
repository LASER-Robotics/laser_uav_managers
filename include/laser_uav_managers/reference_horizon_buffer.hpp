#ifndef LASER_UAV_MANAGERS__REFERENCE_HORIZON_BUFFER_HPP_
#define LASER_UAV_MANAGERS__REFERENCE_HORIZON_BUFFER_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include <rclcpp/time.hpp>
#include <laser_uav_multi_drones/msg/reference_horizon.hpp>

namespace laser_uav_managers
{

// Validates and stores external references independently of control selection.
// Failed messages preserve the accepted snapshot and never refresh its heartbeat.
class ReferenceHorizonBuffer
{
public:
  using Horizon = laser_uav_multi_drones::msg::ReferenceHorizon;
  using Clock = std::chrono::steady_clock;

  ReferenceHorizonBuffer(std::size_t steps, double dt, double maximum_age,
    double future_tolerance)
  : steps_(steps), dt_(dt), maximum_age_(maximum_age), future_tolerance_(future_tolerance)
  {
    if (steps == 0 || steps > 1000 || !std::isfinite(dt) || dt <= 0. ||
      !std::isfinite(maximum_age) || maximum_age <= 0. ||
      !std::isfinite(future_tolerance) || future_tolerance < 0.)
    {
      throw std::invalid_argument("Reference horizon requires 1..1000 steps, positive dt/maximum_age and nonnegative future_tolerance");
    }
  }

  bool store(const Horizon & message, const std::string & odometry_frame,
    const rclcpp::Time & ros_now, Clock::time_point received, std::string & reason)
  {
    if (message.references.size() != steps_ + 1) {
      reason = "Reference horizon must contain exactly NMPC N + 1 states";
      return false;
    }
    if (!std::isfinite(message.dt) || message.dt <= 0. ||
      std::abs(message.dt - dt_) > 1.e-9 * std::max(1., std::abs(dt_)))
    {
      reason = "Reference horizon dt differs from NMPC dt";
      return false;
    }
    if (!checkFrame(message, odometry_frame, reason) ||
      !checkStamp(message, ros_now, reason))
    {
      return false;
    }
    for (const auto & reference : message.references) {
      if (!reference.use_position || !reference.use_linear_velocity ||
        reference.use_individual_thrust)
      {
        reason = "PPO references require position/linear velocity and no individual thrust commands";
        return false;
      }
      const auto & p = reference.pose.position;
      const auto & v = reference.twist.linear;
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
        !std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z))
      {
        reason = "Reference horizon contains a non-finite position or linear velocity";
        return false;
      }
      if (reference.use_orientation) {
        const auto & q = reference.pose.orientation;
        const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
        if (!std::isfinite(norm) || std::abs(norm - 1.) > 1.e-3) {
          reason = "Enabled orientation reference requires a finite unit quaternion";
          return false;
        }
      }
      if (reference.use_angular_velocity) {
        const auto & w = reference.twist.angular;
        if (!std::isfinite(w.x) || !std::isfinite(w.y) || !std::isfinite(w.z)) {
          reason = "Reference horizon contains a non-finite angular velocity";
          return false;
        }
      }
    }
    std::lock_guard<std::mutex> lock(mutex_);
    // Equal stamps are normal while /clock is paused. A ROS clock rewind or
    // an expired reception permits a producer to restart its timeline.
    if (horizon_ && ros_now >= received_ros_time_ &&
      std::chrono::duration<double>(received - received_time_).count() <= maximum_age_ &&
      rclcpp::Time(message.header.stamp, ros_now.get_clock_type()) <
      rclcpp::Time(horizon_->header.stamp, ros_now.get_clock_type()))
    {
      reason = "Reference horizon is older than the last accepted message";
      return false;
    }
    horizon_ = std::make_shared<const Horizon>(message);
    received_time_ = received;
    received_ros_time_ = ros_now;
    reason = "External reference horizon accepted and stored";
    return true;
  }

  Horizon::ConstSharedPtr getFresh(const std::string & odometry_frame,
    const rclcpp::Time & ros_now, Clock::time_point wall_now, std::string & reason) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!horizon_) {
      reason = "No external reference horizon has been accepted";
      return nullptr;
    }
    if (!checkFrame(*horizon_, odometry_frame, reason) ||
      !checkStamp(*horizon_, ros_now, reason))
    {
      return nullptr;
    }
    const double age = std::chrono::duration<double>(wall_now - received_time_).count();
    if (age < 0. || age > maximum_age_) {
      reason = "External reference horizon reception expired";
      return nullptr;
    }
    reason = "External reference horizon is valid";
    return horizon_;
  }

  void clear()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    horizon_.reset();
  }

private:
  bool checkFrame(const Horizon & message, const std::string & odometry_frame,
    std::string & reason) const
  {
    if (odometry_frame.empty() || message.header.frame_id.empty() ||
      message.header.frame_id != odometry_frame)
    {
      reason = "Reference horizon frame must match the nonempty odometry frame";
      return false;
    }
    return true;
  }

  bool checkStamp(const Horizon & message, const rclcpp::Time & ros_now,
    std::string & reason) const
  {
    if (message.header.stamp.sec < 0 || message.header.stamp.nanosec >= 1000000000U) {
      reason = "Reference horizon timestamp is invalid";
      return false;
    }
    const double age = (ros_now - rclcpp::Time(message.header.stamp,
      ros_now.get_clock_type())).seconds();
    if (age > maximum_age_) {
      reason = "External reference horizon ROS timestamp expired";
      return false;
    }
    if (age < -future_tolerance_) {
      reason = "External reference horizon timestamp is in the future";
      return false;
    }
    return true;
  }

  const std::size_t steps_;
  const double dt_;
  const double maximum_age_;
  const double future_tolerance_;
  mutable std::mutex mutex_;
  Horizon::ConstSharedPtr horizon_;
  Clock::time_point received_time_{};
  rclcpp::Time received_ros_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace laser_uav_managers

#endif  // LASER_UAV_MANAGERS__REFERENCE_HORIZON_BUFFER_HPP_
