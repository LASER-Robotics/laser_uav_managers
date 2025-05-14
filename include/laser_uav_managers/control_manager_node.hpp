#ifndef LASER_UAV_MANAGERS__CONTROL_MANAGER_HPP
#define LASER_UAV_MANAGERS__CONTROL_MANAGER_HPP

#include <Eigen/Dense>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <laser_msgs/msg/attitude_rates_and_thrust.hpp>

#include <laser_uav_controllers/nmpc_controller.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace laser_uav_managers
{
class ControlManagerNode : public rclcpp_lifecycle::LifecycleNode {
public:
  explicit ControlManagerNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  ~ControlManagerNode() override;

private:
  CallbackReturn on_configure(const rclcpp_lifecycle::State &);

  CallbackReturn on_activate(const rclcpp_lifecycle::State &state);

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state);

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state);

  rclcpp::CallbackGroup::SharedPtr callback_group_;

  void getParameters();
  void configPubSub();
  void configTimers();
  void configServices();
  void configClasses();

  rclcpp::Subscription<nav_msgs::msg::Odometry>::ConstSharedPtr sub_odometry_;
  void                                                          subOdometry(const nav_msgs::msg::Odometry &msg);

  rclcpp::Subscription<geometry_msgs::msg::Pose>::ConstSharedPtr sub_goto_;
  void                                                           subGoto(const geometry_msgs::msg::Pose &msg);

  rclcpp_lifecycle::LifecyclePublisher<laser_msgs::msg::AttitudeRatesAndThrust>::SharedPtr pub_attitude_rates_and_thrust_reference_;
  double                                                                                   _rate_core_control_;
  rclcpp::TimerBase::SharedPtr                                                             tmr_core_control_;
  void                                                                                     tmrCoreControl();

  double                       _rate_diagnostics_;
  rclcpp::TimerBase::SharedPtr tmr_diagnostics_;
  void                         tmrDiagnostics();

  nav_msgs::msg::Odometry  odometry_;
  geometry_msgs::msg::Pose current_reference_;

  laser_uav_controllers::quadrotor_t    _quadrotor_params_;
  laser_uav_controllers::acados_t       _acados_params_;
  laser_uav_controllers::NmpcController nmpc_controller_;

  bool is_active_{false};
};
}  // namespace laser_uav_managers

#endif
