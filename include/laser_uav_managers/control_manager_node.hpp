#ifndef LASER_UAV_MANAGERS__CONTROL_MANAGER_NODE_HPP
#define LASER_UAV_MANAGERS__CONTROL_MANAGER_NODE_HPP

#include <Eigen/Dense>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <laser_msgs/msg/uav_control_diagnostics.hpp>
#include <laser_msgs/msg/reference_state.hpp>
#include <laser_msgs/msg/api_px4_diagnostics.hpp>
#include <laser_msgs/msg/attitude_rates_and_thrust.hpp>
#include <laser_msgs/msg/trajectory_path.hpp>
#include <laser_msgs/msg/motor_speed.hpp>

#include <laser_uav_lib/filter/irr_filter.hpp>

#include <laser_uav_planner/agile_planner.hpp>
#include <laser_uav_controllers/nmpc_controller.hpp>
#include <laser_uav_controllers/indi_controller.hpp>

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

  rclcpp::Subscription<sensor_msgs::msg::Imu>::ConstSharedPtr sub_imu_;
  void                                                        subImu(const sensor_msgs::msg::Imu &msg);

  rclcpp::Subscription<laser_msgs::msg::MotorSpeed>::ConstSharedPtr sub_motor_speed_;
  void                                                              subMotorSpeed(const laser_msgs::msg::MotorSpeed &msg);

  rclcpp::Subscription<geometry_msgs::msg::Pose>::ConstSharedPtr sub_goto_;
  void                                                           subGoto(const geometry_msgs::msg::Pose &msg);

  rclcpp::Subscription<laser_msgs::msg::TrajectoryPath>::ConstSharedPtr sub_trajectory_path_;
  void                                                                  subTrajectoryPath(const laser_msgs::msg::TrajectoryPath &msg);

  rclcpp::Subscription<laser_msgs::msg::ApiPx4Diagnostics>::ConstSharedPtr sub_api_diagnostics_;
  void                                                                     subApiDiagnostics(const laser_msgs::msg::ApiPx4Diagnostics &msg);

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_takeoff_;
  void srvTakeoff(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_land_;
  void srvLand(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  rclcpp_lifecycle::LifecyclePublisher<laser_msgs::msg::AttitudeRatesAndThrust>::SharedPtr pub_attitude_rates_and_thrust_reference_;
  double                                                                                   _rate_external_loop_control_;
  rclcpp::TimerBase::SharedPtr                                                             tmr_external_loop_control_;
  void                                                                                     tmrExternalLoopControl();

  rclcpp_lifecycle::LifecyclePublisher<laser_msgs::msg::MotorSpeed>::SharedPtr pub_motor_speed_reference_;
  double                                                                       _rate_internal_loop_control_;
  rclcpp::TimerBase::SharedPtr                                                 tmr_internal_loop_control_;
  void                                                                         tmrInternalLoopControl();

  rclcpp_lifecycle::LifecyclePublisher<laser_msgs::msg::UavControlDiagnostics>::SharedPtr pub_diagnostics_;
  double                                                                                  _rate_diagnostics_;
  rclcpp::TimerBase::SharedPtr                                                            tmr_diagnostics_;
  void                                                                                    tmrDiagnostics();

  laser_msgs::msg::UavControlDiagnostics       diagnostics_;
  nav_msgs::msg::Odometry                      odometry_;
  laser_msgs::msg::ReferenceState              last_waypoint_;
  std::vector<geometry_msgs::msg::Pose>        desired_path_;
  std::vector<laser_msgs::msg::ReferenceState> current_horizon_path_;

  laser_uav_planner::quadrotor_t  _planner_quadrotor_params_;
  laser_uav_planner::pmm_t        _pmm_params_;
  laser_uav_planner::AgilePlanner agile_planner_;

  laser_uav_controllers::quadrotor_t    _controller_quadrotor_params_;
  laser_uav_controllers::acados_t       _acados_params_;
  laser_uav_controllers::NmpcController nmpc_controller_;
  laser_uav_controllers::IndiController indi_controller_;

  std::vector<double>      _gyro_a_;
  std::vector<double>      _gyro_b_;
  laser_uav_lib::IIRFilter btw_gyro_x_;
  laser_uav_lib::IIRFilter btw_gyro_y_;
  laser_uav_lib::IIRFilter btw_gyro_z_;

  std::vector<double>                   _motor_a_;
  std::vector<double>                   _motor_b_;
  std::vector<laser_uav_lib::IIRFilter> btw_motors_;

  Eigen::VectorXd nmpc_control_input_;
  Eigen::VectorXd motor_speed_estimated_;
  Eigen::Vector3d last_angular_speed_;
  Eigen::Vector3d angular_acceleration_estimated_;

  int  lock_waypoint_;
  bool _agile_fly_;

  double _takeoff_height_;
  double _takeoff_speed_;

  bool angular_rates_and_thrust_mode_;
  bool lock_control_inputs_{true};
  bool have_nmpc_control_input_{false};
  bool requested_takeoff_{false};
  bool takeoff_done_{false};
  bool requested_land_{false};
  bool land_done_{true};
  bool is_active_{false};
};
}  // namespace laser_uav_managers

#endif
