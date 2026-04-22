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
#include <laser_msgs/msg/pose_with_heading.hpp>
#include <laser_msgs/msg/api_px4_diagnostics.hpp>
#include <laser_msgs/msg/attitude_rates_and_thrust.hpp>
#include <laser_msgs/msg/trajectory_path.hpp>
#include <laser_msgs/msg/motor_speed_stamped.hpp>
#include <laser_msgs/msg/motor_speed.hpp>

#include <laser_uav_lib/filter/irr_filter.hpp>
#include <laser_uav_lib/metrics/rmse.hpp>

#include <laser_uav_planners/agile_planner.hpp>
#include <laser_uav_controllers/nmpc_controller.hpp>
#include <laser_uav_controllers/indi_controller.hpp>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Transform.h"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace laser_uav_managers
{

/* safe_area_t //{ */
struct safe_area_t
{
  bool                enabled;
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> z;
};
//}

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

  void   getParameters();
  void   configPubSub();
  void   configTimers();
  void   configServices();
  void   configClasses();
  double euclideanDistance(geometry_msgs::msg::Point p1, geometry_msgs::msg::Point p2);
  double checkHeadingError();
  double normalizeHeading(double heading);
  double quaternionToHeading(geometry_msgs::msg::Quaternion &q);
  void   checkSafeArea();
  bool   estimateMass();

  rclcpp::Subscription<nav_msgs::msg::Odometry>::ConstSharedPtr sub_odometry_;
  void                                                          subOdometry(const nav_msgs::msg::Odometry &msg);

  rclcpp::Subscription<sensor_msgs::msg::Imu>::ConstSharedPtr sub_imu_;
  void                                                        subImu(const sensor_msgs::msg::Imu &msg);

  rclcpp::Subscription<laser_msgs::msg::MotorSpeedStamped>::ConstSharedPtr sub_motor_speed_;
  void                                                                     subMotorSpeedStamped(const laser_msgs::msg::MotorSpeedStamped &msg);

  rclcpp::Subscription<laser_msgs::msg::PoseWithHeading>::ConstSharedPtr sub_goto_;
  void                                                                   subGoto(const laser_msgs::msg::PoseWithHeading &msg);

  rclcpp::Subscription<laser_msgs::msg::PoseWithHeading>::ConstSharedPtr sub_goto_relative_;
  void                                                                   subGotoRelative(const laser_msgs::msg::PoseWithHeading &msg);

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

  laser_msgs::msg::UavControlDiagnostics        diagnostics_;
  nav_msgs::msg::Odometry                       odometry_;
  laser_msgs::msg::ReferenceState               last_waypoint_;
  std::vector<laser_msgs::msg::PoseWithHeading> desired_path_;
  std::vector<laser_msgs::msg::ReferenceState>  current_horizon_path_;

  laser_uav_planners::multirotor_t _planner_multirotor_params_;
  laser_uav_planners::pmm_t        _pmm_params_;
  laser_uav_planners::AgilePlanner agile_planner_;

  laser_uav_controllers::multirotor_t   _controller_multirotor_params_;
  laser_uav_controllers::acados_t       _acados_params_;
  laser_uav_controllers::NmpcController nmpc_controller_;
  laser_uav_controllers::IndiController indi_controller_;

  safe_area_t _safe_area_;

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

  rclcpp::Time mass_estimation_time_start_;
  double       estimated_mass_;
  double       estimated_mass_for_detect_landing_;

  int lock_waypoint_;

  double _takeoff_height_;
  double _takeoff_speed_;

  double _land_speed_;
  double _land_threshold_detect_;
  double _land_increment_rampdown_;

  double land_start_rampdown_;

  double mission_start_time_;

  laser_uav_lib::RMSE estimated_rmse_;

  bool stop_on_waypoints_{false};
  bool emergency_hover_{false};
  bool calculate_rmse_{false};
  bool start_mass_estimation_{false};
  bool received_first_odometry_msg_{false};
  bool angular_rates_and_thrust_mode_;
  bool lock_control_inputs_{true};
  bool have_nmpc_control_input_{false};
  bool requested_takeoff_{false};
  bool takeoff_done_{false};
  bool requested_land_{false};
  bool land_done_{true};
  bool land_rampdown_{false};
  bool is_active_{false};
};
}  // namespace laser_uav_managers

#endif
