#ifndef LASER_UAV_MANAGERS__CONTROL_MANAGER_NODE_HPP_
#define LASER_UAV_MANAGERS__CONTROL_MANAGER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include <memory>
#include <mutex>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include <std_srvs/srv/trigger.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <sensor_msgs/msg/imu.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <laser_msgs/msg/api_px4_diagnostics.hpp>
#include <laser_msgs/msg/attitude_rates_and_thrust.hpp>
#include <laser_msgs/msg/motor_speed.hpp>
#include <laser_msgs/msg/motor_speed_stamped.hpp>
#include <laser_msgs/msg/pose_with_heading.hpp>
#include <laser_msgs/msg/reference_state.hpp>
#include <laser_msgs/msg/trajectory_path.hpp>
#include <laser_msgs/msg/uav_control_diagnostics.hpp>

#include <laser_uav_controllers/indi_controller.hpp>
#include <laser_uav_controllers/nmpc_controller.hpp>
#include <laser_uav_lib/filter/irr_filter.hpp>
#include <laser_uav_lib/metrics/rmse.hpp>
#include <laser_uav_planners/agile_planner.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace laser_uav_managers
{

/**
 * @brief Represents 3D bounding box constraints for safe UAV operation.
 */
struct SafeArea
{
  bool enabled;
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> z;
};

/**
 * @brief Lifecycle node responsible for UAV trajectory tracking and cascade flight control.
 * It manages an outer Non-linear Model Predictive Controller (NMPC) and an optional
 * inner Incremental Nonlinear Dynamic Inversion (INDI) controller.
 */
class ControlManagerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit ControlManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ControlManagerNode() override;

private:
  // Lifecycle transitions
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  rclcpp::CallbackGroup::SharedPtr callback_group_;

  // Initialization helpers
  void get_parameters();
  void configure_pub_sub();
  void configure_timers();
  void configure_services();
  void configure_classes();

  // Mathematical and safety utility functions
  double euclidean_distance(geometry_msgs::msg::Point p1, geometry_msgs::msg::Point p2);
  double check_heading_error();
  double normalize_heading(double heading);
  double quaternion_to_heading(geometry_msgs::msg::Quaternion & q);
  void check_safe_area();
  bool estimate_mass();

  // Subscribers and callbacks
  rclcpp::Subscription<nav_msgs::msg::Odometry>::ConstSharedPtr sub_odometry_;
  void odometry_callback(const nav_msgs::msg::Odometry & msg);

  rclcpp::Subscription<sensor_msgs::msg::Imu>::ConstSharedPtr sub_imu_;
  void imu_callback(const sensor_msgs::msg::Imu & msg);

  rclcpp::Subscription<laser_msgs::msg::MotorSpeedStamped>::ConstSharedPtr sub_motor_speed_;
  void motor_speed_callback(const laser_msgs::msg::MotorSpeedStamped & msg);

  rclcpp::Subscription<laser_msgs::msg::PoseWithHeading>::ConstSharedPtr sub_goto_;
  void goto_callback(const laser_msgs::msg::PoseWithHeading & msg);

  rclcpp::Subscription<laser_msgs::msg::PoseWithHeading>::ConstSharedPtr sub_goto_relative_;
  void goto_relative_callback(const laser_msgs::msg::PoseWithHeading & msg);

  rclcpp::Subscription<laser_msgs::msg::TrajectoryPath>::ConstSharedPtr sub_trajectory_path_;
  void trajectory_path_callback(const laser_msgs::msg::TrajectoryPath & msg);

  rclcpp::Subscription<laser_msgs::msg::ApiPx4Diagnostics>::ConstSharedPtr sub_api_diagnostics_;
  void api_diagnostics_callback(const laser_msgs::msg::ApiPx4Diagnostics & msg);

  // Services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_takeoff_;
  void takeoff_service_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_land_;
  void land_service_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // Publishers and timers
  rclcpp_lifecycle::LifecyclePublisher<laser_msgs::msg::AttitudeRatesAndThrust>::SharedPtr
    pub_attitude_rates_and_thrust_reference_;
  double rate_external_loop_control_;
  rclcpp::TimerBase::SharedPtr tmr_external_loop_control_;
  void external_loop_timer_callback();

  rclcpp_lifecycle::LifecyclePublisher<laser_msgs::msg::MotorSpeed>::SharedPtr
    pub_motor_speed_reference_;
  double rate_internal_loop_control_;
  rclcpp::TimerBase::SharedPtr tmr_internal_loop_control_;
  void internal_loop_timer_callback();

  rclcpp_lifecycle::LifecyclePublisher<laser_msgs::msg::UavControlDiagnostics>::SharedPtr
    pub_diagnostics_;
  double rate_diagnostics_;
  rclcpp::TimerBase::SharedPtr tmr_diagnostics_;
  void diagnostics_timer_callback();

  // State variables
  laser_msgs::msg::UavControlDiagnostics diagnostics_;
  nav_msgs::msg::Odometry odometry_;
  laser_msgs::msg::ReferenceState last_waypoint_;
  std::vector<laser_msgs::msg::PoseWithHeading> desired_path_;
  std::vector<laser_msgs::msg::ReferenceState> current_horizon_path_;

  // Planners and controllers
  laser_uav_planners::multirotor_t planner_multirotor_params_;
  laser_uav_planners::pmm_t pmm_params_;
  laser_uav_planners::AgilePlanner agile_planner_;

  laser_uav_controllers::multirotor_t controller_multirotor_params_;
  laser_uav_controllers::acados_t acados_params_;
  laser_uav_controllers::NmpcController nmpc_controller_;
  laser_uav_controllers::IndiController indi_controller_;

  SafeArea safe_area_;

  // Signal filters
  std::vector<double> gyro_a_;
  std::vector<double> gyro_b_;
  laser_uav_lib::IIRFilter btw_gyro_x_;
  laser_uav_lib::IIRFilter btw_gyro_y_;
  laser_uav_lib::IIRFilter btw_gyro_z_;

  std::vector<double> motor_a_;
  std::vector<double> motor_b_;
  std::vector<laser_uav_lib::IIRFilter> btw_motors_;

  // Control estimation variables
  std::pair<Eigen::Vector3d, Eigen::VectorXd> nmpc_solution_;
  Eigen::VectorXd motor_speed_estimated_;
  Eigen::Vector3d last_angular_speed_;
  Eigen::Vector3d angular_acceleration_estimated_;

  rclcpp::Time mass_estimation_time_start_;
  double estimated_mass_;
  double estimated_mass_for_detect_landing_;

  int lock_waypoint_;

  double takeoff_height_;
  double takeoff_speed_;

  double land_speed_;
  double land_threshold_detect_;
  double land_increment_rampdown_;
  double land_start_rampdown_;

  double trajectory_speed_;

  laser_uav_lib::RMSE estimated_rmse_;

  // Operational state flags
  bool stop_on_waypoints_{false};
  bool emergency_hover_{false};
  bool calculate_rmse_{false};
  bool start_mass_estimation_{false};
  bool received_first_odometry_msg_{false};
  bool angular_rates_and_thrust_mode_{false};
  bool lock_control_inputs_{true};
  bool have_nmpc_solution_{false};
  bool requested_takeoff_{false};
  bool takeoff_done_{false};
  bool requested_land_{false};
  bool land_done_{true};
  bool land_rampdown_{false};
  bool is_active_{false};
};
}  // namespace laser_uav_managers

#endif  // LASER_UAV_MANAGERS__CONTROL_MANAGER_NODE_HPP_
