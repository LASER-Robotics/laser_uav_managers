#ifndef LASER_UAV_MANAGERS__CONTROL_MANAGER_NODE_HPP_
#define LASER_UAV_MANAGERS__CONTROL_MANAGER_NODE_HPP_

#include <chrono>
#include <cstdint>

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <std_srvs/srv/trigger.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <sensor_msgs/msg/imu.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include <laser_msgs/msg/api_px4_diagnostics.hpp>
#include <laser_msgs/msg/attitude_rates_and_thrust.hpp>
#include <laser_msgs/msg/motor_speed.hpp>
#include <laser_msgs/msg/motor_speed_stamped.hpp>
#include <laser_msgs/msg/reference_state.hpp>
#include <laser_msgs/msg/uav_control_diagnostics.hpp>

#include <laser_uav_lib/filter/irr_filter.hpp>
#include <laser_uav_lib/metrics/rmse.hpp>
#include <mars_quadrotor_msgs/msg/polynomial_trajectory.hpp>
#include <laser_uav_controllers/indi_controller.hpp>
#include <laser_uav_controllers/nmpc_controller.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace laser_uav_managers
{

struct SafeArea
{
  bool enabled{false};
  std::vector<double> x{-1000.0, 1000.0};
  std::vector<double> y{-1000.0, 1000.0};
  std::vector<double> z{-1000.0, 1000.0};
};

/**
 * @brief Lifecycle control manager for external polynomial trajectory + NMPC.
 *
 * The original PMM/AgilePlanner is intentionally absent. Navigation references are sampled
 * directly from external planner's PolynomialTrajectory. Takeoff, hover and landing references are
 * generated locally so the existing NMPC/INDI, safety, diagnostics and RMSE logic can be preserved.
 */
class ControlManagerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit ControlManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ControlManagerNode() override;

private:
  friend struct ControlManagerNodeTestAccess;
  using PolynomialTrajectory = mars_quadrotor_msgs::msg::PolynomialTrajectory;

  // Lifecycle
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  // Configuration
  void declare_parameters_once();
  void get_parameters();
  void validate_parameters() const;
  void configure_pub_sub();
  void configure_timers();
  void configure_services();
  void configure_classes();

  // Callbacks
  void odometry_callback(const nav_msgs::msg::Odometry & msg);
  void imu_callback(const sensor_msgs::msg::Imu & msg);
  void motor_speed_callback(const laser_msgs::msg::MotorSpeedStamped & msg);
  void api_diagnostics_callback(const laser_msgs::msg::ApiPx4Diagnostics & msg);
  void polynomial_trajectory_callback(const PolynomialTrajectory & msg);
  void benchmark_goal_callback(const geometry_msgs::msg::PoseStamped & msg);
  void cancel_benchmark();
  void update_benchmark(const rclcpp::Time & now);
  geometry_msgs::msg::TransformStamped odometry_debug_transform(
    const nav_msgs::msg::Odometry & msg) const;

  void takeoff_service_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void land_service_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void external_loop_timer_callback();
  void internal_loop_timer_callback();
  void diagnostics_timer_callback();

  // external polynomial sampling
  bool build_external_horizon(
    const PolynomialTrajectory & traj, double wall_now_sec,
    std::vector<laser_msgs::msg::ReferenceState> & horizon) const;
  bool sample_position(
    const PolynomialTrajectory & traj, double absolute_wall_time, Eigen::Vector3d & position,
    Eigen::Vector3d & velocity) const;
  bool sample_yaw(
    const PolynomialTrajectory & traj, double absolute_wall_time, double & yaw,
    double & yaw_rate) const;
  bool sample_external_terminal_reference(
    const PolynomialTrajectory & traj, laser_msgs::msg::ReferenceState & ref) const;

  static bool locate_piece(
    const std::vector<double> & durations, double relative_time,
    std::size_t & piece_index, double & local_time);
  static double eval_poly_desc(
    const std::vector<double> & coefficients, std::size_t offset, std::size_t order, double t);
  static double eval_poly_derivative_desc(
    const std::vector<double> & coefficients, std::size_t offset, std::size_t order, double t);
  static double trajectory_duration(const std::vector<double> & durations);

  // Local reference generation (no AgilePlanner)
  void build_hold_horizon(
    const laser_msgs::msg::ReferenceState & hold,
    std::vector<laser_msgs::msg::ReferenceState> & horizon) const;
  bool update_reference_horizon(const rclcpp::Time & now);
  void start_vertical_reference(double target_z, double speed);
  void build_vertical_horizon(
    const rclcpp::Time & now, std::vector<laser_msgs::msg::ReferenceState> & horizon) const;
  static void minimum_jerk(double tau, double & s, double & ds_dtau);

  // Original control-manager helpers retained/adapted
  double euclidean_distance(
    const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2) const;
  static double normalize_heading(double heading);
  static double quaternion_to_heading(const geometry_msgs::msg::Quaternion & q);
  static geometry_msgs::msg::Quaternion heading_to_quaternion(double yaw);
  static double heading_error(double current, double target);
  void check_safe_area();
  bool estimate_mass();
  void set_hold_reference(const geometry_msgs::msg::Pose & pose);
  void set_emergency_hold_inside_safe_area();
  bool external_goal_reached(const laser_msgs::msg::ReferenceState & goal) const;
  void start_rmse_run_if_needed();
  void finish_rmse_run();
  void publish_reference_path(const std::vector<laser_msgs::msg::ReferenceState> & horizon);

  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<laser_msgs::msg::MotorSpeedStamped>::SharedPtr sub_motor_speed_;
  rclcpp::Subscription<laser_msgs::msg::ApiPx4Diagnostics>::SharedPtr sub_api_diagnostics_;
  rclcpp::Subscription<PolynomialTrajectory>::SharedPtr sub_polynomial_trajectory_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_benchmark_goal_;

  // Services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_takeoff_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_land_;

  // Lifecycle publishers
  rclcpp_lifecycle::LifecyclePublisher<laser_msgs::msg::AttitudeRatesAndThrust>::SharedPtr
    pub_attitude_rates_and_thrust_reference_;
  rclcpp_lifecycle::LifecyclePublisher<laser_msgs::msg::MotorSpeed>::SharedPtr
    pub_motor_speed_reference_;
  rclcpp_lifecycle::LifecyclePublisher<laser_msgs::msg::UavControlDiagnostics>::SharedPtr
    pub_diagnostics_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr pub_reference_path_;

  // Timers
  rclcpp::TimerBase::SharedPtr tmr_external_loop_control_;
  rclcpp::TimerBase::SharedPtr tmr_internal_loop_control_;
  rclcpp::TimerBase::SharedPtr tmr_diagnostics_;

  // Controllers (AgilePlanner/PMM intentionally absent)
  laser_uav_controllers::multirotor_t controller_multirotor_params_;
  laser_uav_controllers::acados_t acados_params_;
  laser_uav_controllers::NmpcController nmpc_controller_;
  laser_uav_controllers::IndiController indi_controller_;

  // Filters
  std::vector<double> gyro_a_;
  std::vector<double> gyro_b_;
  laser_uav_lib::IIRFilter btw_gyro_x_;
  laser_uav_lib::IIRFilter btw_gyro_y_;
  laser_uav_lib::IIRFilter btw_gyro_z_;

  std::vector<double> motor_a_;
  std::vector<double> motor_b_;
  std::vector<laser_uav_lib::IIRFilter> btw_motors_;

  // NMPC/INDI state
  std::pair<Eigen::Vector3d, Eigen::VectorXd> nmpc_solution_;
  Eigen::VectorXd motor_speed_estimated_;
  Eigen::Vector3d last_angular_speed_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_acceleration_estimated_{Eigen::Vector3d::Zero()};

  // Measured state is feedback to NMPC, not a moving reference origin.
  nav_msgs::msg::Odometry odometry_;
  laser_msgs::msg::UavControlDiagnostics diagnostics_;
  // Initialized once from odometry, then advanced only by commanded references.
  laser_msgs::msg::ReferenceState commanded_reference_;
  laser_msgs::msg::ReferenceState hold_reference_;
  laser_msgs::msg::ReferenceState vertical_start_reference_;
  std::vector<laser_msgs::msg::ReferenceState> current_horizon_path_;

  // External trajectory state
  // Accessed serially by callbacks in the default mutually exclusive callback group.
  PolynomialTrajectory latest_external_trajectory_;
  bool have_external_position_trajectory_{false};
  bool external_emergency_stop_{false};
  std::chrono::steady_clock::time_point last_external_rx_steady_{};
  bool have_external_rx_time_{false};
  bool stale_hold_active_{false};

  // RMSE
  laser_uav_lib::RMSE estimated_rmse_;
  bool calculate_rmse_{false};
  bool rmse_run_active_{false};

  // Safe area
  SafeArea safe_area_;

  // Rates
  double rate_external_loop_control_{1.0};
  double rate_internal_loop_control_{1.0};
  double rate_diagnostics_{1.0};

  // Takeoff/landing
  double takeoff_height_{0.0};
  double takeoff_speed_{0.0};
  double vertical_duration_{0.0};
  double vertical_target_z_{0.0};
  rclcpp::Time vertical_start_time_{0, 0, RCL_ROS_TIME};
  bool vertical_time_started_{false};

  double land_speed_{0.2};
  double land_threshold_detect_{0.8};
  double land_increment_rampdown_{0.05};
  double land_start_rampdown_{0.0};

  rclcpp::Time mass_estimation_time_start_{0, 0, RCL_ROS_TIME};
  double estimated_mass_{0.0};
  double estimated_mass_for_detect_landing_{0.0};

  // External-trajectory/debug params
  double external_timeout_sec_{0.5};
  bool hold_on_stale_{true};
  double goal_position_tolerance_{0.15};
  double goal_heading_tolerance_{0.15};
  bool publish_reference_path_{true};
  std::string reference_frame_id_{"world"};

  // Operational flags retained from the original control manager
  bool emergency_hover_{false};
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

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  struct BenchmarkMetrics
  {
    bool enabled{true};
    bool active{false};
    uint64_t run_id{0};
    std::string csv_path{"/tmp/laser_uav_control_metrics.csv"};
    std::string goal_topic{"/goal_pose"};
    double heartbeat_timeout{0.5};
    rclcpp::Time start_time{0, 0, RCL_ROS_TIME};
    Eigen::Vector3d start_position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d last_position{Eigen::Vector3d::Zero()};
    double distance{0.0};
    bool have_trajectory{false};
    double trajectory_start{0.0};
    double trajectory_end{0.0};
    laser_msgs::msg::ReferenceState terminal;
    bool require_heading{false};
    bool have_heartbeat{false};
    std::chrono::steady_clock::time_point last_heartbeat{};
  } benchmark_;

};

}  // namespace laser_uav_managers

#endif  // LASER_UAV_MANAGERS__CONTROL_MANAGER_NODE_HPP_
