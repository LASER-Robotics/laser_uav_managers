#include "laser_uav_managers/control_manager_node.hpp"

#include <chrono>
#include <cmath>

#include <algorithm>
#include <array>
#include <fstream>
#include <iomanip>
#include <limits>
#include <stdexcept>

#include <geometry_msgs/msg/pose_stamped.hpp>

namespace laser_uav_managers
{

namespace
{
constexpr double kGravity = 9.81;
constexpr double kTwoPi = 2.0 * M_PI;
constexpr double kMassEstimateVzThreshold = 0.02;
constexpr double kMassEstimateDurationSec = 3.0;
constexpr double kTakeoffPositionTolerance = 0.10;

}  // namespace

ControlManagerNode::ControlManagerNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("control_manager", "", options)
{
  RCLCPP_INFO(
    get_logger(), "Creating external polynomial trajectory + NMPC lifecycle ControlManagerNode");

  declare_parameters_once();
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  diagnostics_ = laser_msgs::msg::UavControlDiagnostics();
  diagnostics_.is_fly = false;
  diagnostics_.metrics.rmse = NAN;
  diagnostics_.metrics.std = NAN;
}

ControlManagerNode::~ControlManagerNode() = default;

void ControlManagerNode::declare_parameters_once()
{
  declare_parameter("rate.external_loop_control", rclcpp::ParameterValue(1.0));
  declare_parameter("rate.internal_loop_control", rclcpp::ParameterValue(1.0));
  declare_parameter("rate.diagnostics", rclcpp::ParameterValue(1.0));

  declare_parameter("takeoff.height", rclcpp::ParameterValue(0.0));
  declare_parameter("takeoff.speed", rclcpp::ParameterValue(0.0));

  declare_parameter("land.speed", rclcpp::ParameterValue(0.2));
  declare_parameter("land.threshold_detect", rclcpp::ParameterValue(0.8));
  declare_parameter("land.increment_rampdown", rclcpp::ParameterValue(0.05));

  declare_parameter(
    "filter_params.butterworth.gyro_a", rclcpp::ParameterValue(std::vector<double>(3, 0.0)));
  declare_parameter(
    "filter_params.butterworth.gyro_b", rclcpp::ParameterValue(std::vector<double>(3, 0.0)));
  declare_parameter(
    "filter_params.butterworth.motor_a", rclcpp::ParameterValue(std::vector<double>(3, 0.0)));
  declare_parameter(
    "filter_params.butterworth.motor_b", rclcpp::ParameterValue(std::vector<double>(3, 0.0)));

  declare_parameter("multirotor_parameters.mass", rclcpp::ParameterValue(0.0));
  declare_parameter(
    "multirotor_parameters.inertia", rclcpp::ParameterValue(std::vector<double>(3, 0.0)));
  declare_parameter("multirotor_parameters.motor_inertia", rclcpp::ParameterValue(0.0));
  declare_parameter("multirotor_parameters.c_thrust", rclcpp::ParameterValue(0.0));
  declare_parameter("multirotor_parameters.c_tau", rclcpp::ParameterValue(0.0));
  declare_parameter(
    "multirotor_parameters.omega_max", rclcpp::ParameterValue(std::vector<double>(3, 0.0)));
  declare_parameter(
    "multirotor_parameters.drag", rclcpp::ParameterValue(std::vector<double>(3, 0.0)));
  declare_parameter("multirotor_parameters.n_motors", rclcpp::ParameterValue(0));
  declare_parameter(
    "multirotor_parameters.G1", rclcpp::ParameterValue(std::vector<double>(32, 0.0)));
  declare_parameter(
    "multirotor_parameters.G2", rclcpp::ParameterValue(std::vector<double>(32, 0.0)));
  declare_parameter("multirotor_parameters.quadratic_motor_model.a", rclcpp::ParameterValue(0.0));
  declare_parameter("multirotor_parameters.quadratic_motor_model.b", rclcpp::ParameterValue(0.0));
  declare_parameter("multirotor_parameters.thrust_min", rclcpp::ParameterValue(0.0));
  declare_parameter("multirotor_parameters.thrust_max", rclcpp::ParameterValue(0.0));
  declare_parameter("multirotor_parameters.total_thrust_max", rclcpp::ParameterValue(0.0));

  declare_parameter("nmpc_controller.nmpc_mode", rclcpp::ParameterValue(std::string("")));
  declare_parameter("nmpc_controller.N", rclcpp::ParameterValue(0));
  declare_parameter("nmpc_controller.dt", rclcpp::ParameterValue(0.0));
  declare_parameter("nmpc_controller.Q", rclcpp::ParameterValue(std::vector<double>(6, 0.0)));
  declare_parameter("nmpc_controller.R", rclcpp::ParameterValue(0.0));

  declare_parameter("safe_area.enabled", rclcpp::ParameterValue(false));
  declare_parameter("safe_area.constraints.x", rclcpp::ParameterValue(std::vector<double>(2, 0.0)));
  declare_parameter("safe_area.constraints.y", rclcpp::ParameterValue(std::vector<double>(2, 0.0)));
  declare_parameter("safe_area.constraints.z", rclcpp::ParameterValue(std::vector<double>(2, 0.0)));

  // New parameters: only the external trajectory interface and its debug/completion behaviour.
  declare_parameter("external_trajectory.timeout", rclcpp::ParameterValue(0.5));
  declare_parameter("external_trajectory.hold_on_stale", rclcpp::ParameterValue(true));
  declare_parameter("external_trajectory.goal_position_tolerance", rclcpp::ParameterValue(0.15));
  declare_parameter("external_trajectory.goal_heading_tolerance", rclcpp::ParameterValue(0.15));
  declare_parameter("debug.publish_reference_path", rclcpp::ParameterValue(true));
  declare_parameter("benchmark_metrics.enabled", rclcpp::ParameterValue(true));
  declare_parameter(
    "benchmark_metrics.csv_path",
    std::string("/tmp/laser_uav_control_metrics.csv"));
  declare_parameter("benchmark_metrics.goal_topic", std::string("/goal_pose"));
  declare_parameter("benchmark_metrics.heartbeat_end_timeout", rclcpp::ParameterValue(0.5));

}

CallbackReturn ControlManagerNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring external polynomial trajectory + NMPC control manager");

  try {
    get_parameters();
    validate_parameters();
    configure_pub_sub();
    configure_timers();
    configure_services();
    configure_classes();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Configuration failed: %s", e.what());
    return CallbackReturn::FAILURE;
  }

  diagnostics_.metrics.rmse = NAN;
  diagnostics_.metrics.std = NAN;
  diagnostics_.estimated_mass = controller_multirotor_params_.mass;
  estimated_mass_ = controller_multirotor_params_.mass;
  estimated_mass_for_detect_landing_ = controller_multirotor_params_.mass;

  lock_control_inputs_ = true;
  have_nmpc_solution_ = false;
  requested_takeoff_ = false;
  takeoff_done_ = false;
  requested_land_ = false;
  land_done_ = true;
  land_rampdown_ = false;
  emergency_hover_ = false;
  diagnostics_.is_fly = false;
  diagnostics_.have_goal = false;

  RCLCPP_INFO(
    get_logger(), "Configured: N=%d dt=%.4f external=%.1fHz mode=%s external planner timeout=%.2fs",
    acados_params_.N, acados_params_.dt, rate_external_loop_control_,
    angular_rates_and_thrust_mode_ ? "body_rates+thrust" : "individual_thrust+INDI",
    external_timeout_sec_);

  return CallbackReturn::SUCCESS;
}

CallbackReturn ControlManagerNode::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating");

  if (angular_rates_and_thrust_mode_) {
    pub_attitude_rates_and_thrust_reference_->on_activate();
  } else {
    pub_motor_speed_reference_->on_activate();
  }
  pub_diagnostics_->on_activate();
  if (publish_reference_path_) {
    pub_reference_path_->on_activate();
  }

  is_active_ = true;
  return CallbackReturn::SUCCESS;
}

CallbackReturn ControlManagerNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating");
  cancel_benchmark();

  if (angular_rates_and_thrust_mode_) {
    pub_attitude_rates_and_thrust_reference_->on_deactivate();
  } else {
    pub_motor_speed_reference_->on_deactivate();
  }
  pub_diagnostics_->on_deactivate();
  if (publish_reference_path_) {
    pub_reference_path_->on_deactivate();
  }

  is_active_ = false;
  return CallbackReturn::SUCCESS;
}

CallbackReturn ControlManagerNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  tmr_external_loop_control_.reset();
  tmr_internal_loop_control_.reset();
  tmr_diagnostics_.reset();

  srv_takeoff_.reset();
  srv_land_.reset();

  sub_odometry_.reset();
  sub_imu_.reset();
  sub_motor_speed_.reset();
  sub_api_diagnostics_.reset();
  sub_polynomial_trajectory_.reset();
  sub_benchmark_goal_.reset();
  cancel_benchmark();

  pub_attitude_rates_and_thrust_reference_.reset();
  pub_motor_speed_reference_.reset();
  pub_diagnostics_.reset();
  pub_reference_path_.reset();

  current_horizon_path_.clear();
  have_external_position_trajectory_ = false;
  external_emergency_stop_ = false;
  have_external_rx_time_ = false;
  stale_hold_active_ = false;
  rmse_run_active_ = false;
  estimated_rmse_.reset();

  is_active_ = false;

  return CallbackReturn::SUCCESS;
}

CallbackReturn ControlManagerNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return CallbackReturn::SUCCESS;
}

void ControlManagerNode::get_parameters()
{
  rclcpp::Parameter aux;

  get_parameter("rate.external_loop_control", rate_external_loop_control_);
  get_parameter("rate.internal_loop_control", rate_internal_loop_control_);
  get_parameter("rate.diagnostics", rate_diagnostics_);

  get_parameter("takeoff.height", takeoff_height_);
  get_parameter("takeoff.speed", takeoff_speed_);

  get_parameter("land.speed", land_speed_);
  get_parameter("land.threshold_detect", land_threshold_detect_);
  get_parameter("land.increment_rampdown", land_increment_rampdown_);

  get_parameter("filter_params.butterworth.gyro_a", aux);
  gyro_a_ = aux.as_double_array();
  get_parameter("filter_params.butterworth.gyro_b", aux);
  gyro_b_ = aux.as_double_array();
  get_parameter("filter_params.butterworth.motor_a", aux);
  motor_a_ = aux.as_double_array();
  get_parameter("filter_params.butterworth.motor_b", aux);
  motor_b_ = aux.as_double_array();

  get_parameter("multirotor_parameters.mass", controller_multirotor_params_.mass);

  get_parameter("multirotor_parameters.inertia", aux);
  if (aux.as_double_array().size() == 3) {
    controller_multirotor_params_.inertia_matrix =
      Eigen::Map<const Eigen::Vector3d>(aux.as_double_array().data()).asDiagonal();
  }

  get_parameter("multirotor_parameters.c_thrust", controller_multirotor_params_.c_thrust);

  get_parameter("multirotor_parameters.drag", aux);
  if (aux.as_double_array().size() == 3) {
    controller_multirotor_params_.drag =
      Eigen::Map<const Eigen::Vector3d>(aux.as_double_array().data());
  }

  get_parameter("multirotor_parameters.omega_max", aux);
  if (aux.as_double_array().size() == 3) {
    controller_multirotor_params_.omega_max =
      Eigen::Map<const Eigen::Vector3d>(aux.as_double_array().data());
  }

  get_parameter("multirotor_parameters.n_motors", controller_multirotor_params_.n_motors);

  get_parameter("multirotor_parameters.G1", aux);
  if (
    controller_multirotor_params_.n_motors > 0 &&
    aux.as_double_array().size() >=
    static_cast<std::size_t>(4 * controller_multirotor_params_.n_motors))
  {
    controller_multirotor_params_.G1 =
      Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      aux.as_double_array().data(), 4, controller_multirotor_params_.n_motors);
  }

  get_parameter("multirotor_parameters.G2", aux);
  if (
    controller_multirotor_params_.n_motors > 0 &&
    aux.as_double_array().size() >=
    static_cast<std::size_t>(4 * controller_multirotor_params_.n_motors))
  {
    controller_multirotor_params_.G2 =
      Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      aux.as_double_array().data(), 4, controller_multirotor_params_.n_motors);
  }

  get_parameter("multirotor_parameters.motor_inertia", controller_multirotor_params_.motor_inertia);
  get_parameter(
    "multirotor_parameters.quadratic_motor_model.a", controller_multirotor_params_.motor_curve_a);
  get_parameter(
    "multirotor_parameters.quadratic_motor_model.b", controller_multirotor_params_.motor_curve_b);
  get_parameter("multirotor_parameters.thrust_min", controller_multirotor_params_.thrust_min);
  get_parameter("multirotor_parameters.thrust_max", controller_multirotor_params_.thrust_max);
  get_parameter(
    "multirotor_parameters.total_thrust_max", controller_multirotor_params_.total_thrust_max);

  get_parameter("nmpc_controller.nmpc_mode", acados_params_.nmpc_mode);
  angular_rates_and_thrust_mode_ = acados_params_.nmpc_mode != "individual_thrust";
  get_parameter("nmpc_controller.N", acados_params_.N);
  get_parameter("nmpc_controller.dt", acados_params_.dt);
  get_parameter("nmpc_controller.Q", aux);
  acados_params_.Q = aux.as_double_array();
  get_parameter("nmpc_controller.R", acados_params_.R);

  get_parameter("safe_area.enabled", safe_area_.enabled);
  get_parameter("safe_area.constraints.x", aux);
  safe_area_.x = aux.as_double_array();
  get_parameter("safe_area.constraints.y", aux);
  safe_area_.y = aux.as_double_array();
  get_parameter("safe_area.constraints.z", aux);
  safe_area_.z = aux.as_double_array();

  get_parameter("external_trajectory.timeout", external_timeout_sec_);
  get_parameter("external_trajectory.hold_on_stale", hold_on_stale_);
  get_parameter("external_trajectory.goal_position_tolerance", goal_position_tolerance_);
  get_parameter("external_trajectory.goal_heading_tolerance", goal_heading_tolerance_);
  get_parameter("debug.publish_reference_path", publish_reference_path_);
  get_parameter("benchmark_metrics.enabled", benchmark_.enabled);
  get_parameter("benchmark_metrics.csv_path", benchmark_.csv_path);
  get_parameter("benchmark_metrics.goal_topic", benchmark_.goal_topic);
  get_parameter("benchmark_metrics.heartbeat_end_timeout", benchmark_.heartbeat_timeout);
  benchmark_.heartbeat_timeout = std::max(0.02, benchmark_.heartbeat_timeout);
}

void ControlManagerNode::validate_parameters() const
{
  if (rate_external_loop_control_ <= 0.0 || rate_diagnostics_ <= 0.0) {
    throw std::runtime_error("rate.external_loop_control and rate.diagnostics must be > 0");
  }
  if (!angular_rates_and_thrust_mode_ && rate_internal_loop_control_ <= 0.0) {
    throw std::runtime_error("rate.internal_loop_control must be > 0 in individual_thrust mode");
  }
  if (acados_params_.N <= 0 || acados_params_.dt <= 0.0) {
    throw std::runtime_error("nmpc_controller.N and dt must be > 0");
  }
  if (controller_multirotor_params_.mass <= 0.0 || controller_multirotor_params_.n_motors <= 0) {
    throw std::runtime_error("multirotor mass and n_motors must be > 0");
  }
  if (takeoff_speed_ <= 0.0) {
    throw std::runtime_error("takeoff.speed must be > 0");
  }
  if (land_speed_ <= 0.0) {
    throw std::runtime_error("land.speed must be > 0");
  }
  if (land_increment_rampdown_ <= 0.0) {
    throw std::runtime_error("land.increment_rampdown must be > 0");
  }
  if (external_timeout_sec_ <= 0.0) {
    throw std::runtime_error("external_trajectory.timeout must be > 0");
  }
  if (
    safe_area_.enabled &&
    (safe_area_.x.size() != 2 || safe_area_.y.size() != 2 || safe_area_.z.size() != 2))
  {
    throw std::runtime_error("safe_area constraints must each contain [min, max]");
  }
}

void ControlManagerNode::configure_pub_sub()
{
  RCLCPP_INFO(get_logger(), "Configuring publishers and subscribers");

  const auto sensor_qos = rclcpp::SensorDataQoS();
  const auto trajectory_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();

  sub_odometry_ = create_subscription<nav_msgs::msg::Odometry>(
    "odometry_in", sensor_qos,
    std::bind(&ControlManagerNode::odometry_callback, this, std::placeholders::_1));

  sub_api_diagnostics_ = create_subscription<laser_msgs::msg::ApiPx4Diagnostics>(
    "api_diagnostics_in", 1,
    std::bind(&ControlManagerNode::api_diagnostics_callback, this, std::placeholders::_1));

  sub_polynomial_trajectory_ = create_subscription<PolynomialTrajectory>(
    "polynomial_trajectory_in", trajectory_qos,
    std::bind(&ControlManagerNode::polynomial_trajectory_callback, this, std::placeholders::_1));

  if (benchmark_.enabled) {
    sub_benchmark_goal_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      benchmark_.goal_topic, 10,
      std::bind(&ControlManagerNode::benchmark_goal_callback, this, std::placeholders::_1));
  }

  if (!angular_rates_and_thrust_mode_) {
    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
      "imu_in", sensor_qos,
      std::bind(&ControlManagerNode::imu_callback, this, std::placeholders::_1));
    sub_motor_speed_ = create_subscription<laser_msgs::msg::MotorSpeedStamped>(
      "motor_speed_estimation_in", sensor_qos,
      std::bind(&ControlManagerNode::motor_speed_callback, this, std::placeholders::_1));
  }

  if (angular_rates_and_thrust_mode_) {
    pub_attitude_rates_and_thrust_reference_ =
      create_publisher<laser_msgs::msg::AttitudeRatesAndThrust>("attitude_rates_thrust_out", 10);
  } else {
    pub_motor_speed_reference_ =
      create_publisher<laser_msgs::msg::MotorSpeed>("motor_speed_reference_out", 10);
  }

  pub_diagnostics_ =
    create_publisher<laser_msgs::msg::UavControlDiagnostics>("diagnostics_out", 10);
  pub_reference_path_ = create_publisher<nav_msgs::msg::Path>("reference_horizon_out", 1);
}

void ControlManagerNode::configure_timers()
{
  RCLCPP_INFO(get_logger(), "Configuring timers");

  tmr_external_loop_control_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / rate_external_loop_control_),
    std::bind(&ControlManagerNode::external_loop_timer_callback, this));

  if (!angular_rates_and_thrust_mode_) {
    tmr_internal_loop_control_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_internal_loop_control_),
      std::bind(&ControlManagerNode::internal_loop_timer_callback, this));
  }

  tmr_diagnostics_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / rate_diagnostics_),
    std::bind(&ControlManagerNode::diagnostics_timer_callback, this));
}

void ControlManagerNode::configure_services()
{
  RCLCPP_INFO(get_logger(), "Configuring takeoff/land services");

  srv_takeoff_ = create_service<std_srvs::srv::Trigger>(
    "takeoff", std::bind(
      &ControlManagerNode::takeoff_service_callback, this, std::placeholders::_1,
      std::placeholders::_2));

  srv_land_ = create_service<std_srvs::srv::Trigger>(
    "land", std::bind(
      &ControlManagerNode::land_service_callback, this, std::placeholders::_1,
      std::placeholders::_2));
}

void ControlManagerNode::configure_classes()
{
  RCLCPP_INFO(get_logger(), "Configuring NMPC/INDI (no AgilePlanner)");

  nmpc_solution_ = std::make_pair(
    Eigen::Vector3d::Zero(), Eigen::VectorXd::Zero(controller_multirotor_params_.n_motors));
  motor_speed_estimated_ = Eigen::VectorXd::Zero(controller_multirotor_params_.n_motors);
  last_angular_speed_ = Eigen::Vector3d::Zero();
  angular_acceleration_estimated_ = Eigen::Vector3d::Zero();

  nmpc_controller_ =
    laser_uav_controllers::NmpcController(controller_multirotor_params_, acados_params_);

  if (!angular_rates_and_thrust_mode_) {
    btw_gyro_x_ = laser_uav_lib::IIRFilter(gyro_a_, gyro_b_);
    btw_gyro_y_ = laser_uav_lib::IIRFilter(gyro_a_, gyro_b_);
    btw_gyro_z_ = laser_uav_lib::IIRFilter(gyro_a_, gyro_b_);

    btw_motors_.clear();
    for (int i = 0; i < controller_multirotor_params_.n_motors; ++i) {
      btw_motors_.emplace_back(motor_a_, motor_b_);
    }

    indi_controller_ = laser_uav_controllers::IndiController(controller_multirotor_params_);
  }
}

void ControlManagerNode::odometry_callback(const nav_msgs::msg::Odometry & msg)
{
  if (!is_active_) {
    return;
  }

  const auto & p = msg.pose.pose.position;
  const auto & q = msg.pose.pose.orientation;
  const auto & v = msg.twist.twist.linear;
  const auto & w = msg.twist.twist.angular;
  const std::array<double, 13> state{
    p.x, p.y, p.z, q.w, q.x, q.y, q.z, v.x, v.y, v.z, w.x, w.y, w.z};
  if (!std::all_of(state.begin(), state.end(), [](double x) {return std::isfinite(x);}) ||
    Eigen::Quaterniond(q.w, q.x, q.y, q.z).norm() < 1e-9)
  {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "Rejected invalid odometry");
    return;
  }
  odometry_ = msg;

  diagnostics_.current_norm_speed = std::sqrt(
    std::pow(msg.twist.twist.linear.x, 2) + std::pow(msg.twist.twist.linear.y, 2) +
    std::pow(msg.twist.twist.linear.z, 2));

  if (!received_first_odometry_msg_) {
    // This is the only measurement used to initialize the internal reference.
    received_first_odometry_msg_ = true;
    set_hold_reference(msg.pose.pose);
    reference_frame_id_ = msg.header.frame_id.empty() ? "world" : msg.header.frame_id;
    commanded_reference_ = hold_reference_;
  }
  if (benchmark_.active) {
    const Eigen::Vector3d current_position(p.x, p.y, p.z);
    benchmark_.distance += (current_position - benchmark_.last_position).norm();
    benchmark_.last_position = current_position;
  }
  tf_broadcaster_->sendTransform(odometry_debug_transform(msg));
}

geometry_msgs::msg::TransformStamped ControlManagerNode::odometry_debug_transform(
  const nav_msgs::msg::Odometry & msg) const
{
  std::string prefix = get_namespace();
  prefix.erase(0, 1);  // TF frame IDs must not start with '/'.
  if (!prefix.empty()) {
    prefix += '/';
  }
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = msg.header.stamp;
  transform.header.frame_id = prefix + "cm_odometry_debug";
  transform.child_frame_id = prefix + "fcu";
  transform.transform.translation.x = msg.pose.pose.position.x;
  transform.transform.translation.y = msg.pose.pose.position.y;
  transform.transform.translation.z = msg.pose.pose.position.z;
  transform.transform.rotation = msg.pose.pose.orientation;
  return transform;
}

void ControlManagerNode::imu_callback(const sensor_msgs::msg::Imu & msg)
{
  if (!is_active_) {
    return;
  }

  Eigen::Vector3d current;
  current << btw_gyro_x_.iterate(msg.angular_velocity.x),
    btw_gyro_y_.iterate(msg.angular_velocity.y), btw_gyro_z_.iterate(msg.angular_velocity.z);

  const double dt = 1.0 / rate_internal_loop_control_;
  angular_acceleration_estimated_ = (current - last_angular_speed_) / dt;
  last_angular_speed_ = current;
}

void ControlManagerNode::motor_speed_callback(const laser_msgs::msg::MotorSpeedStamped & msg)
{
  if (!is_active_) {
    return;
  }

  const auto n = std::min<std::size_t>(msg.data.data.size(), btw_motors_.size());
  for (std::size_t i = 0; i < n; ++i) {
    motor_speed_estimated_(static_cast<Eigen::Index>(i)) = btw_motors_[i].iterate(msg.data.data[i]);
  }
}

void ControlManagerNode::api_diagnostics_callback(const laser_msgs::msg::ApiPx4Diagnostics & msg)
{
  if (!is_active_) {
    return;
  }

  // Preserve the original behaviour: the first takeoff request unlocks control only after
  // the PX4 API reports armed + offboard.
  if (msg.armed && requested_takeoff_ && msg.offboard_mode) {
    lock_control_inputs_ = false;
  }
}

void ControlManagerNode::polynomial_trajectory_callback(const PolynomialTrajectory & msg)
{
  if (!is_active_ || requested_land_ || land_done_) {
    return;
  }

  if ((msg.type & PolynomialTrajectory::HEART_BEAT) != 0U && benchmark_.active &&
    benchmark_.have_trajectory && std::abs(msg.start_wt_pos - benchmark_.trajectory_start) <= 1e-6)
  {
    benchmark_.last_heartbeat = std::chrono::steady_clock::now();
    benchmark_.have_heartbeat = true;
  }

  if ((msg.type & PolynomialTrajectory::EMER_STOP) != 0U) {
    if (!takeoff_done_) {
      return;
    }
    external_emergency_stop_ = true;
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "external planner EMER_STOP received; navigation tracking will switch to hold.");
    return;
  }

  // IMPORTANT:
  // SUPER publishes HEART_BEAT messages much more frequently than complete polynomials.
  // A heartbeat proves that SUPER is alive, but it does NOT prove that the polynomial stored
  // by this ControlManager is still the current committed trajectory.
  //
  // Only heartbeats naming the stored trajectory refresh its active deadline.
  if ((msg.type & PolynomialTrajectory::POSITION_TRAJ) == 0U) {
    if (
      have_external_position_trajectory_ &&
      std::abs(msg.start_wt_pos - latest_external_trajectory_.start_wt_pos) > 1e-6)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "SUPER heartbeat refers to committed trajectory start %.6f, but the last full polynomial "
        "stored by ControlManager starts at %.6f. Waiting for a fresh POSITION_TRAJ.",
        msg.start_wt_pos, latest_external_trajectory_.start_wt_pos);
    } else if (have_external_position_trajectory_ &&
      (msg.type & PolynomialTrajectory::HEART_BEAT) != 0U)
    {
      // Only the heartbeat for the stored polynomial can keep it active.
      last_external_rx_steady_ = std::chrono::steady_clock::now();
    }
    return;
  }

  const std::size_t pos_stride = static_cast<std::size_t>(msg.order_pos) + 1U;
  const std::size_t pos_expected = static_cast<std::size_t>(msg.piece_num_pos) * pos_stride;
  const auto finite_values = [](const std::vector<double> & values) {
      return std::all_of(values.begin(), values.end(), [](double x) {return std::isfinite(x);});
    };
  const auto positive_durations = [&](const std::vector<double> & values) {
      return finite_values(values) &&
             std::all_of(values.begin(), values.end(), [](double x) {return x > 0.0;});
    };
  if (
    msg.piece_num_pos == 0U || msg.time_pos.size() != msg.piece_num_pos ||
    msg.coef_pos_x.size() != pos_expected || msg.coef_pos_y.size() != pos_expected ||
    msg.coef_pos_z.size() != pos_expected || !std::isfinite(msg.start_wt_pos) ||
    !positive_durations(msg.time_pos) || !finite_values(msg.coef_pos_x) ||
    !finite_values(msg.coef_pos_y) || !finite_values(msg.coef_pos_z))
  {
    RCLCPP_ERROR(
      get_logger(),
      "Rejected malformed external planner position polynomial: pieces=%u order=%u time=%zu "
      "coeff=(%zu,%zu,%zu) expected=%zu",
      msg.piece_num_pos, msg.order_pos, msg.time_pos.size(), msg.coef_pos_x.size(),
      msg.coef_pos_y.size(), msg.coef_pos_z.size(), pos_expected);
    return;
  }

  if ((msg.type & PolynomialTrajectory::YAW_TRAJ) != 0U) {
    const std::size_t yaw_stride = static_cast<std::size_t>(msg.order_yaw) + 1U;
    const std::size_t yaw_expected = static_cast<std::size_t>(msg.piece_num_yaw) * yaw_stride;
    if (
      msg.piece_num_yaw == 0U || msg.time_yaw.size() != msg.piece_num_yaw ||
      msg.coef_yaw.size() != yaw_expected || !std::isfinite(msg.start_wt_yaw) ||
      !positive_durations(msg.time_yaw) || !finite_values(msg.coef_yaw))
    {
      RCLCPP_ERROR(get_logger(), "Rejected malformed external yaw polynomial");
      return;
    }
  }
  if ((msg.type & PolynomialTrajectory::YAW_COMMAND) != 0U &&
    (!std::isfinite(msg.yaw) || !std::isfinite(msg.yaw_rate)))
  {
    RCLCPP_ERROR(get_logger(), "Rejected non-finite external yaw command");
    return;
  }

  // A full polynomial replaces the stored trajectory; matching heartbeats keep it fresh.
  last_external_rx_steady_ = std::chrono::steady_clock::now();
  have_external_rx_time_ = true;

  // Store exactly the last valid POSITION_TRAJ published by SUPER.
  latest_external_trajectory_ = msg;
  if (benchmark_.active) {
    benchmark_.have_trajectory = sample_external_terminal_reference(msg, benchmark_.terminal);
    benchmark_.trajectory_start = msg.start_wt_pos;
    benchmark_.trajectory_end = msg.start_wt_pos + trajectory_duration(msg.time_pos);
    benchmark_.require_heading = (msg.type & (PolynomialTrajectory::YAW_TRAJ |
      PolynomialTrajectory::YAW_COMMAND)) != 0U;
    if ((msg.type & PolynomialTrajectory::YAW_TRAJ) != 0U) {
      double yaw = 0.0, yaw_rate = 0.0;
      sample_yaw(msg, msg.start_wt_yaw + trajectory_duration(msg.time_yaw), yaw, yaw_rate);
      benchmark_.terminal.pose.orientation = heading_to_quaternion(yaw);
    }
    benchmark_.have_heartbeat = (msg.type & PolynomialTrajectory::HEART_BEAT) != 0U;
    benchmark_.last_heartbeat = std::chrono::steady_clock::now();
  }
  have_external_position_trajectory_ = true;
  stale_hold_active_ = false;
  external_emergency_stop_ = false;
  if (takeoff_done_) {
    emergency_hover_ = false;
  }

  RCLCPP_INFO(
    get_logger(),
    "SUPER polynomial received: id=%u frame='%s' pieces=%u order=%u start_ros=%.6f duration=%.3fs",
    msg.trajectory_id, msg.header.frame_id.c_str(), msg.piece_num_pos, msg.order_pos,
    msg.start_wt_pos, trajectory_duration(msg.time_pos));
}

void ControlManagerNode::takeoff_service_callback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!is_active_) {
    response->success = false;
    response->message = "control manager is not active";
    return;
  }

  if (!received_first_odometry_msg_) {
    response->success = false;
    response->message = "takeoff failed: odometry not received";
    RCLCPP_ERROR(get_logger(), "Takeoff failed because odometry has not been received.");
    return;
  }

  if (takeoff_done_ || requested_takeoff_ || requested_land_ || land_rampdown_) {
    response->success = false;
    response->message = "takeoff failed: UAV already flying/taking off";
    return;
  }

  requested_takeoff_ = true;
  cancel_benchmark();
  requested_land_ = false;
  land_done_ = false;
  land_rampdown_ = false;
  emergency_hover_ = false;
  diagnostics_.have_goal = true;
  calculate_rmse_ = false;
  rmse_run_active_ = false;
  estimated_rmse_.reset();

  start_vertical_reference(takeoff_height_, takeoff_speed_);
  start_mass_estimation_ = true;

  response->success = true;
  response->message = "takeoff requested";
  RCLCPP_INFO(
    get_logger(), "Takeoff requested: z %.3f -> %.3f, duration %.2fs",
    vertical_start_reference_.pose.position.z, takeoff_height_, vertical_duration_);
}

void ControlManagerNode::land_service_callback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!is_active_) {
    response->success = false;
    response->message = "control manager is not active";
    return;
  }

  if (land_done_ || requested_land_) {
    response->success = false;
    response->message = "land failed: UAV already landed/landing";
    return;
  }

  if (!received_first_odometry_msg_) {
    response->success = false;
    response->message = "land failed: odometry not received";
    return;
  }

  requested_land_ = true;
  cancel_benchmark();
  requested_takeoff_ = false;
  takeoff_done_ = false;
  emergency_hover_ = false;
  diagnostics_.have_goal = true;
  start_vertical_reference(-1.0, land_speed_);
  finish_rmse_run();

  response->success = true;
  response->message = "land requested";
  RCLCPP_INFO(get_logger(), "Landing requested at %.2f m/s", land_speed_);
}

bool ControlManagerNode::locate_piece(
  const std::vector<double> & durations, double relative_time,
  std::size_t & piece_index, double & local_time)
{
  if (durations.empty()) {
    return false;
  }

  double t = std::max(0.0, relative_time);
  for (std::size_t i = 0; i < durations.size(); ++i) {
    if (durations[i] <= 0.0) {
      return false;
    }
    if (t <= durations[i]) {
      piece_index = i;
      local_time = t;
      return true;
    }
    t -= durations[i];
  }

  piece_index = durations.size() - 1U;
  local_time = durations.back();
  return true;
}

double ControlManagerNode::eval_poly_desc(
  const std::vector<double> & coefficients, std::size_t offset, std::size_t order, double t)
{
  // external planner/GCOPTER serializes each row as [a_n, ..., a_1, a_0].
  double value = coefficients[offset];
  for (std::size_t j = 1; j <= order; ++j) {
    value = value * t + coefficients[offset + j];
  }
  return value;
}

double ControlManagerNode::eval_poly_derivative_desc(
  const std::vector<double> & coefficients, std::size_t offset, std::size_t order, double t)
{
  if (order == 0U) {
    return 0.0;
  }

  double value = static_cast<double>(order) * coefficients[offset];
  for (std::size_t j = 1; j < order; ++j) {
    const double power = static_cast<double>(order - j);
    value = value * t + power * coefficients[offset + j];
  }
  return value;
}

double ControlManagerNode::trajectory_duration(const std::vector<double> & durations)
{
  double total = 0.0;
  for (const double dt : durations) {
    total += dt;
  }
  return total;
}

bool ControlManagerNode::sample_position(
  const PolynomialTrajectory & traj, double absolute_ros_time, Eigen::Vector3d & position,
  Eigen::Vector3d & velocity) const
{
  if ((traj.type & PolynomialTrajectory::POSITION_TRAJ) == 0U || traj.piece_num_pos == 0U) {
    return false;
  }

  const double relative_time = absolute_ros_time - traj.start_wt_pos;
  const double total_duration = trajectory_duration(traj.time_pos);

  std::size_t piece = 0U;
  double local_t = 0.0;

  // Thin adapter semantics:
  //   - inside [0, T]: evaluate exactly the polynomial sent by SUPER;
  //   - before t=0: use the first polynomial state;
  //   - after T: use the terminal polynomial state with zero velocity.
  //
  // No trajectory reshaping, replanning, interpolation between messages, or velocity-based
  // orientation logic is performed here.
  if (!locate_piece(traj.time_pos, relative_time, piece, local_t)) {
    return false;
  }

  const std::size_t order = static_cast<std::size_t>(traj.order_pos);
  const std::size_t stride = order + 1U;
  const std::size_t offset = piece * stride;

  position.x() = eval_poly_desc(traj.coef_pos_x, offset, order, local_t);
  position.y() = eval_poly_desc(traj.coef_pos_y, offset, order, local_t);
  position.z() = eval_poly_desc(traj.coef_pos_z, offset, order, local_t);

  velocity.x() = eval_poly_derivative_desc(traj.coef_pos_x, offset, order, local_t);
  velocity.y() = eval_poly_derivative_desc(traj.coef_pos_y, offset, order, local_t);
  velocity.z() = eval_poly_derivative_desc(traj.coef_pos_z, offset, order, local_t);

  if (relative_time < 0.0 || relative_time >= total_duration) {
    velocity.setZero();
  }

  return true;
}

bool ControlManagerNode::sample_yaw(
  const PolynomialTrajectory & traj, double absolute_ros_time, double & yaw,
  double & yaw_rate) const
{
  // Use the yaw polynomial exactly as published by SUPER when present and valid.
  if ((traj.type & PolynomialTrajectory::YAW_TRAJ) != 0U && traj.piece_num_yaw > 0U) {
    const std::size_t order = static_cast<std::size_t>(traj.order_yaw);
    const std::size_t stride = order + 1U;
    const std::size_t expected = static_cast<std::size_t>(traj.piece_num_yaw) * stride;

    if (traj.time_yaw.size() == traj.piece_num_yaw && traj.coef_yaw.size() == expected) {
      const double relative_time = absolute_ros_time - traj.start_wt_yaw;
      const double total_duration = trajectory_duration(traj.time_yaw);

      std::size_t piece = 0U;
      double local_t = 0.0;
      if (locate_piece(traj.time_yaw, relative_time, piece, local_t)) {
        const std::size_t offset = piece * stride;
        yaw = normalize_heading(eval_poly_desc(traj.coef_yaw, offset, order, local_t));
        yaw_rate = eval_poly_derivative_desc(traj.coef_yaw, offset, order, local_t);

        if (relative_time < 0.0 || relative_time >= total_duration) {
          yaw_rate = 0.0;
        }

        return true;
      }
    }
  }

  // If SUPER explicitly publishes a direct yaw command, use it exactly.
  if ((traj.type & PolynomialTrajectory::YAW_COMMAND) != 0U) {
    yaw = normalize_heading(traj.yaw);
    yaw_rate = traj.yaw_rate;
    return true;
  }

  // No yaw information was supplied by SUPER. Do not invent one from velocity.
  return false;
}

bool ControlManagerNode::build_external_horizon(
  const PolynomialTrajectory & traj, double ros_now_sec,
  std::vector<laser_msgs::msg::ReferenceState> & horizon) const
{
  horizon.clear();
  horizon.reserve(static_cast<std::size_t>(acados_params_.N + 1));
  const double fallback_yaw = quaternion_to_heading(commanded_reference_.pose.orientation);

  for (int i = 0; i <= acados_params_.N; ++i) {
    // The only adaptation performed for NMPC is temporal sampling of SUPER's committed
    // polynomial at the controller prediction grid.
    const double sample_time = ros_now_sec + static_cast<double>(i) * acados_params_.dt;

    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    if (!sample_position(traj, sample_time, position, velocity)) {
      return false;
    }

    double yaw = 0.0;
    double yaw_rate = 0.0;
    const bool have_super_yaw = sample_yaw(traj, sample_time, yaw, yaw_rate);

    laser_msgs::msg::ReferenceState ref;
    ref.pose.position.x = position.x();
    ref.pose.position.y = position.y();
    ref.pose.position.z = position.z();

    // The NMPC treats use_orientation=false as an identity yaw target.
    ref.pose.orientation = heading_to_quaternion(have_super_yaw ? yaw : fallback_yaw);

    ref.twist.linear.x = velocity.x();
    ref.twist.linear.y = velocity.y();
    ref.twist.linear.z = velocity.z();

    ref.use_position = true;
    ref.use_linear_velocity = true;
    ref.use_individual_thrust = false;

    ref.twist.angular.z = have_super_yaw ? yaw_rate : 0.0;
    ref.use_orientation = true;
    ref.use_angular_velocity = true;

    horizon.push_back(ref);
  }

  return true;
}

bool ControlManagerNode::sample_external_terminal_reference(
  const PolynomialTrajectory & traj, laser_msgs::msg::ReferenceState & ref) const
{
  if (traj.time_pos.empty()) {
    return false;
  }

  const double end_time = traj.start_wt_pos + trajectory_duration(traj.time_pos);
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  if (!sample_position(traj, end_time, position, velocity)) {
    return false;
  }

  double yaw = quaternion_to_heading(commanded_reference_.pose.orientation);
  double yaw_rate = 0.0;
  sample_yaw(traj, end_time, yaw, yaw_rate);

  ref = laser_msgs::msg::ReferenceState();
  ref.pose.position.x = position.x();
  ref.pose.position.y = position.y();
  ref.pose.position.z = position.z();
  ref.pose.orientation = heading_to_quaternion(yaw);
  ref.use_position = true;
  ref.use_orientation = true;
  ref.use_linear_velocity = true;
  ref.use_angular_velocity = true;
  ref.use_individual_thrust = false;
  return true;
}

void ControlManagerNode::minimum_jerk(double tau, double & s, double & ds_dtau)
{
  const double u = std::max(0.0, std::min(1.0, tau));
  const double u2 = u * u;
  const double u3 = u2 * u;
  const double u4 = u3 * u;
  const double u5 = u4 * u;
  s = 10.0 * u3 - 15.0 * u4 + 6.0 * u5;
  ds_dtau = 30.0 * u2 - 60.0 * u3 + 30.0 * u4;
}

void ControlManagerNode::build_hold_horizon(
  const laser_msgs::msg::ReferenceState & hold,
  std::vector<laser_msgs::msg::ReferenceState> & horizon) const
{
  auto ref = hold;
  ref.twist = geometry_msgs::msg::Twist();
  horizon.assign(static_cast<std::size_t>(acados_params_.N + 1), ref);
}

void ControlManagerNode::start_vertical_reference(double target_z, double speed)
{
  vertical_start_reference_ = commanded_reference_;
  vertical_start_reference_.twist = geometry_msgs::msg::Twist();
  vertical_target_z_ = target_z;
  // The quintic profile has maximum ds/dtau = 1.875.
  vertical_duration_ = std::max(
    acados_params_.dt,
    1.875 * std::abs(target_z - vertical_start_reference_.pose.position.z) / speed);
  vertical_time_started_ = false;
  set_hold_reference(vertical_start_reference_.pose);
  hold_reference_.pose.position.z = target_z;
  have_external_position_trajectory_ = false;
  have_external_rx_time_ = false;
  external_emergency_stop_ = false;
  stale_hold_active_ = false;
}

void ControlManagerNode::build_vertical_horizon(
  const rclcpp::Time & now, std::vector<laser_msgs::msg::ReferenceState> & horizon) const
{
  const double elapsed = std::max(0.0, (now - vertical_start_time_).seconds());
  const double z0 = vertical_start_reference_.pose.position.z;
  const double dz = vertical_target_z_ - z0;
  horizon.clear();
  horizon.reserve(static_cast<std::size_t>(acados_params_.N + 1));
  for (int i = 0; i <= acados_params_.N; ++i) {
    const double tau = (elapsed + i * acados_params_.dt) / vertical_duration_;
    double s, ds_dtau;
    minimum_jerk(tau, s, ds_dtau);
    auto ref = vertical_start_reference_;
    ref.pose.position.z = z0 + dz * s;
    ref.twist.linear.z = dz * ds_dtau / vertical_duration_;
    horizon.push_back(ref);
  }
}

bool ControlManagerNode::update_reference_horizon(const rclcpp::Time & now)
{
  if (!is_active_ || lock_control_inputs_ || !received_first_odometry_msg_) {
    return false;
  }
  if (external_emergency_stop_ && !emergency_hover_) {
    set_hold_reference(commanded_reference_.pose);
    emergency_hover_ = true;
  }

  if (emergency_hover_) {
    build_hold_horizon(hold_reference_, current_horizon_path_);
  } else if (requested_takeoff_ || requested_land_) {
    if (!vertical_time_started_) {
      vertical_start_time_ = now;
      vertical_time_started_ = true;
    }
    build_vertical_horizon(now, current_horizon_path_);
  } else if (takeoff_done_) {
    const bool ended = have_external_position_trajectory_ &&
      now.seconds() >= latest_external_trajectory_.start_wt_pos +
      trajectory_duration(latest_external_trajectory_.time_pos);
    const double age = have_external_rx_time_ ?
      std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_external_rx_steady_).count() :
      std::numeric_limits<double>::infinity();
    if (ended) {
      if (!sample_external_terminal_reference(latest_external_trajectory_, hold_reference_)) {
        return false;
      }
      build_hold_horizon(hold_reference_, current_horizon_path_);
      if (external_goal_reached(hold_reference_)) {
        have_external_position_trajectory_ = false;
        diagnostics_.have_goal = false;
        finish_rmse_run();
      } else {
        diagnostics_.have_goal = true;
        start_rmse_run_if_needed();
      }
    } else if (have_external_position_trajectory_ && age <= external_timeout_sec_) {
      if (!build_external_horizon(
          latest_external_trajectory_, now.seconds(),
          current_horizon_path_))
      {
        return false;
      }
      stale_hold_active_ = false;
      start_rmse_run_if_needed();
      diagnostics_.have_goal = true;
    } else {
      if (have_external_position_trajectory_) {
        if (!hold_on_stale_) {
          RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 2000, "External trajectory stale; output suppressed");
          return false;
        }
        if (!stale_hold_active_) {
          set_hold_reference(commanded_reference_.pose);
          stale_hold_active_ = true;
          diagnostics_.have_goal = false;
          finish_rmse_run();
          RCLCPP_WARN(get_logger(), "External trajectory stale; holding last commanded reference");
        }
      }
      build_hold_horizon(hold_reference_, current_horizon_path_);
    }
  } else {
    return false;
  }

  check_safe_area();
  if (emergency_hover_) {
    build_hold_horizon(hold_reference_, current_horizon_path_);
    diagnostics_.have_goal = false;
    finish_rmse_run();
  }
  commanded_reference_ = current_horizon_path_.front();
  return true;
}

double ControlManagerNode::euclidean_distance(
  const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2) const
{
  return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2) + std::pow(p1.z - p2.z, 2));
}

double ControlManagerNode::normalize_heading(double heading)
{
  double aux = std::fmod(heading + M_PI, kTwoPi);
  if (aux < 0.0) {
    aux += kTwoPi;
  }
  return aux - M_PI;
}

double ControlManagerNode::quaternion_to_heading(const geometry_msgs::msg::Quaternion & q)
{
  Eigen::Quaterniond qe(q.w, q.x, q.y, q.z);
  if (qe.norm() < 1e-9) {
    return 0.0;
  }
  qe.normalize();
  const Eigen::Vector3d heading_vector = qe * Eigen::Vector3d::UnitX();
  return std::atan2(heading_vector.y(), heading_vector.x());
}

geometry_msgs::msg::Quaternion ControlManagerNode::heading_to_quaternion(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  const double half = 0.5 * yaw;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(half);
  q.w = std::cos(half);
  return q;
}

double ControlManagerNode::heading_error(double current, double target)
{
  return std::remainder(target - current, kTwoPi);
}

void ControlManagerNode::set_hold_reference(const geometry_msgs::msg::Pose & pose)
{
  hold_reference_ = laser_msgs::msg::ReferenceState();
  hold_reference_.pose = pose;
  hold_reference_.pose.orientation = heading_to_quaternion(quaternion_to_heading(pose.orientation));
  hold_reference_.twist = geometry_msgs::msg::Twist();
  hold_reference_.use_position = true;
  hold_reference_.use_orientation = true;
  hold_reference_.use_linear_velocity = true;
  hold_reference_.use_angular_velocity = true;
  hold_reference_.use_individual_thrust = false;
}

void ControlManagerNode::set_emergency_hold_inside_safe_area()
{
  geometry_msgs::msg::Pose pose = commanded_reference_.pose;

  if (safe_area_.x.size() == 2) {
    pose.position.x =
      std::max(safe_area_.x[0] + 0.05, std::min(safe_area_.x[1] - 0.05, pose.position.x));
  }
  if (safe_area_.y.size() == 2) {
    pose.position.y =
      std::max(safe_area_.y[0] + 0.05, std::min(safe_area_.y[1] - 0.05, pose.position.y));
  }
  if (safe_area_.z.size() == 2) {
    pose.position.z =
      std::max(safe_area_.z[0] + 0.05, std::min(safe_area_.z[1] - 0.05, pose.position.z));
  }

  set_hold_reference(pose);
}

void ControlManagerNode::check_safe_area()
{
  if (!safe_area_.enabled || emergency_hover_ || current_horizon_path_.empty()) {
    return;
  }

  // Preserve the original control manager semantics: inspect the first reference point.
  const auto & p = current_horizon_path_.front().pose.position;
  bool path_is_ok = true;

  if (p.x < safe_area_.x[0] || p.x > safe_area_.x[1]) {
    path_is_ok = false;
  }
  if (p.y < safe_area_.y[0] || p.y > safe_area_.y[1]) {
    path_is_ok = false;
  }

  if (requested_takeoff_ || requested_land_) {
    // Original code only enforced the upper z bound during takeoff/landing.
    if (p.z > safe_area_.z[1]) {
      path_is_ok = false;
    }
  } else if (p.z < safe_area_.z[0] || p.z > safe_area_.z[1]) {
    path_is_ok = false;
  }

  if (!path_is_ok) {
    RCLCPP_WARN(
      get_logger(),
      "Reference violates safe_area. Entering emergency hold inside configured bounds.");
    set_emergency_hold_inside_safe_area();
    emergency_hover_ = true;
  }
}

bool ControlManagerNode::estimate_mass()
{
  if (
    start_mass_estimation_ && std::abs(odometry_.twist.twist.linear.z) < kMassEstimateVzThreshold)
  {
    mass_estimation_time_start_ = get_clock()->now();
    start_mass_estimation_ = false;
  }

  if (start_mass_estimation_) {
    return false;
  }

  if ((get_clock()->now() - mass_estimation_time_start_).seconds() < kMassEstimateDurationSec) {
    return false;
  }

  estimated_mass_ =
    0.98 * estimated_mass_for_detect_landing_ + 0.02 * controller_multirotor_params_.mass;
  diagnostics_.estimated_mass = estimated_mass_;
  nmpc_controller_.setMass(estimated_mass_);

  RCLCPP_INFO(get_logger(), "Estimated calibrated mass: %.3f kg", estimated_mass_);
  return true;
}

void ControlManagerNode::start_rmse_run_if_needed()
{
  if (!rmse_run_active_) {
    estimated_rmse_.reset();
    diagnostics_.metrics.rmse = NAN;
    diagnostics_.metrics.std = NAN;
    calculate_rmse_ = false;
    rmse_run_active_ = true;
    RCLCPP_INFO(get_logger(), "Starting external planner tracking RMSE run");
  }
}

void ControlManagerNode::finish_rmse_run()
{
  if (rmse_run_active_) {
    calculate_rmse_ = true;
    rmse_run_active_ = false;
  }
}

bool ControlManagerNode::external_goal_reached(const laser_msgs::msg::ReferenceState & goal) const
{
  const double pos_error = euclidean_distance(odometry_.pose.pose.position, goal.pose.position);
  const double yaw_error = std::abs(
    heading_error(
      quaternion_to_heading(odometry_.pose.pose.orientation),
      quaternion_to_heading(goal.pose.orientation)));
  return pos_error < goal_position_tolerance_ && yaw_error < goal_heading_tolerance_;
}

void ControlManagerNode::external_loop_timer_callback()
{
  if (!is_active_ || lock_control_inputs_ || !received_first_odometry_msg_) {
    return;
  }

  const auto start_iteration = std::chrono::high_resolution_clock::now();
  const auto ros_now = get_clock()->now();

  // Landing rampdown is retained exactly as a terminal control phase and has priority over NMPC.
  if (land_rampdown_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000, "Land ramp down: %.3f", land_start_rampdown_);

    if (angular_rates_and_thrust_mode_) {
      laser_msgs::msg::AttitudeRatesAndThrust msg;
      msg.total_thrust_normalized = land_start_rampdown_;
      msg.roll_rate = 0.0;
      msg.pitch_rate = 0.0;
      msg.yaw_rate = 0.0;
      pub_attitude_rates_and_thrust_reference_->publish(msg);
    } else {
      laser_msgs::msg::MotorSpeed msg;
      for (int i = 0; i < controller_multirotor_params_.n_motors; ++i) {
        msg.data.push_back(land_start_rampdown_);
      }
      pub_motor_speed_reference_->publish(msg);
    }

    diagnostics_.last_control_input.unit_of_measurement = "N";
    diagnostics_.last_control_input.data = std::vector<double>(
      static_cast<std::size_t>(controller_multirotor_params_.n_motors),
      laser_uav_controllers::throtleToThrust(
        controller_multirotor_params_.motor_curve_a, controller_multirotor_params_.motor_curve_b,
        land_start_rampdown_));
    diagnostics_.current_control_rampdown = land_start_rampdown_;

    if (land_start_rampdown_ <= 0.0) {
      land_start_rampdown_ = 0.0;
      land_rampdown_ = false;
      lock_control_inputs_ = true;
      RCLCPP_INFO(get_logger(), "Landing rampdown finished; control inputs locked.");
    } else {
      land_start_rampdown_ = std::max(0.0, land_start_rampdown_ - land_increment_rampdown_);
    }

    diagnostics_.control_iteration_duration_ms =
      std::chrono::duration<double, std::milli>(
      std::chrono::high_resolution_clock::now() - start_iteration)
      .count();
    return;
  }

  diagnostics_.current_control_rampdown = NAN;

  if (!update_reference_horizon(ros_now)) {
    return;
  }

  nmpc_solution_ = nmpc_controller_.getCorrection(current_horizon_path_, odometry_);
  have_nmpc_solution_ = true;
  diagnostics_.ocp_elapsed_time_ms = nmpc_controller_.getOcpElapsedTime();
  diagnostics_.header.stamp = ros_now;
  diagnostics_.reference_horizon = current_horizon_path_;
  if (publish_reference_path_) {
    publish_reference_path(current_horizon_path_);
  }

  if (rmse_run_active_) {
    estimated_rmse_.pushEstimated(odometry_.pose.pose.position);
    estimated_rmse_.pushReference(commanded_reference_.pose.position);
    diagnostics_.metrics.rmse = NAN;
    diagnostics_.metrics.std = NAN;
  }

  if (angular_rates_and_thrust_mode_) {
    estimated_mass_for_detect_landing_ = nmpc_solution_.second.sum() / kGravity;

    laser_msgs::msg::AttitudeRatesAndThrust msg;
    msg.total_thrust_normalized = laser_uav_controllers::thrustToThrotle(
      controller_multirotor_params_.motor_curve_a, controller_multirotor_params_.motor_curve_b,
      nmpc_solution_.second.sum() / controller_multirotor_params_.n_motors);
    msg.roll_rate = nmpc_solution_.first(0);
    msg.pitch_rate = nmpc_solution_.first(1);
    msg.yaw_rate = nmpc_solution_.first(2);

    diagnostics_.last_control_input.unit_of_measurement = "N";
    diagnostics_.last_control_input.data = nmpc_controller_.getLastIndividualThrust();
    pub_attitude_rates_and_thrust_reference_->publish(msg);
  }

  // Takeoff completion + original mass-estimation concept.
  if (requested_takeoff_ && !emergency_hover_) {
    const double z_error = std::abs(odometry_.pose.pose.position.z - takeoff_height_);
    if ((ros_now - vertical_start_time_).seconds() >= vertical_duration_ &&
      z_error < kTakeoffPositionTolerance)
    {
      if (estimate_mass()) {
        requested_takeoff_ = false;
        takeoff_done_ = true;
        land_done_ = false;
        diagnostics_.is_fly = true;
        diagnostics_.have_goal = false;
        RCLCPP_INFO(
          get_logger(), "Takeoff done; waiting for/accepting external planner trajectories.");
      }
    }
  }

  // Touchdown detection and rampdown retained from the original control manager.
  if (requested_land_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2500, "Current estimated mass for landing detection: %.3f kg",
      estimated_mass_for_detect_landing_);

    if (estimated_mass_for_detect_landing_ <= estimated_mass_ * land_threshold_detect_) {
      requested_land_ = false;
      land_done_ = true;
      diagnostics_.have_goal = false;
      diagnostics_.is_fly = false;
      land_rampdown_ = true;
      land_start_rampdown_ = laser_uav_controllers::thrustToThrotle(
        controller_multirotor_params_.motor_curve_a, controller_multirotor_params_.motor_curve_b,
        (estimated_mass_ * kGravity) / controller_multirotor_params_.n_motors);
      RCLCPP_INFO(
        get_logger(), "Landing detected at estimated supported mass %.3f kg; starting rampdown.",
        estimated_mass_for_detect_landing_);
    }
  }

  diagnostics_.control_iteration_duration_ms =
    std::chrono::duration<double, std::milli>(
    std::chrono::high_resolution_clock::now() - start_iteration)
    .count();
}

void ControlManagerNode::internal_loop_timer_callback()
{
  if (!is_active_ || angular_rates_and_thrust_mode_ || lock_control_inputs_) {
    return;
  }
  if (land_rampdown_ || !have_nmpc_solution_) {
    return;
  }

  Eigen::VectorXd indi_thrust = indi_controller_.getCorrection(
    angular_acceleration_estimated_, motor_speed_estimated_, nmpc_solution_.second,
    nmpc_solution_.first);

  estimated_mass_for_detect_landing_ = indi_thrust.sum() / kGravity;

  diagnostics_.last_control_input.unit_of_measurement = "N";
  diagnostics_.last_control_input.data =
    std::vector<double>(indi_thrust.data(), indi_thrust.data() + indi_thrust.size());

  laser_msgs::msg::MotorSpeed msg;
  for (Eigen::Index i = 0; i < indi_thrust.size(); ++i) {
    msg.data.push_back(
      laser_uav_controllers::thrustToThrotle(
        controller_multirotor_params_.motor_curve_a, controller_multirotor_params_.motor_curve_b,
        indi_thrust(i), controller_multirotor_params_.thrust_max,
        controller_multirotor_params_.thrust_min));
  }

  pub_motor_speed_reference_->publish(msg);
}

void ControlManagerNode::diagnostics_timer_callback()
{
  if (!is_active_) {
    return;
  }
  update_benchmark(get_clock()->now());

  if (calculate_rmse_) {
    const auto result = estimated_rmse_.calculate();
    diagnostics_.metrics.rmse = result.first;
    diagnostics_.metrics.std = result.second;
    estimated_rmse_.reset();
    calculate_rmse_ = false;
  }

  diagnostics_.estimated_mass = estimated_mass_;
  diagnostics_.header.stamp = get_clock()->now();
  diagnostics_.header.frame_id = "";
  pub_diagnostics_->publish(diagnostics_);
}

void ControlManagerNode::benchmark_goal_callback(const geometry_msgs::msg::PoseStamped &)
{
  if (!is_active_ || !benchmark_.enabled || !received_first_odometry_msg_ ||
    !takeoff_done_ || requested_takeoff_ || requested_land_)
  {
    return;
  }
  cancel_benchmark();
  benchmark_.active = true;
  ++benchmark_.run_id;
  benchmark_.start_time = get_clock()->now();
  const auto & p = odometry_.pose.pose.position;
  benchmark_.start_position = Eigen::Vector3d(p.x, p.y, p.z);
  benchmark_.last_position = benchmark_.start_position;
  benchmark_.distance = 0.0;
  benchmark_.have_trajectory = false;
  benchmark_.have_heartbeat = false;
  RCLCPP_INFO(
    get_logger(), "BENCHMARK START | run=%lu", static_cast<unsigned long>(benchmark_.run_id));
}

void ControlManagerNode::cancel_benchmark()
{
  if (benchmark_.active) {
    RCLCPP_WARN(
      get_logger(), "BENCHMARK CANCELLED | run=%lu", static_cast<unsigned long>(benchmark_.run_id));
  }
  benchmark_.active = false;
}

void ControlManagerNode::update_benchmark(const rclcpp::Time & now)
{
  if (!benchmark_.active || !benchmark_.have_trajectory || !benchmark_.have_heartbeat ||
    !takeoff_done_ || requested_land_ || now.seconds() < benchmark_.trajectory_end)
  {
    return;
  }
  const double heartbeat_age = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - benchmark_.last_heartbeat).count();
  if (heartbeat_age <= benchmark_.heartbeat_timeout ||
    euclidean_distance(odometry_.pose.pose.position, benchmark_.terminal.pose.position) >=
    goal_position_tolerance_)
  {
    return;
  }
  if (benchmark_.require_heading && std::abs(
      heading_error(
        quaternion_to_heading(odometry_.pose.pose.orientation),
        quaternion_to_heading(benchmark_.terminal.pose.orientation))) >= goal_heading_tolerance_)
  {
    return;
  }

  const double duration = std::max(0.0, (now - benchmark_.start_time).seconds());
  const double speed = duration > 1e-9 ? benchmark_.distance / duration : 0.0;
  benchmark_.active = false;
  RCLCPP_INFO(
    get_logger(), "BENCHMARK RESULT | run=%lu | average_speed=%.4f m/s | distance=%.4f m | "
    "total_time=%.4f s | waypoints=1",
    static_cast<unsigned long>(benchmark_.run_id), speed, benchmark_.distance, duration);
  if (benchmark_.csv_path.empty()) {
    return;
  }
  std::ofstream output(benchmark_.csv_path, std::ios::app);
  if (!output.is_open()) {
    RCLCPP_ERROR(get_logger(), "Could not open benchmark CSV: %s", benchmark_.csv_path.c_str());
    return;
  }
  if (output.tellp() == 0) {
    output << "run_id,start_time_s,end_time_s,total_time_s,distance_m,average_speed_mps,"
      "waypoint_count,start_x,start_y,start_z,end_x,end_y,end_z\n";
  }
  const auto & p = odometry_.pose.pose.position;
  output << benchmark_.run_id << ',' << std::fixed << std::setprecision(6)
         << benchmark_.start_time.seconds() << ',' << now.seconds() << ',' << duration << ','
         << benchmark_.distance << ',' << speed << ",1,"
         << benchmark_.start_position.x() << ',' << benchmark_.start_position.y() << ','
         << benchmark_.start_position.z() << ',' << p.x << ',' << p.y << ',' << p.z << '\n';
  output.flush();
  if (!output) {
    RCLCPP_ERROR(get_logger(), "Could not write benchmark CSV: %s", benchmark_.csv_path.c_str());
  }
}

void ControlManagerNode::publish_reference_path(
  const std::vector<laser_msgs::msg::ReferenceState> & horizon)
{
  nav_msgs::msg::Path path;
  path.header.stamp = get_clock()->now();
  path.header.frame_id = reference_frame_id_;
  path.poses.reserve(horizon.size());
  for (const auto & ref : horizon) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose = ref.pose;
    path.poses.push_back(pose);
  }
  pub_reference_path_->publish(path);
}

}  // namespace laser_uav_managers

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(laser_uav_managers::ControlManagerNode)
