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
  RCLCPP_INFO(get_logger(), "Creating LASER reference horizon + NMPC lifecycle ControlManagerNode");

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
  declare_parameter("reference.timeout", rclcpp::ParameterValue(0.5));
  declare_parameter("goal.position_tolerance", rclcpp::ParameterValue(0.15));
  declare_parameter("goal.heading_tolerance", rclcpp::ParameterValue(0.15));
  declare_parameter("debug.publish_reference_path", rclcpp::ParameterValue(true));
  declare_parameter("benchmark_metrics.enabled", rclcpp::ParameterValue(true));
  declare_parameter(
    "benchmark_metrics.csv_path", std::string("/tmp/laser_uav_control_metrics.csv"));
}

CallbackReturn ControlManagerNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring LASER reference horizon + NMPC control manager");

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
    get_logger(), "Configured: N=%d dt=%.4f external=%.1fHz mode=%s reference timeout=%.2fs",
    acados_params_.N, acados_params_.dt, rate_external_loop_control_,
    angular_rates_and_thrust_mode_ ? "body_rates+thrust" : "individual_thrust+INDI",
    reference_timeout_sec_);

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

  pub_control_state_->on_activate();
  is_active_ = true;
  return CallbackReturn::SUCCESS;
}

CallbackReturn ControlManagerNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating");
  cancel_benchmark();
  lock_control_inputs_ = true;
  takeoff_done_ = false;
  have_external_reference_ = false;
  publish_control_state();
  pub_control_state_->on_deactivate();

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
  sub_reference_horizon_.reset();
  sub_planning_diagnostics_.reset();
  cancel_benchmark();

  pub_attitude_rates_and_thrust_reference_.reset();
  pub_motor_speed_reference_.reset();
  pub_diagnostics_.reset();
  pub_reference_path_.reset();
  pub_control_state_.reset();

  current_horizon_path_.clear();
  have_external_reference_ = false;
  flight_id_ = accepted_mission_id_ = closed_mission_id_ = 0;
  received_first_odometry_msg_ = false;
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
      static_cast<std::size_t>(4 * controller_multirotor_params_.n_motors)) {
    controller_multirotor_params_.G1 =
      Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        aux.as_double_array().data(), 4, controller_multirotor_params_.n_motors);
  }

  get_parameter("multirotor_parameters.G2", aux);
  if (
    controller_multirotor_params_.n_motors > 0 &&
    aux.as_double_array().size() >=
      static_cast<std::size_t>(4 * controller_multirotor_params_.n_motors)) {
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

  get_parameter("reference.timeout", reference_timeout_sec_);
  get_parameter("goal.position_tolerance", goal_position_tolerance_);
  get_parameter("goal.heading_tolerance", goal_heading_tolerance_);
  get_parameter("debug.publish_reference_path", publish_reference_path_);
  get_parameter("benchmark_metrics.enabled", benchmark_.enabled);
  get_parameter("benchmark_metrics.csv_path", benchmark_.csv_path);
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
  if (reference_timeout_sec_ <= 0.0) {
    throw std::runtime_error("reference.timeout must be > 0");
  }
  if (
    safe_area_.enabled &&
    (safe_area_.x.size() != 2 || safe_area_.y.size() != 2 || safe_area_.z.size() != 2)) {
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

  sub_reference_horizon_ = create_subscription<ReferenceHorizon>(
    "reference_horizon_in", trajectory_qos,
    std::bind(&ControlManagerNode::reference_horizon_callback, this, std::placeholders::_1));
  sub_planning_diagnostics_ = create_subscription<PlanningDiagnostics>(
    "planning_diagnostics_in", rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&ControlManagerNode::planning_diagnostics_callback, this, std::placeholders::_1));

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
  pub_reference_path_ = create_publisher<nav_msgs::msg::Path>("reference_path_out", 1);
  pub_control_state_ = create_publisher<laser_msgs::msg::UavControlState>(
    "control_state_out", rclcpp::QoS(1).reliable().transient_local());
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
  RCLCPP_INFO(get_logger(), "Configuring NMPC/INDI");

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
  const std::array<double, 13> state{p.x, p.y, p.z, q.w, q.x, q.y, q.z,
                                     v.x, v.y, v.z, w.x, w.y, w.z};
  if (
    !std::all_of(state.begin(), state.end(), [](double x) { return std::isfinite(x); }) ||
    Eigen::Quaterniond(q.w, q.x, q.y, q.z).norm() < 1e-9) {
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

void ControlManagerNode::reference_horizon_callback(const ReferenceHorizon & msg)
{
  if (
    !is_active_ || !takeoff_done_ || requested_land_ || land_rampdown_ ||
    msg.flight_id != flight_id_ || msg.mission_id <= closed_mission_id_ ||
    msg.mission_id < accepted_mission_id_ || msg.header.frame_id != reference_frame_id_ ||
    msg.header.stamp.sec < 0 || msg.header.stamp.nanosec >= 1000000000U || msg.start_time.sec < 0 ||
    msg.start_time.nanosec >= 1000000000U) {
    return;
  }
  std::string reason;
  const auto now = get_clock()->now();
  const double age = (now - rclcpp::Time(msg.header.stamp)).seconds();
  // Independent /clock subscriptions may differ by one outer control cycle.
  const double clock_tolerance = 1.0 / rate_external_loop_control_;
  if (age < -clock_tolerance || age > reference_timeout_sec_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Rejected horizon publication stamp | mission=%lu | age=%.2f ms | allowed=[-%.2f, %.2f] ms",
      static_cast<unsigned long>(msg.mission_id), age * 1000.0, clock_tolerance * 1000.0,
      reference_timeout_sec_ * 1000.0);
    return;
  }
  if (!validate_reference_horizon(msg, reason)) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "Rejected horizon: %s", reason.c_str());
    return;
  }
  if ((now - rclcpp::Time(msg.start_time)).seconds() < msg.time_from_start.front()) {
    return;
  }
  if (have_external_reference_ && msg.mission_id == accepted_mission_id_) {
    if (
      rclcpp::Time(msg.header.stamp) <= rclcpp::Time(latest_reference_.header.stamp) ||
      (latest_reference_.terminal && !msg.terminal)) {
      return;
    }
  }
  if (msg.mission_id > accepted_mission_id_) {
    finish_rmse_run();
    emergency_hover_ = false;
  }
  latest_reference_ = msg;
  accepted_mission_id_ = msg.mission_id;
  have_external_reference_ = true;
  last_reference_rx_ = std::chrono::steady_clock::now();
}

bool ControlManagerNode::validate_reference_horizon(
  const ReferenceHorizon & msg, std::string & reason) const
{
  const auto & refs = msg.reference_horizon;
  const auto & times = msg.time_from_start;
  if (refs.size() < 2 || refs.size() > 65536 || refs.size() != times.size()) {
    reason = "reference samples require matching time offsets";
    return false;
  }
  for (std::size_t i = 0; i < refs.size(); ++i) {
    if (!std::isfinite(times[i]) || times[i] < 0.0 || (i > 0 && times[i] <= times[i - 1])) {
      reason = "sample times must increase strictly";
      return false;
    }
    const auto & r = refs[i];
    const auto & p = r.pose.position;
    const auto & q = r.pose.orientation;
    const auto & v = r.twist.linear;
    const auto & w = r.twist.angular;
    for (double x : {p.x, p.y, p.z, q.w, q.x, q.y, q.z, v.x, v.y, v.z, w.x, w.y, w.z}) {
      if (!std::isfinite(x)) {
        reason = "non-finite reference";
        return false;
      }
    }
    const double norm = Eigen::Quaterniond(q.w, q.x, q.y, q.z).norm();
    if (r.use_orientation && (!std::isfinite(norm) || norm < 1e-9)) {
      reason = "invalid reference quaternion";
      return false;
    }
    const auto & first = refs.front();
    if (
      r.use_position != first.use_position || r.use_orientation != first.use_orientation ||
      r.use_linear_velocity != first.use_linear_velocity ||
      r.use_angular_velocity != first.use_angular_velocity ||
      r.use_individual_thrust != first.use_individual_thrust) {
      reason = "reference flags change inside the trajectory window";
      return false;
    }
    if (
      r.use_individual_thrust &&
      (r.individual_thrust.data.size() !=
         static_cast<std::size_t>(controller_multirotor_params_.n_motors) ||
       r.individual_thrust.unit_of_measurement != "N")) {
      reason = "individual thrust dimension or unit mismatch";
      return false;
    }
    for (double x : r.individual_thrust.data) {
      if (!std::isfinite(x) || x < 0.0) {
        reason = "invalid individual thrust";
        return false;
      }
    }
    if (msg.terminal) {
      if (
        euclidean_distance(p, first.pose.position) > 1e-9 ||
        std::hypot(std::hypot(v.x, v.y), v.z) > 1e-9 ||
        std::hypot(std::hypot(w.x, w.y), w.z) > 1e-9) {
        reason = "terminal reference must be stationary";
        return false;
      }
      if (r.use_orientation) {
        const auto & q0 = first.pose.orientation;
        if (
          std::abs(Eigen::Quaterniond(q.w, q.x, q.y, q.z)
                     .normalized()
                     .dot(Eigen::Quaterniond(q0.w, q0.x, q0.y, q0.z).normalized())) < 1.0 - 1e-9) {
          reason = "terminal attitude must be constant";
          return false;
        }
      }
    }
  }
  return true;
}

laser_msgs::msg::ReferenceState ControlManagerNode::interpolate_reference(
  const laser_msgs::msg::ReferenceState & a, const laser_msgs::msg::ReferenceState & b,
  double alpha)
{
  auto r = a;
  const auto lerp = [alpha](double x, double y) { return x + alpha * (y - x); };
  r.pose.position.x = lerp(a.pose.position.x, b.pose.position.x);
  r.pose.position.y = lerp(a.pose.position.y, b.pose.position.y);
  r.pose.position.z = lerp(a.pose.position.z, b.pose.position.z);
  r.twist.linear.x = lerp(a.twist.linear.x, b.twist.linear.x);
  r.twist.linear.y = lerp(a.twist.linear.y, b.twist.linear.y);
  r.twist.linear.z = lerp(a.twist.linear.z, b.twist.linear.z);
  r.twist.angular.x = lerp(a.twist.angular.x, b.twist.angular.x);
  r.twist.angular.y = lerp(a.twist.angular.y, b.twist.angular.y);
  r.twist.angular.z = lerp(a.twist.angular.z, b.twist.angular.z);
  if (r.use_orientation) {
    const auto & qa = a.pose.orientation;
    const auto & qb = b.pose.orientation;
    const Eigen::Quaterniond q =
      Eigen::Quaterniond(qa.w, qa.x, qa.y, qa.z)
        .normalized()
        .slerp(alpha, Eigen::Quaterniond(qb.w, qb.x, qb.y, qb.z).normalized());
    r.pose.orientation.w = q.w();
    r.pose.orientation.x = q.x();
    r.pose.orientation.y = q.y();
    r.pose.orientation.z = q.z();
  }
  if (r.use_individual_thrust) {
    for (std::size_t i = 0; i < r.individual_thrust.data.size(); ++i) {
      r.individual_thrust.data[i] = lerp(a.individual_thrust.data[i], b.individual_thrust.data[i]);
    }
  }
  return r;
}

bool ControlManagerNode::sample_reference_horizon(
  const ReferenceHorizon & msg, double elapsed,
  std::vector<laser_msgs::msg::ReferenceState> & output) const
{
  const auto & refs = msg.reference_horizon;
  const auto & times = msg.time_from_start;
  if (elapsed < times.front() - 1e-6) {
    return false;
  }
  if (msg.terminal) {
    output.assign(
      static_cast<std::size_t>(acados_params_.N + 1),
      interpolate_reference(refs.front(), refs.front(), 0.0));
    return true;
  }
  if (elapsed + acados_params_.N * acados_params_.dt > times.back() + 1e-9) {
    return false;
  }
  output.clear();
  output.reserve(static_cast<std::size_t>(acados_params_.N + 1));
  for (int i = 0; i <= acados_params_.N; ++i) {
    const double time = std::clamp(elapsed + i * acados_params_.dt, times.front(), times.back());
    const auto upper = std::upper_bound(times.begin(), times.end(), time);
    if (upper == times.end()) {
      output.push_back(interpolate_reference(refs.back(), refs.back(), 0.0));
    } else {
      const std::size_t hi = static_cast<std::size_t>(upper - times.begin());
      const std::size_t lo = hi - 1;
      output.push_back(
        interpolate_reference(refs[lo], refs[hi], (time - times[lo]) / (times[hi] - times[lo])));
    }
  }
  return true;
}

void ControlManagerNode::latch_reference_hold()
{
  set_hold_reference(commanded_reference_.pose);
  have_external_reference_ = false;
  closed_mission_id_ = std::max(closed_mission_id_, accepted_mission_id_);
  emergency_hover_ = true;
  diagnostics_.have_goal = false;
  finish_rmse_run();
  cancel_benchmark();
}

void ControlManagerNode::publish_control_state()
{
  if (!received_first_odometry_msg_) {
    return;
  }
  laser_msgs::msg::UavControlState msg;
  msg.header.stamp = get_clock()->now();
  msg.header.frame_id = reference_frame_id_;
  msg.flight_id = flight_id_;
  msg.mission_id = std::max(accepted_mission_id_, closed_mission_id_);
  msg.ready_for_tracking = is_active_ && takeoff_done_ && !requested_land_ && !land_rampdown_;
  msg.emergency_hold = emergency_hover_;
  msg.horizon_length = static_cast<uint32_t>(acados_params_.N);
  msg.horizon_dt = acados_params_.dt;
  msg.reference = commanded_reference_;
  msg.estimated_mass = estimated_mass_;
  pub_control_state_->publish(msg);
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
  flight_id_ = static_cast<uint64_t>(std::chrono::system_clock::now().time_since_epoch().count());
  accepted_mission_id_ = closed_mission_id_ = 0;
  benchmark_.last_mission_id = 0;
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
  set_hold_reference(commanded_reference_.pose);
  vertical_start_reference_ = hold_reference_;
  vertical_target_z_ = target_z;
  // The quintic profile has maximum ds/dtau = 1.875.
  vertical_duration_ = std::max(
    acados_params_.dt,
    1.875 * std::abs(target_z - vertical_start_reference_.pose.position.z) / speed);
  vertical_time_started_ = false;
  hold_reference_.pose.position.z = target_z;
  have_external_reference_ = false;
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
  if (emergency_hover_) {
    build_hold_horizon(hold_reference_, current_horizon_path_);
  } else if (requested_takeoff_ || requested_land_) {
    if (!vertical_time_started_) {
      vertical_start_time_ = now;
      vertical_time_started_ = true;
    }
    build_vertical_horizon(now, current_horizon_path_);
  } else if (takeoff_done_) {
    if (have_external_reference_) {
      const double age =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - last_reference_rx_)
          .count();
      const double elapsed = (now - rclcpp::Time(latest_reference_.start_time)).seconds();
      if (
        (!latest_reference_.terminal && age > reference_timeout_sec_) ||
        !sample_reference_horizon(latest_reference_, elapsed, current_horizon_path_)) {
        latch_reference_hold();
        build_hold_horizon(hold_reference_, current_horizon_path_);
      } else {
        hold_reference_ = current_horizon_path_.front();
        diagnostics_.have_goal =
          !latest_reference_.terminal || !external_goal_reached(current_horizon_path_.front());
        if (diagnostics_.have_goal) {
          start_rmse_run_if_needed();
        } else {
          finish_rmse_run();
        }
      }
    } else {
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
    start_mass_estimation_ && std::abs(odometry_.twist.twist.linear.z) < kMassEstimateVzThreshold) {
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
  const double yaw_error = std::abs(heading_error(
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
    if (
      (ros_now - vertical_start_time_).seconds() >= vertical_duration_ &&
      z_error < kTakeoffPositionTolerance) {
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
    msg.data.push_back(laser_uav_controllers::thrustToThrotle(
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
  publish_control_state();
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

void ControlManagerNode::planning_diagnostics_callback(const PlanningDiagnostics & msg)
{
  if (
    !is_active_ || msg.flight_id != flight_id_ || msg.mission_id == 0 ||
    msg.header.frame_id != reference_frame_id_ || msg.header.stamp.sec < 0 ||
    msg.header.stamp.nanosec >= 1000000000U || msg.start_time.sec < 0 ||
    msg.start_time.nanosec >= 1000000000U || msg.waypoint_count == 0 ||
    !std::isfinite(msg.goal.position.x) || !std::isfinite(msg.goal.position.y) ||
    !std::isfinite(msg.goal.position.z) || !std::isfinite(msg.goal.heading)) {
    return;
  }
  if (msg.state == PlanningDiagnostics::ABORTED || msg.state == PlanningDiagnostics::FAILED) {
    closed_mission_id_ = std::max(closed_mission_id_, msg.mission_id);
    if (msg.mission_id >= accepted_mission_id_ && takeoff_done_) {
      latch_reference_hold();
    }
    if (benchmark_.active && benchmark_.mission_id == msg.mission_id) {
      cancel_benchmark();
    }
    return;
  }
  if (
    msg.mission_id <= closed_mission_id_ || !takeoff_done_ || requested_land_ ||
    msg.header.frame_id != reference_frame_id_) {
    return;
  }
  if (
    (msg.state == PlanningDiagnostics::GENERATING || msg.state == PlanningDiagnostics::TRACKING) &&
    benchmark_.enabled && msg.mission_id > benchmark_.last_mission_id) {
    cancel_benchmark();
    benchmark_.last_mission_id = msg.mission_id;
    benchmark_.mission_id = msg.mission_id;
    benchmark_.active = true;
    benchmark_.completed = false;
    ++benchmark_.run_id;
    benchmark_.waypoint_count = msg.waypoint_count;
    benchmark_.goal = msg.goal;
    benchmark_.start_time = rclcpp::Time(msg.start_time);
    const auto & p = odometry_.pose.pose.position;
    benchmark_.start_position = Eigen::Vector3d(p.x, p.y, p.z);
    benchmark_.last_position = benchmark_.start_position;
    benchmark_.distance = 0.0;
  }
  if (
    msg.state == PlanningDiagnostics::COMPLETED && benchmark_.active &&
    msg.mission_id == benchmark_.mission_id) {
    benchmark_.completed = true;
  }
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
  if (!benchmark_.active || !benchmark_.completed || !takeoff_done_ || requested_land_) {
    return;
  }
  laser_msgs::msg::ReferenceState goal;
  goal.pose.position = benchmark_.goal.position;
  goal.pose.orientation = heading_to_quaternion(benchmark_.goal.heading);
  if (!external_goal_reached(goal)) {
    return;
  }

  const double duration = std::max(0.0, (now - benchmark_.start_time).seconds());
  const double speed = duration > 1e-9 ? benchmark_.distance / duration : 0.0;
  benchmark_.active = false;
  RCLCPP_INFO(
    get_logger(),
    "BENCHMARK RESULT | run=%lu | average_speed=%.4f m/s | distance=%.4f m | "
    "total_time=%.4f s | waypoints=%u",
    static_cast<unsigned long>(benchmark_.run_id), speed, benchmark_.distance, duration,
    benchmark_.waypoint_count);
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
         << benchmark_.distance << ',' << speed << ',' << benchmark_.waypoint_count << ','
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
