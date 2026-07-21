#include "laser_uav_managers/control_manager_node.hpp"

namespace laser_uav_managers
{
/* ControlManagerNode() //{ */
ControlManagerNode::ControlManagerNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("control_manager", "", options)
{
  RCLCPP_INFO(get_logger(), "Creating ControlManagerNode");

  declare_parameter("rate.external_loop_control", rclcpp::ParameterValue(1.0));
  declare_parameter("rate.internal_loop_control", rclcpp::ParameterValue(1.0));
  declare_parameter("rate.diagnostics", rclcpp::ParameterValue(1.0));

  declare_parameter("takeoff.height", rclcpp::ParameterValue(0.0));
  declare_parameter("takeoff.speed", rclcpp::ParameterValue(0.0));

  declare_parameter("land.speed", rclcpp::ParameterValue(0.2));
  declare_parameter("land.threshold_detect", rclcpp::ParameterValue(0.8));
  declare_parameter("land.increment_rampdown", rclcpp::ParameterValue(0.05));

  declare_parameter(
    "filter_params.butterworth.gyro_a", rclcpp::ParameterValue(std::vector<float_t>(3, 0.0)));
  declare_parameter(
    "filter_params.butterworth.gyro_b", rclcpp::ParameterValue(std::vector<float_t>(3, 0.0)));

  declare_parameter(
    "filter_params.butterworth.motor_a", rclcpp::ParameterValue(std::vector<float_t>(3, 0.0)));
  declare_parameter(
    "filter_params.butterworth.motor_b", rclcpp::ParameterValue(std::vector<float_t>(3, 0.0)));

  declare_parameter("agile_planner.multirotor_parameters.max_accel", rclcpp::ParameterValue(0.0));
  declare_parameter("agile_planner.multirotor_parameters.max_vel", rclcpp::ParameterValue(0.0));
  declare_parameter("agile_planner.multirotor_parameters.default_vel", rclcpp::ParameterValue(0.5));

  declare_parameter("agile_planner.ltd_opt.use_drag", rclcpp::ParameterValue(false));
  declare_parameter(
    "agile_planner.ltd_opt.thrust_decomp_acc_precision", rclcpp::ParameterValue(0.0));
  declare_parameter("agile_planner.ltd_opt.thrust_decomp_max_iter", rclcpp::ParameterValue(0));

  declare_parameter("agile_planner.first_vel_opt.alpha", rclcpp::ParameterValue(0.0));
  declare_parameter(
    "agile_planner.first_vel_opt.alpha_reduction_factor", rclcpp::ParameterValue(0.0));
  declare_parameter("agile_planner.first_vel_opt.alpha_min_threshold", rclcpp::ParameterValue(0.0));
  declare_parameter("agile_planner.first_vel_opt.max_iter", rclcpp::ParameterValue(0));

  declare_parameter("agile_planner.second_vel_opt.run", rclcpp::ParameterValue(true));
  declare_parameter("agile_planner.second_vel_opt.alpha", rclcpp::ParameterValue(0.0));
  declare_parameter(
    "agile_planner.second_vel_opt.alpha_reduction_factor", rclcpp::ParameterValue(0.0));
  declare_parameter(
    "agile_planner.second_vel_opt.alpha_min_threshold", rclcpp::ParameterValue(0.0));
  declare_parameter("agile_planner.second_vel_opt.max_iter", rclcpp::ParameterValue(0));

  declare_parameter("agile_planner.time.dt_precision", rclcpp::ParameterValue(0.0));
  declare_parameter("agile_planner.time.sampling_step", rclcpp::ParameterValue(0.0));

  declare_parameter("multirotor_parameters.mass", rclcpp::ParameterValue(0.0));
  declare_parameter(
    "multirotor_parameters.inertia", rclcpp::ParameterValue(std::vector<float_t>(3, 0.0)));
  declare_parameter("multirotor_parameters.motor_inertia", rclcpp::ParameterValue(0.0));
  declare_parameter("multirotor_parameters.c_thrust", rclcpp::ParameterValue(0.0));
  declare_parameter("multirotor_parameters.c_tau", rclcpp::ParameterValue(0.0));
  declare_parameter(
    "multirotor_parameters.omega_max", rclcpp::ParameterValue(std::vector<float_t>(3, 0.0)));
  declare_parameter(
    "multirotor_parameters.drag", rclcpp::ParameterValue(std::vector<float_t>(3, 0.0)));
  declare_parameter("multirotor_parameters.n_motors", rclcpp::ParameterValue(0));
  declare_parameter(
    "multirotor_parameters.G1", rclcpp::ParameterValue(std::vector<float_t>(32, 0.0)));
  declare_parameter(
    "multirotor_parameters.G2", rclcpp::ParameterValue(std::vector<float_t>(32, 0.0)));
  declare_parameter("multirotor_parameters.quadratic_motor_model.a", rclcpp::ParameterValue(0.0));
  declare_parameter("multirotor_parameters.quadratic_motor_model.b", rclcpp::ParameterValue(0.0));
  declare_parameter("multirotor_parameters.thrust_min", rclcpp::ParameterValue(0.0));
  declare_parameter("multirotor_parameters.thrust_max", rclcpp::ParameterValue(0.0));
  declare_parameter("multirotor_parameters.total_thrust_max", rclcpp::ParameterValue(0.0));

  declare_parameter("nmpc_controller.nmpc_mode", rclcpp::ParameterValue(""));
  declare_parameter("nmpc_controller.N", rclcpp::ParameterValue(0));
  declare_parameter("nmpc_controller.dt", rclcpp::ParameterValue(0.0));
  declare_parameter("nmpc_controller.Q", rclcpp::ParameterValue(std::vector<float_t>(6, 0.0)));
  declare_parameter("nmpc_controller.R", rclcpp::ParameterValue(0.0));

  declare_parameter("safe_area.enabled", rclcpp::ParameterValue(false));
  declare_parameter(
    "safe_area.constraints.x", rclcpp::ParameterValue(std::vector<float_t>(2, 0.0)));
  declare_parameter(
    "safe_area.constraints.y", rclcpp::ParameterValue(std::vector<float_t>(2, 0.0)));
  declare_parameter(
    "safe_area.constraints.z", rclcpp::ParameterValue(std::vector<float_t>(2, 0.0)));

  odometry_ = nav_msgs::msg::Odometry();
  diagnostics_ = laser_msgs::msg::UavControlDiagnostics();
  diagnostics_.is_fly = false;

  last_angular_speed_ = Eigen::Vector3d::Zero();
  angular_acceleration_estimated_ = Eigen::Vector3d::Zero();

  diagnostics_.metrics.rmse = NAN;
  diagnostics_.metrics.std = NAN;
}
//}

/* ~ControlManagerNode() //{ */
ControlManagerNode::~ControlManagerNode()
{}
//}

/* on_configure() //{ */
CallbackReturn ControlManagerNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  get_parameters();
  configure_pub_sub();
  configure_timers();
  configure_services();
  configure_classes();

  return CallbackReturn::SUCCESS;
}
//}

/* on_activate() //{ */
CallbackReturn ControlManagerNode::on_activate(
  [[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Activating");

  if (angular_rates_and_thrust_mode_) {
    pub_attitude_rates_and_thrust_reference_->on_activate();
  } else {
    pub_motor_speed_reference_->on_activate();
  }
  pub_diagnostics_->on_activate();

  is_active_ = true;
  return CallbackReturn::SUCCESS;
}
//}

/* on_deactivate() //{ */
CallbackReturn ControlManagerNode::on_deactivate(
  [[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  if (angular_rates_and_thrust_mode_) {
    pub_attitude_rates_and_thrust_reference_->on_deactivate();
  } else {
    pub_motor_speed_reference_->on_deactivate();
  }
  pub_diagnostics_->on_deactivate();

  is_active_ = false;
  return CallbackReturn::SUCCESS;
}
//}

/* on_cleanup() //{ */
CallbackReturn ControlManagerNode::on_cleanup(
  [[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  if (angular_rates_and_thrust_mode_) {
    pub_attitude_rates_and_thrust_reference_.reset();
  } else {
    pub_motor_speed_reference_.reset();
    sub_motor_speed_.reset();
    sub_imu_.reset();
  }

  pub_diagnostics_.reset();
  sub_odometry_.reset();
  sub_goto_.reset();
  sub_goto_relative_.reset();
  sub_api_diagnostics_.reset();
  sub_trajectory_path_.reset();

  return CallbackReturn::SUCCESS;
}
//}

/* on_shutdown() //{ */
CallbackReturn ControlManagerNode::on_shutdown(
  [[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return CallbackReturn::SUCCESS;
}
//}

/* get_parameters() //{ */
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

  get_parameter("agile_planner.multirotor_parameters.max_accel", pmm_params_.max_accel_norm);
  get_parameter("agile_planner.multirotor_parameters.max_vel", pmm_params_.max_vel_norm);
  get_parameter("agile_planner.multirotor_parameters.default_vel", pmm_params_.default_vel_norm);

  get_parameter("agile_planner.ltd_opt.use_drag", pmm_params_.use_drag);
  get_parameter(
    "agile_planner.ltd_opt.thrust_decomp_acc_precision", pmm_params_.thrust_decomp_acc_precision);
  get_parameter("agile_planner.ltd_opt.thrust_decomp_max_iter", pmm_params_.thrust_decomp_max_iter);

  get_parameter("agile_planner.first_vel_opt.alpha", pmm_params_.first_run_alpha);
  get_parameter(
    "agile_planner.first_vel_opt.alpha_reduction_factor",
    pmm_params_.first_run_alpha_reduction_factor);
  get_parameter(
    "agile_planner.first_vel_opt.alpha_min_threshold", pmm_params_.first_run_alpha_min_threshold);
  get_parameter("agile_planner.first_vel_opt.max_iter", pmm_params_.first_run_max_iter);

  get_parameter("agile_planner.second_vel_opt.run", pmm_params_.run_second_opt);
  get_parameter("agile_planner.second_vel_opt.alpha", pmm_params_.second_run_alpha);
  get_parameter(
    "agile_planner.second_vel_opt.alpha_reduction_factor",
    pmm_params_.second_run_alpha_reduction_factor);
  get_parameter(
    "agile_planner.second_vel_opt.alpha_min_threshold", pmm_params_.second_run_alpha_min_threshold);
  get_parameter("agile_planner.second_vel_opt.max_iter", pmm_params_.second_run_max_iter);

  get_parameter("agile_planner.time.dt_precision", pmm_params_.dt_precision);
  get_parameter("agile_planner.time.sampling_step", pmm_params_.sampling_step);

  get_parameter("multirotor_parameters.mass", controller_multirotor_params_.mass);
  planner_multirotor_params_.mass = controller_multirotor_params_.mass;

  get_parameter("multirotor_parameters.inertia", aux);
  controller_multirotor_params_.inertia_matrix = planner_multirotor_params_.inertia_matrix =
    Eigen::Map<const Eigen::Vector3d>(aux.as_double_array().data(), aux.as_double_array().size())
      .asDiagonal();

  get_parameter("multirotor_parameters.c_thrust", controller_multirotor_params_.c_thrust);

  get_parameter("multirotor_parameters.drag", aux);
  controller_multirotor_params_.drag =
    Eigen::Map<const Eigen::Vector3d>(aux.as_double_array().data(), aux.as_double_array().size());

  get_parameter("multirotor_parameters.omega_max", aux);
  controller_multirotor_params_.omega_max =
    Eigen::Map<const Eigen::Vector3d>(aux.as_double_array().data(), aux.as_double_array().size());

  get_parameter("multirotor_parameters.n_motors", controller_multirotor_params_.n_motors);
  get_parameter("multirotor_parameters.G1", aux);
  controller_multirotor_params_.G1 = planner_multirotor_params_.G1 =
    Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      aux.as_double_array().data(), 4, controller_multirotor_params_.n_motors);
  get_parameter("multirotor_parameters.G2", aux);
  controller_multirotor_params_.G2 =
    Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      aux.as_double_array().data(), 4, controller_multirotor_params_.n_motors);

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
  if (acados_params_.nmpc_mode == "individual_thrust") {
    angular_rates_and_thrust_mode_ = false;
  } else {
    angular_rates_and_thrust_mode_ = true;
  }

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

  nmpc_solution_ = std::pair<Eigen::Vector3d, Eigen::VectorXd>(
    Eigen::Vector3d::Zero(), Eigen::VectorXd(controller_multirotor_params_.n_motors));
  motor_speed_estimated_ = Eigen::VectorXd(controller_multirotor_params_.n_motors);

  diagnostics_.estimated_mass = controller_multirotor_params_.mass;
}
//}

/* configure_pub_sub() //{ */
void ControlManagerNode::configure_pub_sub()
{
  RCLCPP_INFO(get_logger(), "Configuring publishers and subscribers");

  sub_odometry_ = create_subscription<nav_msgs::msg::Odometry>(
    "odometry_in", 1,
    std::bind(&ControlManagerNode::odometry_callback, this, std::placeholders::_1));
  sub_goto_ = create_subscription<laser_msgs::msg::PoseWithHeading>(
    "goto_in", 1, std::bind(&ControlManagerNode::goto_callback, this, std::placeholders::_1));
  sub_goto_relative_ = create_subscription<laser_msgs::msg::PoseWithHeading>(
    "goto_relative_in", 1,
    std::bind(&ControlManagerNode::goto_relative_callback, this, std::placeholders::_1));
  sub_api_diagnostics_ = create_subscription<laser_msgs::msg::ApiPx4Diagnostics>(
    "api_diagnostics_in", 1,
    std::bind(&ControlManagerNode::api_diagnostics_callback, this, std::placeholders::_1));
  sub_trajectory_path_ = create_subscription<laser_msgs::msg::TrajectoryPath>(
    "trajectory_path_in", 1,
    std::bind(&ControlManagerNode::trajectory_path_callback, this, std::placeholders::_1));

  if (angular_rates_and_thrust_mode_) {
    pub_attitude_rates_and_thrust_reference_ =
      create_publisher<laser_msgs::msg::AttitudeRatesAndThrust>("attitude_rates_thrust_out", 10);
  } else {
    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
      "imu_in", 1, std::bind(&ControlManagerNode::imu_callback, this, std::placeholders::_1));
    sub_motor_speed_ = create_subscription<laser_msgs::msg::MotorSpeedStamped>(
      "motor_speed_estimation_in", 1,
      std::bind(&ControlManagerNode::motor_speed_callback, this, std::placeholders::_1));
    pub_motor_speed_reference_ =
      create_publisher<laser_msgs::msg::MotorSpeed>("motor_speed_reference_out", 10);
  }
  pub_diagnostics_ =
    create_publisher<laser_msgs::msg::UavControlDiagnostics>("diagnostics_out", 10);
}
//}

/* configure_timers() //{ */
void ControlManagerNode::configure_timers()
{
  RCLCPP_INFO(get_logger(), "Configuring timers");

  tmr_external_loop_control_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / rate_external_loop_control_),
    std::bind(&ControlManagerNode::external_loop_timer_callback, this), nullptr);
  if (!angular_rates_and_thrust_mode_) {
    tmr_internal_loop_control_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_internal_loop_control_),
      std::bind(&ControlManagerNode::internal_loop_timer_callback, this), nullptr);
  }

  tmr_diagnostics_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / rate_diagnostics_),
    std::bind(&ControlManagerNode::diagnostics_timer_callback, this), nullptr);
}
//}

/* configure_services() //{ */
void ControlManagerNode::configure_services()
{
  RCLCPP_INFO(get_logger(), "Configuring services");

  srv_takeoff_ = create_service<std_srvs::srv::Trigger>(
    "takeoff", std::bind(
                 &ControlManagerNode::takeoff_service_callback, this, std::placeholders::_1,
                 std::placeholders::_2));
  srv_land_ = create_service<std_srvs::srv::Trigger>(
    "land", std::bind(
              &ControlManagerNode::land_service_callback, this, std::placeholders::_1,
              std::placeholders::_2));
}
//}

/* configure_classes() //{ */
void ControlManagerNode::configure_classes()
{
  RCLCPP_INFO(get_logger(), "Configuring internal helper classes");

  agile_planner_ =
    laser_uav_planners::AgilePlanner(planner_multirotor_params_, pmm_params_, acados_params_.dt);
  nmpc_controller_ =
    laser_uav_controllers::NmpcController(controller_multirotor_params_, acados_params_);
  if (!angular_rates_and_thrust_mode_) {
    btw_gyro_x_ = laser_uav_lib::IIRFilter(gyro_a_, gyro_b_);
    btw_gyro_y_ = laser_uav_lib::IIRFilter(gyro_a_, gyro_b_);
    btw_gyro_z_ = laser_uav_lib::IIRFilter(gyro_a_, gyro_b_);

    for (auto i = 0; i < controller_multirotor_params_.n_motors; i++) {
      btw_motors_.push_back(laser_uav_lib::IIRFilter(motor_a_, motor_b_));
    }

    indi_controller_ = laser_uav_controllers::IndiController(controller_multirotor_params_);
  }
}
//}

/* check_heading_error() //{ */
double ControlManagerNode::check_heading_error()
{
  return std::abs(
    quaternion_to_heading(odometry_.pose.pose.orientation) -
    quaternion_to_heading(last_waypoint_.pose.orientation));
}
//}

/* euclidean_distance() //{ */
double ControlManagerNode::euclidean_distance(
  geometry_msgs::msg::Point p1, geometry_msgs::msg::Point p2)
{
  return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2) + std::pow(p1.z - p2.z, 2));
}
//}

/* quaternion_to_heading() //{ */
double ControlManagerNode::quaternion_to_heading(geometry_msgs::msg::Quaternion & q)
{
  Eigen::Quaterniond q_eigen(q.w, q.x, q.y, q.z);
  q_eigen.normalize();
  Eigen::Vector3d heading_vector = q_eigen * Eigen::Vector3d::UnitX();
  return std::atan2(heading_vector.y(), heading_vector.x());
}
//}

/* normalize_heading() //{ */
double ControlManagerNode::normalize_heading(double heading)
{
  auto aux = std::fmod(heading + M_PI, 2.0 * M_PI);
  if (aux < 0.0) {
    aux += 2.0 * M_PI;
  }
  return aux - M_PI;
}
//}

/**
 * @brief Checks if the future NMPC trajectory horizon remains within the configured bounding box.
 * If safety constraints are violated, an emergency hover is triggered inside the safe zone.
 */
/* check_safe_area() //{ */
void ControlManagerNode::check_safe_area()
{
  bool path_is_ok = true;
  laser_msgs::msg::PoseWithHeading emergency_hover_reference;
  emergency_hover_reference.position = odometry_.pose.pose.position;

  if (current_horizon_path_[0].pose.position.x < safe_area_.x[0]) {
    emergency_hover_reference.position.x = safe_area_.x[0] + 0.05;
    path_is_ok = false;
  } else if (current_horizon_path_[0].pose.position.x > safe_area_.x[1]) {
    emergency_hover_reference.position.x = safe_area_.x[1] - 0.05;
    path_is_ok = false;
  }

  if (current_horizon_path_[0].pose.position.y < safe_area_.y[0]) {
    emergency_hover_reference.position.y = safe_area_.y[0];
    path_is_ok = false;
  } else if (current_horizon_path_[0].pose.position.y > safe_area_.y[1]) {
    emergency_hover_reference.position.y = safe_area_.y[1] - 0.05;
    path_is_ok = false;
  }

  if (requested_takeoff_ || requested_land_) {
    if (current_horizon_path_[0].pose.position.z > safe_area_.z[1]) {
      emergency_hover_reference.position.z = safe_area_.z[1] - 0.05;
      path_is_ok = false;
    }
  } else {
    if (current_horizon_path_[0].pose.position.z < safe_area_.z[0]) {
      emergency_hover_reference.position.z = safe_area_.z[0] + 0.05;
      path_is_ok = false;
    } else if (current_horizon_path_[0].pose.position.z > safe_area_.z[1]) {
      emergency_hover_reference.position.z = safe_area_.z[1] - 0.05;
      path_is_ok = false;
    }
  }

  if (!path_is_ok) {
    RCLCPP_WARN(
      this->get_logger(),
      "Trajectory will not be executed because the next trajectory points are outside the safe "
      "area. Entering emergency hover. Please submit a valid point within the safe area.");
    RCLCPP_WARN(
      this->get_logger(),
      "\nSafe Area Constraints:\n"
      " (x , y)\n"
      "(%.1f, %.1f)  --------------------- (%.1f, -%.1f)\n"
      "     |                                 |\n"
      "     |                                 |\n"
      "     |                                 |   %.1f <= z <= %.1f\n"
      "     |                                 |\n"
      "     |                                 |\n"
      "(-%.1f, %.1f) --------------------- (-%.1f, -%.1f)",
      safe_area_.x[1], safe_area_.y[1], safe_area_.x[1], safe_area_.y[0], safe_area_.z[0],
      safe_area_.z[1], safe_area_.x[0], safe_area_.y[1], safe_area_.x[0], safe_area_.y[0]);

    emergency_hover_reference.heading = quaternion_to_heading(odometry_.pose.pose.orientation);
    agile_planner_.generateTrajectory(last_waypoint_, emergency_hover_reference, 0.0, false);
    emergency_hover_ = true;
  }
}
//}

/**
 * @brief Estimates the UAV mass during a stable hover after takeoff.
 * Updates both NMPC and Agile Planner models with the new mass.
 */
/* estimate_mass() //{ */
bool ControlManagerNode::estimate_mass()
{
  if (start_mass_estimation_ && abs(odometry_.twist.twist.linear.z) < 0.02) {
    mass_estimation_time_start_ = this->get_clock()->now();
    start_mass_estimation_ = false;
  }

  if (start_mass_estimation_) {
    return false;
  }

  if (rclcpp::Duration(this->get_clock()->now() - mass_estimation_time_start_).seconds() < 3.0) {
    return false;
  }

  estimated_mass_ =
    (0.98 * estimated_mass_for_detect_landing_) + ((1 - 0.98) * controller_multirotor_params_.mass);
  diagnostics_.estimated_mass = estimated_mass_;
  nmpc_controller_.setMass(estimated_mass_);
  agile_planner_.setMass(estimated_mass_);
  RCLCPP_INFO(this->get_logger(), "Estimated Calibrated Mass: %.2f", estimated_mass_);

  return true;
}
//}

/* odometry_callback() //{ */
void ControlManagerNode::odometry_callback(const nav_msgs::msg::Odometry & msg)
{
  if (!is_active_) {
    return;
  }

  odometry_ = msg;
  diagnostics_.current_norm_speed = sqrt(
    pow(odometry_.twist.twist.linear.x, 2) + pow(odometry_.twist.twist.linear.y, 2) +
    pow(odometry_.twist.twist.linear.z, 2));
  received_first_odometry_msg_ = true;
}
//}

/**
 * @brief Filters raw IMU angular velocities and numerically differentiates them
 * to estimate angular acceleration required for INDI control.
 */
/* imu_callback() //{ */
void ControlManagerNode::imu_callback(const sensor_msgs::msg::Imu & msg)
{
  if (!is_active_) {
    return;
  }

  Eigen::Vector3d current;
  current << btw_gyro_x_.iterate(msg.angular_velocity.x),
    btw_gyro_y_.iterate(msg.angular_velocity.y), btw_gyro_z_.iterate(msg.angular_velocity.z);

  angular_acceleration_estimated_(0) = (current(0) - last_angular_speed_(0)) / 0.004;
  angular_acceleration_estimated_(1) = (current(1) - last_angular_speed_(1)) / 0.004;
  angular_acceleration_estimated_(2) = (current(2) - last_angular_speed_(2)) / 0.004;

  last_angular_speed_ = current;
}
//}

/* motor_speed_callback() //{ */
void ControlManagerNode::motor_speed_callback(const laser_msgs::msg::MotorSpeedStamped & msg)
{
  if (!is_active_) {
    return;
  }

  for (auto i = 0; i < (int)msg.data.data.size(); i++) {
    motor_speed_estimated_(i) = btw_motors_[i].iterate(msg.data.data[i]);
  }
}
//}

/* api_diagnostics_callback() //{ */
void ControlManagerNode::api_diagnostics_callback(const laser_msgs::msg::ApiPx4Diagnostics & msg)
{
  if (!is_active_) {
    return;
  }

  if (msg.armed && requested_takeoff_ && msg.offboard_mode) {
    lock_control_inputs_ = false;
  }
}
//}

/* trajectory_path_callback() //{ */
void ControlManagerNode::trajectory_path_callback(const laser_msgs::msg::TrajectoryPath & msg)
{
  if (!is_active_) {
    return;
  }

  if (!requested_takeoff_ && !requested_land_ && takeoff_done_) {
    int count_not_deviation = 0;
    bool valid_deviation = false;
    for (auto i = 0; i < msg.waypoints.size(); i++) {
      if (valid_deviation) {
        break;
      }

      if (euclidean_distance(odometry_.pose.pose.position, msg.waypoints[i].position) < 0.1) {
        count_not_deviation++;
      } else {
        valid_deviation = true;
      }
    }

    if (count_not_deviation == msg.waypoints.size()) {
      RCLCPP_WARN(
        this->get_logger(),
        "Trajectory will not executed, because dont have any point with deviation more bigger than "
        "0.1m.");
    } else if (count_not_deviation > 0) {
      RCLCPP_WARN(
        this->get_logger(),
        "Points with deviation smaller tan 0.1 filtered and removed from the trajectory called. "
        "Quantity Points: %d",
        count_not_deviation);
    }

    stop_on_waypoints_ = msg.stop_on_waypoints;
    trajectory_speed_ = msg.speed;
    desired_path_ = msg.waypoints;
    emergency_hover_ = false;

    if (desired_path_.empty()) {
      RCLCPP_WARN(
        this->get_logger(),
        "Trajectory will not executed, because it does not contain any waypoint.");
      return;
    }

    if (stop_on_waypoints_) {
      agile_planner_.generateTrajectory(
        last_waypoint_, desired_path_.front(), trajectory_speed_, true);
    } else {
      agile_planner_.generateTrajectory(last_waypoint_, desired_path_, trajectory_speed_);
    }

    RCLCPP_INFO(this->get_logger(), "Trajectory Received!");
    diagnostics_.have_goal = true;
  } else {
    RCLCPP_WARN(this->get_logger(), "Trajectory will not executed, because the uav is not flying.");
  }
}
//}

/* goto_callback() //{ */
void ControlManagerNode::goto_callback(const laser_msgs::msg::PoseWithHeading & msg)
{
  if (!is_active_) {
    return;
  }

  if (!requested_takeoff_ && !requested_land_ && takeoff_done_) {
    if (euclidean_distance(odometry_.pose.pose.position, msg.position) < 0.1) {
      RCLCPP_WARN(
        this->get_logger(),
        "GOTO's Point will not executed, because the uav alerady is at this point.");
    } else {
      agile_planner_.generateTrajectory(last_waypoint_, msg, 0.0, false);
      stop_on_waypoints_ = false;
      emergency_hover_ = false;
      RCLCPP_INFO(this->get_logger(), "GOTO's Point Received!");
      diagnostics_.have_goal = true;
    }
  } else {
    RCLCPP_WARN(
      this->get_logger(), "GOTO's Point will not executed, because the uav is not flying.");
  }
}
//}

/* goto_relative_callback() //{ */
void ControlManagerNode::goto_relative_callback(const laser_msgs::msg::PoseWithHeading & msg)
{
  if (!is_active_) {
    return;
  }

  if (!requested_takeoff_ && !requested_land_ && takeoff_done_) {
    Eigen::Quaterniond q(
      last_waypoint_.pose.orientation.w, last_waypoint_.pose.orientation.x,
      last_waypoint_.pose.orientation.y, last_waypoint_.pose.orientation.z);
    q.normalize();
    Eigen::Vector3d aux;
    aux << msg.position.x, msg.position.y, msg.position.z;

    laser_msgs::msg::PoseWithHeading world_point;
    aux = q * aux;
    std::cout << aux << std::endl;
    world_point.position.x = last_waypoint_.pose.position.x + aux(0);
    world_point.position.y = last_waypoint_.pose.position.y + aux(1);
    world_point.position.z = last_waypoint_.pose.position.z + aux(2);
    world_point.heading = quaternion_to_heading(last_waypoint_.pose.orientation) + msg.heading;

    agile_planner_.generateTrajectory(last_waypoint_, world_point, 0.0, false);
    stop_on_waypoints_ = false;
    emergency_hover_ = false;
    RCLCPP_INFO(this->get_logger(), "GOTO's Relative Point Received!");
    diagnostics_.have_goal = true;
  } else {
    RCLCPP_WARN(
      this->get_logger(),
      "GOTO's Relative Point will not executed, because the uav is not flying.");
  }
}
//}

/* takeoff_service_callback() //{ */
void ControlManagerNode::takeoff_service_callback(
  [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  [[maybe_unused]] std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!is_active_) {
    return;
  }

  if (!received_first_odometry_msg_) {
    response->success = false;
    response->message = "takeoff requested failed, odometry msg not received";
    RCLCPP_ERROR(
      this->get_logger(), "Takeoff requested failed, because the odometry msg not received!");
    return;
  }

  if (takeoff_done_) {
    response->success = false;
    response->message = "takeoff requested failed, uav already flying";
  } else {
    response->success = true;
    response->message = "takeoff requested success";

    requested_takeoff_ = true;

    laser_msgs::msg::PoseWithHeading takeoff_waypoint;
    takeoff_waypoint.position = odometry_.pose.pose.position;
    takeoff_waypoint.position.z = takeoff_height_;
    takeoff_waypoint.heading = quaternion_to_heading(odometry_.pose.pose.orientation);

    agile_planner_.generateTrajectory(last_waypoint_, takeoff_waypoint, takeoff_speed_, true);

    land_done_ = false;
    diagnostics_.have_goal = true;
    start_mass_estimation_ = true;
  }
}
//}

/* land_service_callback() //{ */
void ControlManagerNode::land_service_callback(
  [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  [[maybe_unused]] std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!is_active_) {
    return;
  }

  if (land_done_) {
    response->success = false;
    response->message = "takeoff requested failed, uav already in ground";
  } else {
    response->success = true;
    response->message = "land requested success";

    requested_land_ = true;

    laser_msgs::msg::PoseWithHeading land_waypoint;
    land_waypoint.position = odometry_.pose.pose.position;
    land_waypoint.position.z = -1.0;
    land_waypoint.heading = quaternion_to_heading(odometry_.pose.pose.orientation);

    agile_planner_.generateTrajectory(last_waypoint_, land_waypoint, land_speed_, true);

    takeoff_done_ = false;
    diagnostics_.have_goal = true;
  }
}
//}

/**
 * @brief High-level NMPC external control loop.
 * Solves the Optimal Control Problem (OCP) for the current horizon, manages touchdown detection,
 * executes landing thrust ramp-down, and publishes attitude/thrust setpoints.
 */
/* external_loop_timer_callback() //{ */
void ControlManagerNode::external_loop_timer_callback()
{
  if (!is_active_) {
    return;
  }

  if (lock_control_inputs_) {
    return;
  }

  auto start_iteration = std::chrono::high_resolution_clock::now();

  // Handle landing thrust ramp-down after touchdown is detected
  if (land_rampdown_) {
    RCLCPP_INFO(this->get_logger(), "Land Ramp Down: %.2f", land_start_rampdown_);

    if (angular_rates_and_thrust_mode_) {
      laser_msgs::msg::AttitudeRatesAndThrust msg;
      msg.total_thrust_normalized = land_start_rampdown_;
      msg.roll_rate = 0;
      msg.pitch_rate = 0;
      msg.yaw_rate = 0;

      pub_attitude_rates_and_thrust_reference_->publish(msg);
    } else {
      laser_msgs::msg::MotorSpeed msg;
      for (auto i = 0; i < nmpc_solution_.second.size(); i++) {
        msg.data.push_back(land_start_rampdown_);
      }
      diagnostics_.last_control_input.data = nmpc_controller_.getLastIndividualThrust();
      pub_motor_speed_reference_->publish(msg);
    }

    diagnostics_.last_control_input.unit_of_measurement = "N";
    diagnostics_.last_control_input.data = std::vector(
      controller_multirotor_params_.n_motors,
      laser_uav_controllers::throtleToThrust(
        controller_multirotor_params_.motor_curve_a, controller_multirotor_params_.motor_curve_b,
        land_start_rampdown_));

    if (land_start_rampdown_ == 0.0) {
      land_rampdown_ = false;
      lock_control_inputs_ = true;
    }

    diagnostics_.current_control_rampdown = land_start_rampdown_;
    land_start_rampdown_ -= land_increment_rampdown_;

    if (land_start_rampdown_ < 0.0) {
      land_start_rampdown_ = 0.0;
    }

    diagnostics_.control_iteration_duration_ms =
      std::chrono::duration<double, std::milli>(
        std::chrono::high_resolution_clock::now() - start_iteration)
        .count();
    return;
  } else {
    diagnostics_.current_control_rampdown = NAN;
  }

  // Generate trajectory reference horizon and solve the NMPC optimization problem
  if (
    stop_on_waypoints_ && desired_path_.size() > 0 &&
    (euclidean_distance(odometry_.pose.pose.position, desired_path_[0].position) < 0.1 &&
     std::abs(
       quaternion_to_heading(last_waypoint_.pose.orientation) -
       normalize_heading(desired_path_[0].heading)) < 0.1)) {
    if (
      euclidean_distance(odometry_.pose.pose.position, desired_path_[0].position) < 0.15 &&
      check_heading_error() < 0.2) {
      lock_waypoint_++;
    }

    if (lock_waypoint_ > 300) {
      desired_path_.erase(desired_path_.begin());
      lock_waypoint_ = 0;

      if (!desired_path_.empty()) {
        agile_planner_.generateTrajectory(
          last_waypoint_, desired_path_.front(), trajectory_speed_, true);
      }
    }

    last_waypoint_.use_linear_velocity = false;
    last_waypoint_.use_angular_velocity = false;
    last_waypoint_.use_individual_thrust = false;

    nmpc_solution_ = nmpc_controller_.getCorrection(last_waypoint_, odometry_);
    diagnostics_.ocp_elapsed_time_ms = nmpc_controller_.getOcpElapsedTime();
    diagnostics_.header.stamp = get_clock()->now();
  } else {
    current_horizon_path_ =
      agile_planner_.getTrajectory(acados_params_.N + 1, this->get_clock()->now().seconds());
    if (safe_area_.enabled && diagnostics_.is_fly && !emergency_hover_) {
      check_safe_area();
    }
    last_waypoint_ = current_horizon_path_[0];
    lock_waypoint_ = 0;

    diagnostics_.reference_horizon = current_horizon_path_;
    nmpc_solution_ = nmpc_controller_.getCorrection(current_horizon_path_, odometry_);
    diagnostics_.ocp_elapsed_time_ms = nmpc_controller_.getOcpElapsedTime();
    diagnostics_.header.stamp = get_clock()->now();
  }
  have_nmpc_solution_ = true;

  if (diagnostics_.have_goal) {
    estimated_rmse_.pushEstimated(odometry_.pose.pose.position);
    estimated_rmse_.pushReference(last_waypoint_.pose.position);

    diagnostics_.metrics.rmse = NAN;
    diagnostics_.metrics.std = NAN;
  }

  // Publish attitude rate and thrust setpoints if configured in this control mode
  if (angular_rates_and_thrust_mode_) {
    estimated_mass_for_detect_landing_ = (1 / GRAVITY) * nmpc_solution_.second.sum();

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

  // Handle takeoff procedure and trigger mass estimation
  if (requested_takeoff_) {
    if (agile_planner_.isHover()) {
      if (estimate_mass()) {
        requested_takeoff_ = false;
        takeoff_done_ = true;
        diagnostics_.is_fly = true;
        RCLCPP_INFO(this->get_logger(), "Takeoff Done!");
      }
    }
  }

  // Handle landing procedure and touchdown detection based on thrust drop
  if (requested_land_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 2500,
      "Current estimated mass for detect landing: %.3f", estimated_mass_for_detect_landing_);
    if (estimated_mass_for_detect_landing_ <= estimated_mass_ * land_threshold_detect_) {
      requested_land_ = false;
      land_done_ = true;
      diagnostics_.have_goal = false;
      diagnostics_.is_fly = false;
      land_rampdown_ = true;
      land_start_rampdown_ = laser_uav_controllers::thrustToThrotle(
        controller_multirotor_params_.motor_curve_a, controller_multirotor_params_.motor_curve_b,
        (estimated_mass_ * GRAVITY) / controller_multirotor_params_.n_motors);
      RCLCPP_INFO(
        this->get_logger(), "Landing Done!, Detected land with estimated mass: %.3f",
        estimated_mass_for_detect_landing_);
      RCLCPP_INFO(this->get_logger(), "Start Land Ramp Down!");
    } else if (agile_planner_.isHover()) {
      laser_msgs::msg::PoseWithHeading land_waypoint;
      land_waypoint.position = odometry_.pose.pose.position;
      land_waypoint.heading = quaternion_to_heading(odometry_.pose.pose.orientation);
      land_waypoint.position.z += -1.0;

      agile_planner_.generateTrajectory(last_waypoint_, land_waypoint, land_speed_, true);
    }
  }

  if (diagnostics_.have_goal) {
    if (agile_planner_.isHover()) {
      if (
        euclidean_distance(odometry_.pose.pose.position, last_waypoint_.pose.position) < 0.15 ||
        check_heading_error() > 0.15) {
        if (stop_on_waypoints_) {
          if (desired_path_.empty()) {
            diagnostics_.have_goal = false;
          }
        } else {
          diagnostics_.have_goal = false;
        }
      }
    }

    if (!diagnostics_.have_goal) {
      calculate_rmse_ = true;
    }
  }

  diagnostics_.control_iteration_duration_ms =
    std::chrono::duration<double, std::milli>(
      std::chrono::high_resolution_clock::now() - start_iteration)
      .count();
}
//}

/**
 * @brief Fast INDI internal loop.
 * Computes incremental motor speed corrections based on angular acceleration errors and publishes
 * individual rotor speed commands when direct motor speed control is enabled.
 */
/* internal_loop_timer_callback() //{ */
void ControlManagerNode::internal_loop_timer_callback()
{
  if (!is_active_) {
    return;
  }

  if (lock_control_inputs_) {
    return;
  }

  if (land_rampdown_) {
    return;
  }

  if (have_nmpc_solution_) {
    Eigen::VectorXd indi_thrust = indi_controller_.getCorrection(
      angular_acceleration_estimated_, motor_speed_estimated_, nmpc_solution_.second,
      nmpc_solution_.first);

    estimated_mass_for_detect_landing_ = (1 / GRAVITY) * indi_thrust.sum();

    diagnostics_.last_control_input.unit_of_measurement = "N";
    diagnostics_.last_control_input.data =
      std::vector<double>(indi_thrust.data(), indi_thrust.data() + indi_thrust.size());

    laser_msgs::msg::MotorSpeed msg;
    for (auto i = 0; i < indi_thrust.size(); i++) {
      msg.data.push_back(laser_uav_controllers::thrustToThrotle(
        controller_multirotor_params_.motor_curve_a, controller_multirotor_params_.motor_curve_b,
        indi_thrust(i), controller_multirotor_params_.thrust_max,
        controller_multirotor_params_.thrust_min));
    }

    pub_motor_speed_reference_->publish(msg);
  }
}
//}

/* diagnostics_timer_callback() //{ */
void ControlManagerNode::diagnostics_timer_callback()
{
  if (!is_active_) {
    return;
  }

  if (calculate_rmse_) {
    auto result = estimated_rmse_.calculate();
    diagnostics_.metrics.rmse = result.first;
    diagnostics_.metrics.std = result.second;
    estimated_rmse_.reset();
    calculate_rmse_ = false;
  }

  diagnostics_.header.frame_id = "";
  pub_diagnostics_->publish(diagnostics_);
}
//}
}  // namespace laser_uav_managers

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(laser_uav_managers::ControlManagerNode)
