#include "laser_uav_managers/control_manager_node.hpp"

namespace laser_uav_managers
{
/* ControlManagerNode() //{ */
ControlManagerNode::ControlManagerNode(const rclcpp::NodeOptions &options) : rclcpp_lifecycle::LifecycleNode("control_manager", "", options) {
  RCLCPP_INFO(get_logger(), "Creating");

  declare_parameter("rate.external_loop_control", rclcpp::ParameterValue(1.0));
  declare_parameter("rate.internal_loop_control", rclcpp::ParameterValue(1.0));
  declare_parameter("rate.diagnostics", rclcpp::ParameterValue(1.0));

  declare_parameter("takeoff.height", rclcpp::ParameterValue(0.0));
  declare_parameter("takeoff.speed", rclcpp::ParameterValue(0.0));

  declare_parameter("land.speed", rclcpp::ParameterValue(0.2));
  declare_parameter("land.threshold_detect", rclcpp::ParameterValue(0.8));
  declare_parameter("land.increment_rampdown", rclcpp::ParameterValue(0.05));

  declare_parameter("filter_params.butterworth.gyro_a", rclcpp::ParameterValue(std::vector<float_t>(3, 0.0)));
  declare_parameter("filter_params.butterworth.gyro_b", rclcpp::ParameterValue(std::vector<float_t>(3, 0.0)));

  declare_parameter("filter_params.butterworth.motor_a", rclcpp::ParameterValue(std::vector<float_t>(3, 0.0)));
  declare_parameter("filter_params.butterworth.motor_b", rclcpp::ParameterValue(std::vector<float_t>(3, 0.0)));

  declare_parameter("agile_planner.multirotor_parameters.max_accel", rclcpp::ParameterValue(0.0));
  declare_parameter("agile_planner.multirotor_parameters.max_vel", rclcpp::ParameterValue(0.0));
  declare_parameter("agile_planner.multirotor_parameters.default_vel", rclcpp::ParameterValue(0.5));

  declare_parameter("agile_planner.ltd_opt.use_drag", rclcpp::ParameterValue(false));
  declare_parameter("agile_planner.ltd_opt.thrust_decomp_acc_precision", rclcpp::ParameterValue(0.0));
  declare_parameter("agile_planner.ltd_opt.thrust_decomp_max_iter", rclcpp::ParameterValue(0));

  declare_parameter("agile_planner.first_vel_opt.alpha", rclcpp::ParameterValue(0.0));
  declare_parameter("agile_planner.first_vel_opt.alpha_reduction_factor", rclcpp::ParameterValue(0.0));
  declare_parameter("agile_planner.first_vel_opt.alpha_min_threshold", rclcpp::ParameterValue(0.0));
  declare_parameter("agile_planner.first_vel_opt.max_iter", rclcpp::ParameterValue(0));

  declare_parameter("agile_planner.second_vel_opt.run", rclcpp::ParameterValue(true));
  declare_parameter("agile_planner.second_vel_opt.alpha", rclcpp::ParameterValue(0.0));
  declare_parameter("agile_planner.second_vel_opt.alpha_reduction_factor", rclcpp::ParameterValue(0.0));
  declare_parameter("agile_planner.second_vel_opt.alpha_min_threshold", rclcpp::ParameterValue(0.0));
  declare_parameter("agile_planner.second_vel_opt.max_iter", rclcpp::ParameterValue(0));

  declare_parameter("agile_planner.time.dt_precision", rclcpp::ParameterValue(0.0));
  declare_parameter("agile_planner.time.sampling_step", rclcpp::ParameterValue(0.0));

  declare_parameter("multirotor_parameters.mass", rclcpp::ParameterValue(0.0));
  declare_parameter("multirotor_parameters.inertia", rclcpp::ParameterValue(std::vector<float_t>(3, 0.0)));
  declare_parameter("multirotor_parameters.motor_inertia", rclcpp::ParameterValue(0.0));
  declare_parameter("multirotor_parameters.c_thrust", rclcpp::ParameterValue(0.0));
  declare_parameter("multirotor_parameters.c_tau", rclcpp::ParameterValue(0.0));
  declare_parameter("multirotor_parameters.drag", rclcpp::ParameterValue(std::vector<float_t>(3, 0.0)));
  declare_parameter("multirotor_parameters.n_motors", rclcpp::ParameterValue(0));
  declare_parameter("multirotor_parameters.G1", rclcpp::ParameterValue(std::vector<float_t>(32, 0.0)));
  declare_parameter("multirotor_parameters.G2", rclcpp::ParameterValue(std::vector<float_t>(32, 0.0)));
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
  declare_parameter("safe_area.constraints.x", rclcpp::ParameterValue(std::vector<float_t>(2, 0.0)));
  declare_parameter("safe_area.constraints.y", rclcpp::ParameterValue(std::vector<float_t>(2, 0.0)));
  declare_parameter("safe_area.constraints.z", rclcpp::ParameterValue(std::vector<float_t>(2, 0.0)));

  odometry_           = nav_msgs::msg::Odometry();
  diagnostics_        = laser_msgs::msg::UavControlDiagnostics();
  diagnostics_.is_fly = false;

  last_angular_speed_             = Eigen::Vector3d::Zero();
  angular_acceleration_estimated_ = Eigen::Vector3d::Zero();
}
//}

/* ~ControlManagerNode() //{ */
ControlManagerNode::~ControlManagerNode() {
}
//}

/* on_configure() //{ */
CallbackReturn ControlManagerNode::on_configure(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Configuring");

  getParameters();
  configPubSub();
  configTimers();
  configServices();
  configClasses();

  return CallbackReturn::SUCCESS;
}
//}

/* on_activate() //{ */
CallbackReturn ControlManagerNode::on_activate([[maybe_unused]] const rclcpp_lifecycle::State &state) {
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
CallbackReturn ControlManagerNode::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State &state) {
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

/* on_clenaup() //{ */
CallbackReturn ControlManagerNode::on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State &state) {
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
CallbackReturn ControlManagerNode::on_shutdown([[maybe_unused]] const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Shutting down");

  return CallbackReturn::SUCCESS;
}
//}

/* getParameters() //{ */
void ControlManagerNode::getParameters() {
  rclcpp::Parameter aux;
  Eigen::VectorXd   aux_eigen;

  get_parameter("rate.external_loop_control", _rate_external_loop_control_);
  get_parameter("rate.internal_loop_control", _rate_internal_loop_control_);
  get_parameter("rate.diagnostics", _rate_diagnostics_);

  get_parameter("takeoff.height", _takeoff_height_);
  get_parameter("takeoff.speed", _takeoff_speed_);

  get_parameter("land.speed", _land_speed_);
  get_parameter("land.threshold_detect", _land_threshold_detect_);
  get_parameter("land.increment_rampdown", _land_increment_rampdown_);

  get_parameter("filter_params.butterworth.gyro_a", aux);
  _gyro_a_ = aux.as_double_array();
  get_parameter("filter_params.butterworth.gyro_b", aux);
  _gyro_b_ = aux.as_double_array();

  get_parameter("filter_params.butterworth.motor_a", aux);
  _motor_a_ = aux.as_double_array();
  get_parameter("filter_params.butterworth.motor_b", aux);
  _motor_b_ = aux.as_double_array();

  get_parameter("agile_planner.multirotor_parameters.max_accel", _pmm_params_.max_accel_norm);
  get_parameter("agile_planner.multirotor_parameters.max_vel", _pmm_params_.max_vel_norm);
  get_parameter("agile_planner.multirotor_parameters.default_vel", _pmm_params_.default_vel_norm);

  get_parameter("agile_planner.ltd_opt.use_drag", _pmm_params_.use_drag);
  get_parameter("agile_planner.ltd_opt.thrust_decomp_acc_precision", _pmm_params_.thrust_decomp_acc_precision);
  get_parameter("agile_planner.ltd_opt.thrust_decomp_max_iter", _pmm_params_.thrust_decomp_max_iter);

  get_parameter("agile_planner.first_vel_opt.alpha", _pmm_params_.first_run_alpha);
  get_parameter("agile_planner.first_vel_opt.alpha_reduction_factor", _pmm_params_.first_run_alpha_reduction_factor);
  get_parameter("agile_planner.first_vel_opt.alpha_min_threshold", _pmm_params_.first_run_alpha_min_threshold);
  get_parameter("agile_planner.first_vel_opt.max_iter", _pmm_params_.first_run_max_iter);

  get_parameter("agile_planner.second_vel_opt.run", _pmm_params_.run_second_opt);
  get_parameter("agile_planner.second_vel_opt.alpha", _pmm_params_.second_run_alpha);
  get_parameter("agile_planner.second_vel_opt.alpha_reduction_factor", _pmm_params_.second_run_alpha_reduction_factor);
  get_parameter("agile_planner.second_vel_opt.alpha_min_threshold", _pmm_params_.second_run_alpha_min_threshold);
  get_parameter("agile_planner.second_vel_opt.max_iter", _pmm_params_.second_run_max_iter);

  get_parameter("agile_planner.time.dt_precision", _pmm_params_.dt_precision);
  get_parameter("agile_planner.time.sampling_step", _pmm_params_.sampling_step);

  get_parameter("multirotor_parameters.mass", _controller_multirotor_params_.mass);
  _planner_multirotor_params_.mass = _controller_multirotor_params_.mass;

  get_parameter("multirotor_parameters.inertia", aux);
  _controller_multirotor_params_.inertia_matrix = _planner_multirotor_params_.inertia_matrix =
      Eigen::Map<const Eigen::Vector3d>(aux.as_double_array().data(), aux.as_double_array().size()).asDiagonal();

  get_parameter("multirotor_parameters.c_thrust", _controller_multirotor_params_.c_thrust);

  get_parameter("multirotor_parameters.drag", aux);
  _controller_multirotor_params_.drag = Eigen::Map<const Eigen::Vector3d>(aux.as_double_array().data(), aux.as_double_array().size());

  get_parameter("multirotor_parameters.n_motors", _controller_multirotor_params_.n_motors);
  get_parameter("multirotor_parameters.G1", aux);
  _controller_multirotor_params_.G1 = _planner_multirotor_params_.G1 = Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      aux.as_double_array().data(), 4, _controller_multirotor_params_.n_motors);
  get_parameter("multirotor_parameters.G2", aux);
  _controller_multirotor_params_.G2 = Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      aux.as_double_array().data(), 4, _controller_multirotor_params_.n_motors);

  get_parameter("multirotor_parameters.motor_inertia", _controller_multirotor_params_.motor_inertia);

  get_parameter("multirotor_parameters.quadratic_motor_model.a", _controller_multirotor_params_.motor_curve_a);
  get_parameter("multirotor_parameters.quadratic_motor_model.b", _controller_multirotor_params_.motor_curve_b);
  get_parameter("multirotor_parameters.thrust_min", _controller_multirotor_params_.thrust_min);
  get_parameter("multirotor_parameters.thrust_max", _controller_multirotor_params_.thrust_max);
  get_parameter("multirotor_parameters.total_thrust_max", _controller_multirotor_params_.total_thrust_max);

  get_parameter("nmpc_controller.nmpc_mode", _acados_params_.nmpc_mode);
  if (_acados_params_.nmpc_mode == "individual_thrust") {
    angular_rates_and_thrust_mode_ = false;
  } else {
    angular_rates_and_thrust_mode_ = true;
  }

  get_parameter("nmpc_controller.N", _acados_params_.N);
  get_parameter("nmpc_controller.dt", _acados_params_.dt);

  get_parameter("nmpc_controller.Q", aux);
  _acados_params_.Q = aux.as_double_array();

  get_parameter("nmpc_controller.R", _acados_params_.R);

  get_parameter("safe_area.enabled", _safe_area_.enabled);
  get_parameter("safe_area.constraints.x", aux);
  _safe_area_.x = aux.as_double_array();
  get_parameter("safe_area.constraints.y", aux);
  _safe_area_.y = aux.as_double_array();
  get_parameter("safe_area.constraints.z", aux);
  _safe_area_.z = aux.as_double_array();

  nmpc_control_input_    = Eigen::VectorXd(_controller_multirotor_params_.n_motors);
  motor_speed_estimated_ = Eigen::VectorXd(_controller_multirotor_params_.n_motors);

  diagnostics_.estimated_mass = _controller_multirotor_params_.mass;
}
//}

/* configPubSub() //{ */
void ControlManagerNode::configPubSub() {
  RCLCPP_INFO(get_logger(), "initPubSub");

  sub_odometry_ = create_subscription<nav_msgs::msg::Odometry>("odometry_in", 1, std::bind(&ControlManagerNode::subOdometry, this, std::placeholders::_1));
  sub_goto_     = create_subscription<laser_msgs::msg::PoseWithHeading>("goto_in", 1, std::bind(&ControlManagerNode::subGoto, this, std::placeholders::_1));
  sub_goto_relative_   = create_subscription<laser_msgs::msg::PoseWithHeading>("goto_relative_in", 1,
                                                                             std::bind(&ControlManagerNode::subGotoRelative, this, std::placeholders::_1));
  sub_api_diagnostics_ = create_subscription<laser_msgs::msg::ApiPx4Diagnostics>(
      "api_diagnostics_in", 1, std::bind(&ControlManagerNode::subApiDiagnostics, this, std::placeholders::_1));
  sub_trajectory_path_ = create_subscription<laser_msgs::msg::TrajectoryPath>("trajectory_path_in", 1,
                                                                              std::bind(&ControlManagerNode::subTrajectoryPath, this, std::placeholders::_1));

  if (angular_rates_and_thrust_mode_) {
    pub_attitude_rates_and_thrust_reference_ = create_publisher<laser_msgs::msg::AttitudeRatesAndThrust>("attitude_rates_thrust_out", 10);
  } else {
    sub_imu_         = create_subscription<sensor_msgs::msg::Imu>("imu_in", 1, std::bind(&ControlManagerNode::subImu, this, std::placeholders::_1));
    sub_motor_speed_ = create_subscription<laser_msgs::msg::MotorSpeedStamped>(
        "motor_speed_estimation_in", 1, std::bind(&ControlManagerNode::subMotorSpeedStamped, this, std::placeholders::_1));
    pub_motor_speed_reference_ = create_publisher<laser_msgs::msg::MotorSpeed>("motor_speed_reference_out", 10);
  }
  pub_diagnostics_ = create_publisher<laser_msgs::msg::UavControlDiagnostics>("diagnostics_out", 10);
}
//}

/* configTimers() //{ */
void ControlManagerNode::configTimers() {
  RCLCPP_INFO(get_logger(), "initTimers");

  tmr_external_loop_control_ = create_wall_timer(std::chrono::duration<double>(1.0 / _rate_external_loop_control_),
                                                 std::bind(&ControlManagerNode::tmrExternalLoopControl, this), nullptr);
  if (!angular_rates_and_thrust_mode_) {
    tmr_internal_loop_control_ = create_wall_timer(std::chrono::duration<double>(1.0 / _rate_internal_loop_control_),
                                                   std::bind(&ControlManagerNode::tmrInternalLoopControl, this), nullptr);
  }

  tmr_diagnostics_ = create_wall_timer(std::chrono::duration<double>(1.0 / _rate_diagnostics_), std::bind(&ControlManagerNode::tmrDiagnostics, this), nullptr);
}
//}

/* configServices() //{ */
void ControlManagerNode::configServices() {
  RCLCPP_INFO(get_logger(), "initServices");

  srv_takeoff_ =
      create_service<std_srvs::srv::Trigger>("takeoff", std::bind(&ControlManagerNode::srvTakeoff, this, std::placeholders::_1, std::placeholders::_2));
  srv_land_ = create_service<std_srvs::srv::Trigger>("land", std::bind(&ControlManagerNode::srvLand, this, std::placeholders::_1, std::placeholders::_2));
}
//}

/* configClasses() //{ */
void ControlManagerNode::configClasses() {
  RCLCPP_INFO(get_logger(), "initClasses");

  agile_planner_   = laser_uav_planners::AgilePlanner(_planner_multirotor_params_, _pmm_params_, _acados_params_.dt);
  nmpc_controller_ = laser_uav_controllers::NmpcController(_controller_multirotor_params_, _acados_params_);
  if (!angular_rates_and_thrust_mode_) {
    btw_gyro_x_ = laser_uav_lib::IIRFilter(_gyro_a_, _gyro_b_);
    btw_gyro_y_ = laser_uav_lib::IIRFilter(_gyro_a_, _gyro_b_);
    btw_gyro_z_ = laser_uav_lib::IIRFilter(_gyro_a_, _gyro_b_);

    for (auto i = 0; i < _controller_multirotor_params_.n_motors; i++) {
      btw_motors_.push_back(laser_uav_lib::IIRFilter(_motor_a_, _motor_b_));
    }

    indi_controller_ = laser_uav_controllers::IndiController(_controller_multirotor_params_);
  }
}
//}

/* checkHeadingError() //{ */
double ControlManagerNode::checkHeadingError() {
  return std::abs(quaternionToHeading(odometry_.pose.pose.orientation) - quaternionToHeading(last_waypoint_.pose.orientation));
}
//}

/* euclideanDistance() //{ */
double ControlManagerNode::euclideanDistance(geometry_msgs::msg::Point p1, geometry_msgs::msg::Point p2) {
  return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2) + std::pow(p1.z - p2.z, 2));
}
//}

/* quaternionToHeading() //{ */
double ControlManagerNode::quaternionToHeading(geometry_msgs::msg::Quaternion &q) {
  Eigen::Quaterniond q_eigen(q.w, q.x, q.y, q.z);
  q_eigen.normalize();
  Eigen::Vector3d heading_vector = q_eigen * Eigen::Vector3d::UnitX();
  return std::atan2(heading_vector.y(), heading_vector.x());
}
//}

/* normalizeHeading() //{ */
double ControlManagerNode::normalizeHeading(double heading) {
  auto aux = std::fmod(heading + M_PI, 2.0 * M_PI);
  if (aux < 0.0) {
    aux += 2.0 * M_PI;
  }
  return aux - M_PI;
}
//}

/* checkSafeArea() //{ */
void ControlManagerNode::checkSafeArea() {
  bool                             path_is_ok = true;
  laser_msgs::msg::PoseWithHeading emergency_hover_reference;
  emergency_hover_reference.position = odometry_.pose.pose.position;

  if (current_horizon_path_[0].pose.position.x < _safe_area_.x[0]) {
    emergency_hover_reference.position.x = _safe_area_.x[0] + 0.05;
    path_is_ok                           = false;
  } else if (current_horizon_path_[0].pose.position.x > _safe_area_.x[1]) {
    emergency_hover_reference.position.x = _safe_area_.x[1] - 0.05;
    path_is_ok                           = false;
  }

  if (current_horizon_path_[0].pose.position.y < _safe_area_.y[0]) {
    emergency_hover_reference.position.y = _safe_area_.y[0];
    path_is_ok                           = false;
  } else if (current_horizon_path_[0].pose.position.y > _safe_area_.y[1]) {
    emergency_hover_reference.position.y = _safe_area_.y[1] - 0.05;
    path_is_ok                           = false;
  }

  if (requested_takeoff_ || requested_land_) {
    if (current_horizon_path_[0].pose.position.z > _safe_area_.z[1]) {
      emergency_hover_reference.position.z = _safe_area_.z[1] - 0.05;
      path_is_ok                           = false;
    }
  } else {
    if (current_horizon_path_[0].pose.position.z < _safe_area_.z[0]) {
      emergency_hover_reference.position.z = _safe_area_.z[0] + 0.05;
      path_is_ok                           = false;
    } else if (current_horizon_path_[0].pose.position.z > _safe_area_.z[1]) {
      emergency_hover_reference.position.z = _safe_area_.z[1] - 0.05;
      path_is_ok                           = false;
    }
  }


  if (!path_is_ok) {
    RCLCPP_WARN(this->get_logger(),
                "Trajectory will not be executed because the next trajectory points are outside the safe area. Entering emergency hover. Please submit a "
                "valid point within the safe area.");
    RCLCPP_WARN(this->get_logger(),
                "\nSafe Area Constraints:\n"
                " (x , y)\n"
                "(%.1f, %.1f)  --------------------- (%.1f, -%.1f)\n"
                "     |                                 |\n"
                "     |                                 |\n"
                "     |                                 |   %.1f <= z <= %.1f\n"
                "     |                                 |\n"
                "     |                                 |\n"
                "(-%.1f, %.1f) --------------------- (-%.1f, -%.1f)",
                _safe_area_.x[1], _safe_area_.y[1], _safe_area_.x[1], _safe_area_.y[0], _safe_area_.z[0], _safe_area_.z[1], _safe_area_.x[0], _safe_area_.y[1],
                _safe_area_.x[0], _safe_area_.y[0]);


    emergency_hover_reference.heading = quaternionToHeading(odometry_.pose.pose.orientation);
    agile_planner_.generateTrajectory(odometry_, emergency_hover_reference, 0.0, false);
    emergency_hover_ = true;
  }
}
//}

/* estimateMass() //{ */
bool ControlManagerNode::estimateMass() {
  if (start_mass_estimation_ && abs(odometry_.twist.twist.linear.z) < 0.02) {
    mass_estimation_time_start_ = this->get_clock()->now();
    start_mass_estimation_      = false;
  }

  if (start_mass_estimation_) {
    return false;
  }

  if (rclcpp::Duration(this->get_clock()->now() - mass_estimation_time_start_).seconds() < 3.0) {
    return false;
  }

  estimated_mass_             = (0.98 * estimated_mass_for_detect_landing_) + ((1 - 0.98) * _controller_multirotor_params_.mass);
  diagnostics_.estimated_mass = estimated_mass_;
  nmpc_controller_.setMass(estimated_mass_);
  agile_planner_.setMass(estimated_mass_);
  RCLCPP_INFO(this->get_logger(), "Estimated Calibrated Mass: %.2f", estimated_mass_);

  return true;
}
//}

/* subOdometry() //{ */
void ControlManagerNode::subOdometry(const nav_msgs::msg::Odometry &msg) {
  if (!is_active_) {
    return;
  }

  odometry_ = msg;
  diagnostics_.current_norm_speed =
      sqrt(pow(odometry_.twist.twist.linear.x, 2) + pow(odometry_.twist.twist.linear.y, 2) + pow(odometry_.twist.twist.linear.z, 2));
  received_first_odometry_msg_ = true;
}
//}

/* subImu() //{ */
void ControlManagerNode::subImu(const sensor_msgs::msg::Imu &msg) {
  if (!is_active_) {
    return;
  }

  Eigen::Vector3d current;
  current << btw_gyro_x_.iterate(msg.angular_velocity.x), btw_gyro_y_.iterate(msg.angular_velocity.y), btw_gyro_z_.iterate(msg.angular_velocity.z);

  angular_acceleration_estimated_(0) = (current(0) - last_angular_speed_(0)) / 0.004;
  angular_acceleration_estimated_(1) = (current(1) - last_angular_speed_(1)) / 0.004;
  angular_acceleration_estimated_(2) = (current(2) - last_angular_speed_(2)) / 0.004;

  last_angular_speed_ = current;
}
//}

/* subMotorSpeedStamped() //{ */
void ControlManagerNode::subMotorSpeedStamped(const laser_msgs::msg::MotorSpeedStamped &msg) {
  if (!is_active_) {
    return;
  }

  for (auto i = 0; i < (int)msg.data.data.size(); i++) {
    motor_speed_estimated_(i) = btw_motors_[i].iterate(msg.data.data[i]);
  }
}
//}

/* subApiDiagnostics() //{ */
void ControlManagerNode::subApiDiagnostics(const laser_msgs::msg::ApiPx4Diagnostics &msg) {
  if (!is_active_) {
    return;
  }

  if (msg.armed && requested_takeoff_ && msg.offboard_mode) {
    lock_control_inputs_ = false;
  }
}
//}

/* subTrajectoryPath() //{ */
void ControlManagerNode::subTrajectoryPath(const laser_msgs::msg::TrajectoryPath &msg) {
  if (!is_active_) {
    return;
  }

  if (!requested_takeoff_ && !requested_land_ && takeoff_done_) {
    agile_planner_.generateTrajectory(odometry_, msg.waypoints, msg.speed);
    stop_on_waypoints_ = msg.stop_on_waypoints;
    desired_path_      = msg.waypoints;
    emergency_hover_   = false;
    RCLCPP_INFO(this->get_logger(), "Trajectory Received!");
    diagnostics_.have_goal = true;
  } else {
    RCLCPP_WARN(this->get_logger(), "Trajectory not will executed, because the uav is not flying.");
  }
}
//}

/* subGoto() //{ */
void ControlManagerNode::subGoto(const laser_msgs::msg::PoseWithHeading &msg) {
  if (!is_active_) {
    return;
  }

  if (!requested_takeoff_ && !requested_land_ && takeoff_done_) {
    agile_planner_.generateTrajectory(odometry_, msg, 0.0, false);
    stop_on_waypoints_ = false;
    emergency_hover_   = false;
    RCLCPP_INFO(this->get_logger(), "GOTO's Point Received!");
    diagnostics_.have_goal = true;
  } else {
    RCLCPP_WARN(this->get_logger(), "GOTO's Point not will executed, because the uav is not flying.");
  }
}
//}

/* subGotoRelative() //{ */
void ControlManagerNode::subGotoRelative(const laser_msgs::msg::PoseWithHeading &msg) {
  if (!is_active_) {
    return;
  }

  if (!requested_takeoff_ && !requested_land_ && takeoff_done_) {
    tf2::Transform world_to_body_tf;
    tf2::Vector3   aux;

    tf2::fromMsg(odometry_.pose.pose, world_to_body_tf);
    tf2::fromMsg(msg.position, aux);

    laser_msgs::msg::PoseWithHeading world_point;
    aux                    = world_to_body_tf * aux;
    world_point.position.x = aux.x();
    world_point.position.y = aux.y();
    world_point.position.z = aux.z();
    world_point.heading    = quaternionToHeading(odometry_.pose.pose.orientation) + msg.heading;

    agile_planner_.generateTrajectory(odometry_, world_point, 0.0, false);
    stop_on_waypoints_ = false;
    emergency_hover_   = false;
    RCLCPP_INFO(this->get_logger(), "GOTO's Relative Point Received!");
    diagnostics_.have_goal = true;
  } else {
    RCLCPP_WARN(this->get_logger(), "GOTO's Relative Point not will executed, because the uav is not flying.");
  }
}
//}

/* srvTakeoff() //{ */
void ControlManagerNode::srvTakeoff([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                    [[maybe_unused]] std::shared_ptr<std_srvs::srv::Trigger::Response>      response) {
  if (!is_active_) {
    return;
  }

  if (!received_first_odometry_msg_) {
    response->success = false;
    response->message = "takeoff requested failed, odometry msg not received";
    RCLCPP_ERROR(this->get_logger(), "Takeoff requested failed, because the odometry msg not received!");
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
    takeoff_waypoint.position   = odometry_.pose.pose.position;
    takeoff_waypoint.position.z = _takeoff_height_;
    takeoff_waypoint.heading    = quaternionToHeading(odometry_.pose.pose.orientation);

    agile_planner_.generateTrajectory(odometry_, takeoff_waypoint, _takeoff_speed_, true);

    land_done_             = false;
    diagnostics_.have_goal = true;
    start_mass_estimation_ = true;
  }
}
//}

/* srvLand() //{ */
void ControlManagerNode::srvLand([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                 [[maybe_unused]] std::shared_ptr<std_srvs::srv::Trigger::Response>      response) {
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
    land_waypoint.position   = odometry_.pose.pose.position;
    land_waypoint.position.z = -1.0;
    land_waypoint.heading    = quaternionToHeading(odometry_.pose.pose.orientation);

    agile_planner_.generateTrajectory(odometry_, land_waypoint, _land_speed_, true);

    takeoff_done_          = false;
    diagnostics_.have_goal = true;
  }
}
//}

/* tmrExternalLoopControl() //{ */
void ControlManagerNode::tmrExternalLoopControl() {
  if (!is_active_) {
    return;
  }

  if (lock_control_inputs_) {
    return;
  }

  auto start_iteration = std::chrono::high_resolution_clock::now();

  if (land_rampdown_) {
    RCLCPP_INFO(this->get_logger(), "Land Ramp Down: %.2f", land_start_rampdown_);

    if (angular_rates_and_thrust_mode_) {
      laser_msgs::msg::AttitudeRatesAndThrust msg;
      msg.total_thrust_normalized = land_start_rampdown_;
      msg.roll_rate               = 0;
      msg.pitch_rate              = 0;
      msg.yaw_rate                = 0;

      pub_attitude_rates_and_thrust_reference_->publish(msg);
    } else {
      laser_msgs::msg::MotorSpeed msg;
      for (auto i = 0; i < nmpc_control_input_.size(); i++) {
        msg.data.push_back(land_start_rampdown_);
      }
      diagnostics_.last_control_input.data = nmpc_controller_.getLastIndividualThrust();
      pub_motor_speed_reference_->publish(msg);
    }

    diagnostics_.last_control_input.unit_of_measurement = "N";
    diagnostics_.last_control_input.data                = std::vector(
                       _controller_multirotor_params_.n_motors, laser_uav_controllers::throtleToThrust(_controller_multirotor_params_.motor_curve_a,
                                                                                                       _controller_multirotor_params_.motor_curve_b, land_start_rampdown_));

    if (land_start_rampdown_ == 0.0) {
      land_rampdown_       = false;
      lock_control_inputs_ = true;
    }

    land_start_rampdown_ -= _land_increment_rampdown_;

    if (land_start_rampdown_ < 0.0) {
      land_start_rampdown_ = 0.0;
    }

    diagnostics_.control_iteration_duration_ms = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - start_iteration).count();
    return;
  }

  if (stop_on_waypoints_ && desired_path_.size() > 0 &&
      (euclideanDistance(last_waypoint_.pose.position, desired_path_[0].position) < 0.1 &&
       std::abs(quaternionToHeading(last_waypoint_.pose.orientation) - normalizeHeading(desired_path_[0].heading)) < 0.1)) {
    if (euclideanDistance(odometry_.pose.pose.position, desired_path_[0].position) < 0.15 && checkHeadingError() < 0.2) {
      lock_waypoint_++;
    }

    if (lock_waypoint_ > 300) {
      desired_path_.erase(desired_path_.begin());
    }

    last_waypoint_.use_linear_velocity   = false;
    last_waypoint_.use_angular_velocity  = false;
    last_waypoint_.use_individual_thrust = false;

    nmpc_control_input_       = nmpc_controller_.getCorrection(last_waypoint_, odometry_);
    diagnostics_.header.stamp = get_clock()->now();
  } else {
    current_horizon_path_ = agile_planner_.getTrajectory(_acados_params_.N + 1, this->get_clock()->now().seconds());
    if (_safe_area_.enabled && diagnostics_.is_fly && !emergency_hover_) {
      checkSafeArea();
    }
    last_waypoint_ = current_horizon_path_[0];
    lock_waypoint_ = 0;

    diagnostics_.reference_horizon = current_horizon_path_;
    nmpc_control_input_            = nmpc_controller_.getCorrection(current_horizon_path_, odometry_);
    diagnostics_.header.stamp      = get_clock()->now();
  }
  have_nmpc_control_input_ = true;

  if (diagnostics_.have_goal) {
    estimated_rmse_.pushEstimated(odometry_.pose.pose.position);
    estimated_rmse_.pushReference(last_waypoint_.pose.position);

    diagnostics_.metrics.rmse = -1.0;
    diagnostics_.metrics.std  = -1.0;
  }

  if (angular_rates_and_thrust_mode_) {
    estimated_mass_for_detect_landing_ = (1 / GRAVITY) * nmpc_control_input_(0);

    laser_msgs::msg::AttitudeRatesAndThrust msg;
    msg.total_thrust_normalized =
        laser_uav_controllers::thrustToThrotle(_controller_multirotor_params_.motor_curve_a, _controller_multirotor_params_.motor_curve_b,
                                               nmpc_control_input_(0) / _controller_multirotor_params_.n_motors);
    msg.roll_rate  = nmpc_control_input_(1);
    msg.pitch_rate = nmpc_control_input_(2);
    msg.yaw_rate   = nmpc_control_input_(3);

    diagnostics_.last_control_input.unit_of_measurement = "N";
    diagnostics_.last_control_input.data                = nmpc_controller_.getLastIndividualThrust();
    pub_attitude_rates_and_thrust_reference_->publish(msg);
  }

  if (requested_takeoff_) {
    if (agile_planner_.isHover()) {
      if (estimateMass()) {
        requested_takeoff_  = false;
        takeoff_done_       = true;
        diagnostics_.is_fly = true;
        RCLCPP_INFO(this->get_logger(), "Takeoff Done!");
      }
    }
  }

  if (requested_land_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2500, "Current estimated mass for detect landing: %.3f", estimated_mass_for_detect_landing_);
    if (estimated_mass_for_detect_landing_ <= estimated_mass_ * _land_threshold_detect_ && agile_planner_.isHover()) {
      requested_land_        = false;
      land_done_             = true;
      diagnostics_.have_goal = false;
      diagnostics_.is_fly    = false;
      land_rampdown_         = true;
      land_start_rampdown_ = laser_uav_controllers::thrustToThrotle(_controller_multirotor_params_.motor_curve_a, _controller_multirotor_params_.motor_curve_b,
                                                                    (estimated_mass_ * GRAVITY) / _controller_multirotor_params_.n_motors);
      RCLCPP_INFO(this->get_logger(), "Landing Done!, Detected land with estimated mass: %.3f", estimated_mass_for_detect_landing_);
      RCLCPP_INFO(this->get_logger(), "Start Land Ramp Down!");
    } else if (agile_planner_.isHover()) {
      laser_msgs::msg::PoseWithHeading land_waypoint;
      land_waypoint.position = odometry_.pose.pose.position;
      land_waypoint.heading  = quaternionToHeading(odometry_.pose.pose.orientation);
      land_waypoint.position.z += -1.0;

      agile_planner_.generateTrajectory(odometry_, land_waypoint, _land_speed_, true);
    }
  }

  if (diagnostics_.have_goal) {
    diagnostics_.have_goal =
        (!(agile_planner_.isHover() && euclideanDistance(odometry_.pose.pose.position, last_waypoint_.pose.position) < 0.15)) || (checkHeadingError() > 0.15);

    if (!diagnostics_.have_goal) {
      calculate_rmse_ = true;
    }
  }

  diagnostics_.control_iteration_duration_ms = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - start_iteration).count();
}
//}

/* tmrInternalLoopControl() //{ */
void ControlManagerNode::tmrInternalLoopControl() {
  if (!is_active_) {
    return;
  }

  if (lock_control_inputs_) {
    return;
  }

  if (land_rampdown_) {
    return;
  }

  if (have_nmpc_control_input_) {
    Eigen::VectorXd indi_thrust =
        indi_controller_.getCorrection(angular_acceleration_estimated_, motor_speed_estimated_, nmpc_control_input_, last_angular_speed_);

    estimated_mass_for_detect_landing_ = (1 / GRAVITY) * indi_thrust.sum();

    diagnostics_.last_control_input.unit_of_measurement = "N";
    diagnostics_.last_control_input.data                = std::vector<double>(indi_thrust.data(), indi_thrust.data() + indi_thrust.size());

    laser_msgs::msg::MotorSpeed msg;
    for (auto i = 0; i < indi_thrust.size(); i++) {
      msg.data.push_back(laser_uav_controllers::thrustToThrotle(_controller_multirotor_params_.motor_curve_a, _controller_multirotor_params_.motor_curve_b,
                                                                indi_thrust(i), _controller_multirotor_params_.thrust_max,
                                                                _controller_multirotor_params_.thrust_min));
    }

    pub_motor_speed_reference_->publish(msg);
  }
}
//}

/* tmrDiagnostics() //{ */
void ControlManagerNode::tmrDiagnostics() {
  if (!is_active_) {
    return;
  }

  if (calculate_rmse_) {
    auto result               = estimated_rmse_.calculate();
    diagnostics_.metrics.rmse = result.first;
    diagnostics_.metrics.std  = result.second;
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
