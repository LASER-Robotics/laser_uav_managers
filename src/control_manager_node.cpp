#include "laser_uav_managers/control_manager_node.hpp"

namespace laser_uav_managers
{
/* ControlManagerNode() //{ */
ControlManagerNode::ControlManagerNode(const rclcpp::NodeOptions &options) : rclcpp_lifecycle::LifecycleNode("control_manager", "", options) {
  RCLCPP_INFO(get_logger(), "Creating");

  declare_parameter("agile_fly", rclcpp::ParameterValue(false));

  declare_parameter("rate.external_loop_control", rclcpp::ParameterValue(1.0));
  declare_parameter("rate.internal_loop_control", rclcpp::ParameterValue(1.0));
  declare_parameter("rate.diagnostics", rclcpp::ParameterValue(1.0));

  declare_parameter("takeoff.height", rclcpp::ParameterValue(0.0));
  declare_parameter("takeoff.speed", rclcpp::ParameterValue(0.0));

  declare_parameter("filter_params.butterworth.gyro_a", rclcpp::ParameterValue(std::vector<float_t>(3, 0.0)));
  declare_parameter("filter_params.butterworth.gyro_b", rclcpp::ParameterValue(std::vector<float_t>(3, 0.0)));

  declare_parameter("filter_params.butterworth.motor_a", rclcpp::ParameterValue(std::vector<float_t>(3, 0.0)));
  declare_parameter("filter_params.butterworth.motor_b", rclcpp::ParameterValue(std::vector<float_t>(3, 0.0)));

  declare_parameter("agile_planner.quadrotor_parameters.max_acc", rclcpp::ParameterValue(0.0));
  declare_parameter("agile_planner.quadrotor_parameters.max_vel", rclcpp::ParameterValue(0.0));
  declare_parameter("agile_planner.quadrotor_parameters.default_vel", rclcpp::ParameterValue(0.5));

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

  declare_parameter("nmpc_controller.quadrotor_parameters.mass", rclcpp::ParameterValue(0.0));
  declare_parameter("nmpc_controller.quadrotor_parameters.inertia", rclcpp::ParameterValue(std::vector<float_t>(3, 0.0)));
  declare_parameter("nmpc_controller.quadrotor_parameters.motor_inertia", rclcpp::ParameterValue(0.0));
  declare_parameter("nmpc_controller.quadrotor_parameters.c_thrust", rclcpp::ParameterValue(0.0));
  declare_parameter("nmpc_controller.quadrotor_parameters.c_tau", rclcpp::ParameterValue(0.0));
  declare_parameter("nmpc_controller.quadrotor_parameters.drag", rclcpp::ParameterValue(std::vector<float_t>(3, 0.0)));
  declare_parameter("nmpc_controller.quadrotor_parameters.n_motors", rclcpp::ParameterValue(0));
  declare_parameter("nmpc_controller.quadrotor_parameters.G1", rclcpp::ParameterValue(std::vector<float_t>(32, 0.0)));
  declare_parameter("nmpc_controller.quadrotor_parameters.G2", rclcpp::ParameterValue(std::vector<float_t>(32, 0.0)));
  declare_parameter("nmpc_controller.quadrotor_parameters.quadratic_motor_model.a", rclcpp::ParameterValue(0.0));
  declare_parameter("nmpc_controller.quadrotor_parameters.quadratic_motor_model.b", rclcpp::ParameterValue(0.0));
  declare_parameter("nmpc_controller.quadrotor_parameters.thrust_min", rclcpp::ParameterValue(0.0));
  declare_parameter("nmpc_controller.quadrotor_parameters.thrust_max", rclcpp::ParameterValue(0.0));
  declare_parameter("nmpc_controller.quadrotor_parameters.total_thrust_max", rclcpp::ParameterValue(0.0));

  declare_parameter("nmpc_controller.acados_parameters.nmpc_mode", rclcpp::ParameterValue(""));
  declare_parameter("nmpc_controller.acados_parameters.N", rclcpp::ParameterValue(0));
  declare_parameter("nmpc_controller.acados_parameters.dt", rclcpp::ParameterValue(0.0));
  declare_parameter("nmpc_controller.acados_parameters.Q", rclcpp::ParameterValue(std::vector<float_t>(6, 0.0)));
  declare_parameter("nmpc_controller.acados_parameters.R", rclcpp::ParameterValue(0.0));

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

  get_parameter("agile_fly", _agile_fly_);

  get_parameter("rate.external_loop_control", _rate_external_loop_control_);
  get_parameter("rate.internal_loop_control", _rate_internal_loop_control_);
  get_parameter("rate.diagnostics", _rate_diagnostics_);

  get_parameter("takeoff.height", _takeoff_height_);
  get_parameter("takeoff.speed", _takeoff_speed_);

  get_parameter("filter_params.butterworth.gyro_a", aux);
  _gyro_a_ = aux.as_double_array();
  get_parameter("filter_params.butterworth.gyro_b", aux);
  _gyro_b_ = aux.as_double_array();

  get_parameter("filter_params.butterworth.motor_a", aux);
  _motor_a_ = aux.as_double_array();
  get_parameter("filter_params.butterworth.motor_b", aux);
  _motor_b_ = aux.as_double_array();

  get_parameter("agile_planner.quadrotor_parameters.max_acc", _pmm_params_.max_acc_norm);
  get_parameter("agile_planner.quadrotor_parameters.max_vel", _pmm_params_.max_vel_norm);
  get_parameter("agile_planner.quadrotor_parameters.default_vel", _pmm_params_.default_vel_norm);

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

  get_parameter("nmpc_controller.quadrotor_parameters.mass", _controller_quadrotor_params_.mass);
  _planner_quadrotor_params_.mass = _controller_quadrotor_params_.mass;

  get_parameter("nmpc_controller.quadrotor_parameters.inertia", aux);
  _controller_quadrotor_params_.inertia_matrix = _planner_quadrotor_params_.inertia_matrix =
      Eigen::Map<const Eigen::Vector3d>(aux.as_double_array().data(), aux.as_double_array().size()).asDiagonal();
  /* _controller_quadrotor_params_.inertia_x = _planner_quadrotor_params_.inertia_x = (aux.as_double_array())[0]; */
  /* _controller_quadrotor_params_.inertia_y = _planner_quadrotor_params_.inertia_y = (aux.as_double_array())[1]; */
  /* _controller_quadrotor_params_.inertia_z = _planner_quadrotor_params_.inertia_z = (aux.as_double_array())[2]; */

  get_parameter("nmpc_controller.quadrotor_parameters.c_thrust", _controller_quadrotor_params_.c_thrust);
  /* get_parameter("nmpc_controller.quadrotor_parameters.c_tau", _controller_quadrotor_params_.c_tau); */
  /* _planner_quadrotor_params_.c_tau = _controller_quadrotor_params_.c_tau; */

  get_parameter("nmpc_controller.quadrotor_parameters.drag", aux);
  _controller_quadrotor_params_.drag = Eigen::Map<const Eigen::Vector3d>(aux.as_double_array().data(), aux.as_double_array().size());
  /* _controller_quadrotor_params_.drag_x = (aux.as_double_array())[0]; */
  /* _controller_quadrotor_params_.drag_y = (aux.as_double_array())[1]; */
  /* _controller_quadrotor_params_.drag_z = (aux.as_double_array())[1]; */

  get_parameter("nmpc_controller.quadrotor_parameters.n_motors", _controller_quadrotor_params_.n_motors);
  get_parameter("nmpc_controller.quadrotor_parameters.G1", aux);
  _controller_quadrotor_params_.G1 = _planner_quadrotor_params_.G1 = Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      aux.as_double_array().data(), 4, _controller_quadrotor_params_.n_motors);
  get_parameter("nmpc_controller.quadrotor_parameters.G2", aux);
  _controller_quadrotor_params_.G2 = Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      aux.as_double_array().data(), 4, _controller_quadrotor_params_.n_motors);

  get_parameter("nmpc_controller.quadrotor_parameters.motor_inertia", _controller_quadrotor_params_.motor_inertia);

  get_parameter("nmpc_controller.quadrotor_parameters.quadratic_motor_model.a", _controller_quadrotor_params_.motor_curve_a);
  get_parameter("nmpc_controller.quadrotor_parameters.quadratic_motor_model.b", _controller_quadrotor_params_.motor_curve_b);
  get_parameter("nmpc_controller.quadrotor_parameters.thrust_min", _controller_quadrotor_params_.thrust_min);
  get_parameter("nmpc_controller.quadrotor_parameters.thrust_max", _controller_quadrotor_params_.thrust_max);
  get_parameter("nmpc_controller.quadrotor_parameters.total_thrust_max", _controller_quadrotor_params_.total_thrust_max);

  get_parameter("nmpc_controller.acados_parameters.nmpc_mode", _acados_params_.nmpc_mode);
  if (_acados_params_.nmpc_mode == "individual_thrust") {
    angular_rates_and_thrust_mode_ = false;
  } else {
    angular_rates_and_thrust_mode_ = true;
  }

  get_parameter("nmpc_controller.acados_parameters.N", _acados_params_.N);
  get_parameter("nmpc_controller.acados_parameters.dt", _acados_params_.dt);

  get_parameter("nmpc_controller.acados_parameters.Q", aux);
  _acados_params_.Q = aux.as_double_array();

  get_parameter("nmpc_controller.acados_parameters.R", _acados_params_.R);

  nmpc_control_input_    = Eigen::VectorXd(_controller_quadrotor_params_.n_motors);
  motor_speed_estimated_ = Eigen::VectorXd(_controller_quadrotor_params_.n_motors);
}
//}

/* configPubSub() //{ */
void ControlManagerNode::configPubSub() {
  RCLCPP_INFO(get_logger(), "initPubSub");

  sub_odometry_ = create_subscription<nav_msgs::msg::Odometry>("odometry_in", 1, std::bind(&ControlManagerNode::subOdometry, this, std::placeholders::_1));
  sub_goto_     = create_subscription<laser_msgs::msg::PoseWithHeading>("goto_in", 1, std::bind(&ControlManagerNode::subGoto, this, std::placeholders::_1));
  sub_api_diagnostics_ = create_subscription<laser_msgs::msg::ApiPx4Diagnostics>(
      "api_diagnostics_in", 1, std::bind(&ControlManagerNode::subApiDiagnostics, this, std::placeholders::_1));
  sub_trajectory_path_ = create_subscription<laser_msgs::msg::TrajectoryPath>("trajectory_path_in", 1,
                                                                              std::bind(&ControlManagerNode::subTrajectoryPath, this, std::placeholders::_1));

  if (angular_rates_and_thrust_mode_) {
    pub_attitude_rates_and_thrust_reference_ = create_publisher<laser_msgs::msg::AttitudeRatesAndThrust>("attitude_rates_thrust_out", 10);
  } else {
    sub_imu_                   = create_subscription<sensor_msgs::msg::Imu>("imu_in", 1, std::bind(&ControlManagerNode::subImu, this, std::placeholders::_1));
    sub_motor_speed_           = create_subscription<laser_msgs::msg::MotorSpeed>("motor_speed_estimation_in", 1,
                                                                        std::bind(&ControlManagerNode::subMotorSpeed, this, std::placeholders::_1));
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

  agile_planner_   = laser_uav_planners::AgilePlanner(_planner_quadrotor_params_, _pmm_params_);
  nmpc_controller_ = laser_uav_controllers::NmpcController(_controller_quadrotor_params_, _acados_params_);
  if (!angular_rates_and_thrust_mode_) {
    btw_gyro_x_ = laser_uav_lib::IIRFilter(_gyro_a_, _gyro_b_);
    btw_gyro_y_ = laser_uav_lib::IIRFilter(_gyro_a_, _gyro_b_);
    btw_gyro_z_ = laser_uav_lib::IIRFilter(_gyro_a_, _gyro_b_);

    for (auto i = 0; i < _controller_quadrotor_params_.n_motors; i++) {
      btw_motors_.push_back(laser_uav_lib::IIRFilter(_motor_a_, _motor_b_));
    }

    indi_controller_ = laser_uav_controllers::IndiController(_controller_quadrotor_params_);
  }
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

/* subMotorSpeed() //{ */
void ControlManagerNode::subMotorSpeed(const laser_msgs::msg::MotorSpeed &msg) {
  if (!is_active_) {
    return;
  }

  for (auto i = 0; i < (int)msg.data.size(); i++) {
    motor_speed_estimated_(i) = btw_motors_[i].iterate(msg.data[i]);
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
    agile_planner_.generateTrajectory(last_waypoint_, msg.waypoints, msg.speed);
    desired_path_ = msg.waypoints;
    RCLCPP_INFO(this->get_logger(), "Trajectory Received!");
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
    agile_planner_.generateTrajectory(last_waypoint_, msg, 0.0, false);
    RCLCPP_INFO(this->get_logger(), "GOTO's Point Received!");
  } else {
    RCLCPP_WARN(this->get_logger(), "GOTO's Point not will executed, because the uav is not flying.");
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

    laser_msgs::msg::ReferenceState ground_waypoint;
    ground_waypoint.pose = odometry_.pose.pose;

    laser_msgs::msg::PoseWithHeading takeoff_waypoint;
    takeoff_waypoint.position   = ground_waypoint.pose.position;
    takeoff_waypoint.position.z = _takeoff_height_;

    Eigen::Quaterniond q(ground_waypoint.pose.orientation.w, ground_waypoint.pose.orientation.x, ground_waypoint.pose.orientation.y,
                         ground_waypoint.pose.orientation.z);
    q.normalize();
    takeoff_waypoint.heading = (q.toRotationMatrix().eulerAngles(2, 1, 0))[0];

    agile_planner_.generateTrajectory(ground_waypoint, takeoff_waypoint, _takeoff_speed_, true);

    land_done_ = false;
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

    laser_msgs::msg::ReferenceState current_pose;
    current_pose.pose = odometry_.pose.pose;

    laser_msgs::msg::PoseWithHeading land_waypoint;
    land_waypoint.position   = odometry_.pose.pose.position;
    land_waypoint.position.z = -1.0;

    Eigen::Quaterniond q(current_pose.pose.orientation.w, current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z);
    q.normalize();
    land_waypoint.heading = (q.toRotationMatrix().eulerAngles(2, 1, 0))[0];

    agile_planner_.generateTrajectory(current_pose, land_waypoint, 0.2, true);

    takeoff_done_ = false;
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

  if (!_agile_fly_) {
    if (desired_path_.size() > 0) {
      if (!(last_waypoint_.pose.position.x == desired_path_[0].position.x && last_waypoint_.pose.position.y == desired_path_[0].position.y &&
            last_waypoint_.pose.position.z == desired_path_[0].position.z)) {
        current_horizon_path_ = agile_planner_.getTrajectory(_acados_params_.N);
        last_waypoint_        = current_horizon_path_[0];

        current_horizon_path_[0].twist.linear.x = 0;
        current_horizon_path_[0].twist.linear.y = 0;
        current_horizon_path_[0].twist.linear.z = 0;

        lock_waypoint_ = 0;
      } else {
        if (sqrt(pow(odometry_.pose.pose.position.x - desired_path_[0].position.x, 2) + pow(odometry_.pose.pose.position.y - desired_path_[0].position.y, 2) +
                 pow(odometry_.pose.pose.position.z - desired_path_[0].position.z, 2)) < 0.15 &&
            lock_waypoint_ > 300) {
          desired_path_.erase(desired_path_.begin());
        } else {
          lock_waypoint_++;
        }
      }
    } else {
      current_horizon_path_ = agile_planner_.getTrajectory(_acados_params_.N + 1);
      last_waypoint_        = current_horizon_path_[0];
    }
  } else {
    current_horizon_path_ = agile_planner_.getTrajectory(_acados_params_.N + 1);
    last_waypoint_        = current_horizon_path_[0];
  }

  nmpc_control_input_      = nmpc_controller_.getCorrection(current_horizon_path_, odometry_);
  have_nmpc_control_input_ = true;

  diagnostics_.last_planner_waypoint = last_waypoint_;

  if (angular_rates_and_thrust_mode_) {
    estimated_mass_for_detect_landing_ = (1 / 9.81) * nmpc_control_input_(0);

    laser_msgs::msg::AttitudeRatesAndThrust msg;
    msg.total_thrust_normalized =
        laser_uav_controllers::thrustToThrotle(_controller_quadrotor_params_.motor_curve_a, _controller_quadrotor_params_.motor_curve_b,
                                               nmpc_control_input_(0) / _controller_quadrotor_params_.n_motors);
    msg.roll_rate  = nmpc_control_input_(1);
    msg.pitch_rate = nmpc_control_input_(2);
    msg.yaw_rate   = nmpc_control_input_(3);

    diagnostics_.last_control_input.unit_of_measurement = "N";
    diagnostics_.last_control_input.data                = nmpc_controller_.getLastIndividualThrust();
    pub_attitude_rates_and_thrust_reference_->publish(msg);
  }

  if (requested_takeoff_) {
    if (abs(odometry_.pose.pose.position.z - _takeoff_height_) <= 0.3) {
      requested_takeoff_  = false;
      takeoff_done_       = true;
      diagnostics_.is_fly = true;
      RCLCPP_INFO(this->get_logger(), "Takeoff Done!");
    }
  }

  if (requested_land_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2500, "Current estimated mass for detect landing: %.3f", estimated_mass_for_detect_landing_);
    if (estimated_mass_for_detect_landing_ <= _controller_quadrotor_params_.mass * 0.70) {
      requested_land_     = false;
      land_done_          = true;
      diagnostics_.is_fly = false;
      RCLCPP_INFO(this->get_logger(), "Landing Done!, Detected land with estimated mass: %.3f", estimated_mass_for_detect_landing_);
    } else if (agile_planner_.isHover()) {
      laser_msgs::msg::PoseWithHeading land_waypoint;
      land_waypoint.position = last_waypoint_.pose.position;
      Eigen::Quaterniond q(last_waypoint_.pose.orientation.w, last_waypoint_.pose.orientation.x, last_waypoint_.pose.orientation.y,
                           last_waypoint_.pose.orientation.z);
      q.normalize();
      land_waypoint.heading = (q.toRotationMatrix().eulerAngles(2, 1, 0))[0];
      land_waypoint.position.z += -1.0;

      agile_planner_.generateTrajectory(last_waypoint_, land_waypoint, 0.2, true);
    }
  }
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

  if (have_nmpc_control_input_) {
    Eigen::VectorXd indi_thrust =
        indi_controller_.getCorrection(angular_acceleration_estimated_, motor_speed_estimated_, nmpc_control_input_, last_angular_speed_);

    estimated_mass_for_detect_landing_ = (1 / 9.81) * indi_thrust.sum();

    diagnostics_.last_control_input.unit_of_measurement = "N";
    diagnostics_.last_control_input.data                = std::vector<double>(indi_thrust.data(), indi_thrust.data() + indi_thrust.size());

    laser_msgs::msg::MotorSpeed msg;
    for (auto i = 0; i < indi_thrust.size(); i++) {
      msg.data.push_back(laser_uav_controllers::thrustToThrotle(_controller_quadrotor_params_.motor_curve_a, _controller_quadrotor_params_.motor_curve_b,
                                                                indi_thrust(i), _controller_quadrotor_params_.thrust_max,
                                                                _controller_quadrotor_params_.thrust_min));
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

  diagnostics_.header.stamp    = get_clock()->now();
  diagnostics_.header.frame_id = "";

  pub_diagnostics_->publish(diagnostics_);
}
//}
}  // namespace laser_uav_managers

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(laser_uav_managers::ControlManagerNode)
