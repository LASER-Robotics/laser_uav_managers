#include "laser_uav_managers/control_manager_node.hpp"

namespace laser_uav_managers
{
/* ControlManagerNode() //{ */
ControlManagerNode::ControlManagerNode(const rclcpp::NodeOptions &options) : rclcpp_lifecycle::LifecycleNode("control_manager", "", options) {
  RCLCPP_INFO(get_logger(), "Creating");

  declare_parameter("rate.loop_control", rclcpp::ParameterValue(1.0));
  declare_parameter("rate.diagnostics", rclcpp::ParameterValue(1.0));

  declare_parameter("takeoff.height", rclcpp::ParameterValue(0.0));
  declare_parameter("takeoff.speed", rclcpp::ParameterValue(0.0));

  declare_parameter("agile_planner.quadrotor_parameters.max_acc", rclcpp::ParameterValue(0.0));
  declare_parameter("agile_planner.quadrotor_parameters.max_vel", rclcpp::ParameterValue(0.0));

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
  declare_parameter("nmpc_controller.quadrotor_parameters.c_tau", rclcpp::ParameterValue(0.0));
  declare_parameter("nmpc_controller.quadrotor_parameters.drag", rclcpp::ParameterValue(std::vector<float_t>(3, 0.0)));
  declare_parameter("nmpc_controller.quadrotor_parameters.motors_positions", rclcpp::ParameterValue(std::vector<float_t>(8, 0.0)));
  declare_parameter("nmpc_controller.quadrotor_parameters.quadratic_motor_model.a", rclcpp::ParameterValue(0.0));
  declare_parameter("nmpc_controller.quadrotor_parameters.quadratic_motor_model.b", rclcpp::ParameterValue(0.0));
  declare_parameter("nmpc_controller.quadrotor_parameters.thrust_min", rclcpp::ParameterValue(0.0));
  declare_parameter("nmpc_controller.quadrotor_parameters.thrust_max", rclcpp::ParameterValue(0.0));
  declare_parameter("nmpc_controller.quadrotor_parameters.total_thrust_max", rclcpp::ParameterValue(0.0));

  declare_parameter("nmpc_controller.acados_parameters.Q", rclcpp::ParameterValue(std::vector<float_t>(6, 0.0)));
  declare_parameter("nmpc_controller.acados_parameters.R", rclcpp::ParameterValue(0.0));

  odometry_ = nav_msgs::msg::Odometry();
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

  pub_attitude_rates_and_thrust_reference_->on_activate();
  pub_current_waypoint_->on_activate();

  is_active_ = true;

  return CallbackReturn::SUCCESS;
}
//}

/* on_deactivate() //{ */
CallbackReturn ControlManagerNode::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Deactivating");

  pub_attitude_rates_and_thrust_reference_->on_deactivate();

  is_active_ = false;

  return CallbackReturn::SUCCESS;
}
//}

/* on_clenaup() //{ */
CallbackReturn ControlManagerNode::on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Cleaning up");

  pub_attitude_rates_and_thrust_reference_.reset();

  sub_odometry_.reset();
  sub_goto_.reset();
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

  get_parameter("rate.loop_control", _rate_loop_control_);
  /* get_parameter("rate.diagnostics", ); */

  get_parameter("takeoff.height", _takeoff_height_);
  get_parameter("takeoff.speed", _takeoff_speed_);

  get_parameter("agile_planner.quadrotor_parameters.max_acc", _pmm_params_.max_acc_norm);
  get_parameter("agile_planner.quadrotor_parameters.max_vel", _pmm_params_.max_vel_norm);

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

  get_parameter("nmpc_controller.quadrotor_parameters.mass", _quadrotor_params_.mass);
  get_parameter("nmpc_controller.quadrotor_parameters.inertia", aux);
  _quadrotor_params_.inertia_x = (aux.as_double_array())[0];
  _quadrotor_params_.inertia_y = (aux.as_double_array())[1];
  _quadrotor_params_.inertia_z = (aux.as_double_array())[2];

  get_parameter("nmpc_controller.quadrotor_parameters.c_tau", _quadrotor_params_.c_tau);
  get_parameter("nmpc_controller.quadrotor_parameters.drag", aux);
  _quadrotor_params_.drag_x = (aux.as_double_array())[0];
  _quadrotor_params_.drag_y = (aux.as_double_array())[1];
  _quadrotor_params_.drag_z = (aux.as_double_array())[1];

  get_parameter("nmpc_controller.quadrotor_parameters.motors_positions", aux);
  _quadrotor_params_.motor_position_0[0] = (aux.as_double_array())[0];
  _quadrotor_params_.motor_position_0[1] = (aux.as_double_array())[1];
  _quadrotor_params_.motor_position_1[0] = (aux.as_double_array())[2];
  _quadrotor_params_.motor_position_1[1] = (aux.as_double_array())[3];
  _quadrotor_params_.motor_position_2[0] = (aux.as_double_array())[4];
  _quadrotor_params_.motor_position_2[1] = (aux.as_double_array())[5];
  _quadrotor_params_.motor_position_3[0] = (aux.as_double_array())[6];
  _quadrotor_params_.motor_position_3[1] = (aux.as_double_array())[7];

  get_parameter("nmpc_controller.quadrotor_parameters.quadratic_motor_model.a", _quadrotor_params_.motor_curve_a);
  get_parameter("nmpc_controller.quadrotor_parameters.quadratic_motor_model.b", _quadrotor_params_.motor_curve_b);
  get_parameter("nmpc_controller.quadrotor_parameters.thrust_min", _quadrotor_params_.thrust_min);
  get_parameter("nmpc_controller.quadrotor_parameters.thrust_max", _quadrotor_params_.thrust_max);
  get_parameter("nmpc_controller.quadrotor_parameters.total_thrust_max", _quadrotor_params_.total_thrust_max);

  get_parameter("nmpc_controller.acados_parameters.Q", aux);
  _acados_params_.Q = aux.as_double_array();

  get_parameter("nmpc_controller.acados_parameters.R", _acados_params_.R);
}
//}

/* configPubSub() //{ */
void ControlManagerNode::configPubSub() {
  RCLCPP_INFO(get_logger(), "initPubSub");

  sub_odometry_ = create_subscription<nav_msgs::msg::Odometry>("odometry_in", 1, std::bind(&ControlManagerNode::subOdometry, this, std::placeholders::_1));
  sub_goto_     = create_subscription<geometry_msgs::msg::Pose>("goto_in", 1, std::bind(&ControlManagerNode::subGoto, this, std::placeholders::_1));
  sub_trajectory_path_ = create_subscription<laser_msgs::msg::TrajectoryPath>("trajectory_path_in", 1,
                                                                              std::bind(&ControlManagerNode::subTrajectoryPath, this, std::placeholders::_1));

  pub_attitude_rates_and_thrust_reference_ = create_publisher<laser_msgs::msg::AttitudeRatesAndThrust>("/uav1/attitude_rates_thrust_in", 10);
  pub_current_waypoint_                    = create_publisher<laser_msgs::msg::ReferenceState>("/uav1/trajectory_debug", 10);
}
//}

/* configTimers() //{ */
void ControlManagerNode::configTimers() {
  RCLCPP_INFO(get_logger(), "initTimers");

  tmr_loop_control_ =
      create_wall_timer(std::chrono::duration<double>(1.0 / _rate_loop_control_), std::bind(&ControlManagerNode::tmrLoopControl, this), nullptr);

  /* tmr_diagnostics_ = create_wall_timer(std::chrono::duration<double>(1.0 / _rate_diagnostics_), std::bind(&ControlManagerNode::tmrDiagnostics, this),
   * nullptr); */
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

  agile_planner_   = laser_uav_planner::AgilePlanner(_pmm_params_);
  nmpc_controller_ = laser_uav_controllers::NmpcController(_quadrotor_params_, _acados_params_);
}
//}

/* subOdometry() //{ */
void ControlManagerNode::subOdometry(const nav_msgs::msg::Odometry &msg) {
  if (!is_active_) {
    return;
  }

  odometry_ = msg;
}
//}

/* subTrajectoryPath() //{ */
void ControlManagerNode::subTrajectoryPath(const laser_msgs::msg::TrajectoryPath &msg) {
  if (!is_active_) {
    return;
  }

  if (!requested_takeoff_ && !requested_land_ && takeoff_done_) {
    agile_planner_.generateTrajectory(last_waypoint_, msg.waypoints, msg.speed);
  }
}
//}

/* subGoto() //{ */
void ControlManagerNode::subGoto(const geometry_msgs::msg::Pose &msg) {
  if (!is_active_) {
    return;
  }

  if (!requested_takeoff_ && !requested_land_ && takeoff_done_) {
    agile_planner_.generateTrajectory(last_waypoint_, msg, 0.0, false);
  }
}
//}

/* srvTakeoff() //{ */
void ControlManagerNode::srvTakeoff([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                    [[maybe_unused]] std::shared_ptr<std_srvs::srv::Trigger::Response>      response) {
  if (!is_active_) {
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

    geometry_msgs::msg::Pose takeoff_waypoint;
    takeoff_waypoint            = last_waypoint_.pose;
    takeoff_waypoint.position.z = _takeoff_height_;

    agile_planner_.generateTrajectory(ground_waypoint, takeoff_waypoint, _takeoff_speed_, true);

    land_done_           = false;
    lock_control_inputs_ = false;
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

    geometry_msgs::msg::Pose land_waypoint;
    land_waypoint            = odometry_.pose.pose;
    land_waypoint.position.z = -0.2;

    agile_planner_.generateTrajectory(current_pose, land_waypoint, 0.2, true);

    takeoff_done_ = false;
  }
}
//}

/* tmrLoopControl() //{ */
void ControlManagerNode::tmrLoopControl() {
  if (!is_active_) {
    return;
  }

  if (lock_control_inputs_) {
    return;
  }

  std::vector<laser_msgs::msg::ReferenceState> current_trajectory_waypoints = agile_planner_.getTrajectory(30);
  last_waypoint_                                                            = current_trajectory_waypoints[0];

  std::cout << "Current Waypoint: x: " << last_waypoint_.pose.position.x << ", y: " << last_waypoint_.pose.position.y
            << ", z:" << last_waypoint_.pose.position.z << ", w: " << last_waypoint_.pose.orientation.w << ", x: " << last_waypoint_.pose.orientation.x
            << ", y: " << last_waypoint_.pose.orientation.y << ", z: " << last_waypoint_.pose.orientation.z << std::endl;

  laser_msgs::msg::AttitudeRatesAndThrust msg = nmpc_controller_.getCorrection(current_trajectory_waypoints, odometry_);
  pub_attitude_rates_and_thrust_reference_->publish(msg);
  pub_current_waypoint_->publish(last_waypoint_);

  if (requested_takeoff_) {
    if (odometry_.pose.pose.position.z - _takeoff_height_ <= 0.3) {
      requested_takeoff_ = false;
      takeoff_done_      = true;
    }
  }

  if (requested_land_) {
    if (odometry_.pose.pose.position.z - 0.0 <= 0.3) {
      requested_land_ = false;
      land_done_      = true;
      /* lock_control_inputs_ = true; */
    }
  }
}
//}

/* tmrDiagnostics() //{ */
void ControlManagerNode::tmrDiagnostics() {
  if (!is_active_) {
    return;
  }
}
//}
}  // namespace laser_uav_managers

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(laser_uav_managers::ControlManagerNode)
