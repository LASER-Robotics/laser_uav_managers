#include "laser_uav_managers/control_manager_node.hpp"

namespace laser_uav_managers
{
/* ControlManagerNode() //{ */
ControlManagerNode::ControlManagerNode(const rclcpp::NodeOptions &options) : rclcpp_lifecycle::LifecycleNode("control_manager", "", options) {
  RCLCPP_INFO(get_logger(), "Creating");

  declare_parameter("rate.core_control", rclcpp::ParameterValue(10.0));
  declare_parameter("rate.diagnostics", rclcpp::ParameterValue(10.0));

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

  odometry_                        = nav_msgs::msg::Odometry();
  current_reference_               = geometry_msgs::msg::Pose();
  current_reference_.orientation.w = 1;
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

  /* get_parameter("rate.core_control", rclcpp::ParameterValue(10.0)); */
  /* get_parameter("rate.diagnostics", rclcpp::ParameterValue(10.0)); */

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

  pub_attitude_rates_and_thrust_reference_ = create_publisher<laser_msgs::msg::AttitudeRatesAndThrust>("/uav1/attitude_rates_thrust_in", 10);
}
//}

/* configTimers() //{ */
void ControlManagerNode::configTimers() {
  RCLCPP_INFO(get_logger(), "initTimers");

  tmr_core_control_ =
      /* create_wall_timer(std::chrono::duration<double>(1.0 / _rate_core_control_), std::bind(&ControlManagerNode::tmrCoreControl, this), nullptr); */
      create_wall_timer(std::chrono::duration<double>(1.0 / 100), std::bind(&ControlManagerNode::tmrCoreControl, this), nullptr);

  /* tmr_diagnostics_ = create_wall_timer(std::chrono::duration<double>(1.0 / _rate_diagnostics_), std::bind(&ControlManagerNode::tmrDiagnostics, this),
   * nullptr); */
}
//}

/* configServices() //{ */
void ControlManagerNode::configServices() {
  RCLCPP_INFO(get_logger(), "initServices");
}
//}

/* configClasses() //{ */
void ControlManagerNode::configClasses() {
  RCLCPP_INFO(get_logger(), "initClasses");

  waypoint_tracker_ = laser_uav_trackers::WaypointTracker();
  waypoint_tracker_.setInitialAndEndWaypoint(current_reference_, current_reference_);
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

/* subGoto() //{ */
void ControlManagerNode::subGoto(const geometry_msgs::msg::Pose &msg) {
  if (!is_active_) {
    return;
  }

  current_reference_ = msg;
  waypoint_tracker_.setInitialAndEndWaypoint(odometry_.pose.pose, current_reference_);

}
//}

/* tmrCoreControl() //{ */
void ControlManagerNode::tmrCoreControl() {
  if (!is_active_) {
    return;
  }

  geometry_msgs::msg::Pose current_waypoint = waypoint_tracker_.updateReference(odometry_);
  std::cout << "Current Waypoint: x: " << current_waypoint.position.x << ", y: " << current_waypoint.position.y << ", z:" << current_waypoint.position.z << ", w: " << current_waypoint.orientation.w << ", x: " << current_waypoint.orientation.x << ", y: " << current_waypoint.orientation.y << ", z: " << current_waypoint.orientation.z << std::endl;
  std::cout << "Current Reference: x: " << current_reference_.position.x << ", y: " << current_reference_.position.y << ", z:" << current_reference_.position.z << ", w: " << current_reference_.orientation.w << ", x: " << current_reference_.orientation.x << ", y: " << current_reference_.orientation.y << ", z: " << current_reference_.orientation.z << std::endl;
  laser_msgs::msg::AttitudeRatesAndThrust msg = nmpc_controller_.getCorrection(current_waypoint, odometry_);
  pub_attitude_rates_and_thrust_reference_->publish(msg);
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
