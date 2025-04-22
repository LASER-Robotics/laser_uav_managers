#include "laser_uav_managers/control_manager_node.hpp"

namespace laser_uav_managers
{
/* ControlManagerNode() //{ */
ControlManagerNode::ControlManagerNode(const rclcpp::NodeOptions &options) : rclcpp_lifecycle::LifecycleNode("control_manager", "", options) {
  RCLCPP_INFO(get_logger(), "Creating");

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

  is_active_ = true;

  return CallbackReturn::SUCCESS;
}
//}

/* on_deactivate() //{ */
CallbackReturn ControlManagerNode::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Deactivating");

  is_active_ = false;

  return CallbackReturn::SUCCESS;
}
//}

/* on_clenaup() //{ */
CallbackReturn ControlManagerNode::on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Cleaning up");

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
}
//}

/* configPubSub() //{ */
void ControlManagerNode::configPubSub() {
  RCLCPP_INFO(get_logger(), "initPubSub");

  sub_odometry_ = create_subscription<nav_msgs::msg::Odometry>("odometry_in", 1, std::bind(&ControlManagerNode::subOdometry, this, std::placeholders::_1));
  sub_goto_     = create_subscription<geometry_msgs::msg::Pose>("goto_in", 1, std::bind(&ControlManagerNode::subGoto, this, std::placeholders::_1));
}
//}

/* configTimers() //{ */
void ControlManagerNode::configTimers() {
  RCLCPP_INFO(get_logger(), "initTimers");

  /* tmr_core_control_ = */
  /*     create_wall_timer(std::chrono::duration<double>(1.0 / _rate_core_control_), std::bind(&ControlManagerNode::tmrCoreControl, this), nullptr); */

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

  /* nmpc_controller_ = laser_uav_controllers::NmpcController(); */
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

  nmpc_controller_.getCorrection(msg, odometry_);
}
//}

/* tmrCoreControl() //{ */
void ControlManagerNode::tmrCoreControl() {
  if (!is_active_) {
    return;
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
