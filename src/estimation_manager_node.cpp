#include <laser_uav_managers/estimation_manager_node.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace laser_uav_managers
{
/* EstimationManager() //{ */
EstimationManager::EstimationManager(const rclcpp::NodeOptions &options) : rclcpp_lifecycle::LifecycleNode("state_estimator", options) {
  RCLCPP_INFO(get_logger(), "Creating EstimationManager...");

  declare_parameter("frequency", 100.0);
  declare_parameter("initial_odometry_source", "px4_api_odom");
  declare_parameter("odometry_source_names", std::vector<std::string>{});
  declare_parameter("odometry_switch_distance_threshold", 0.25);
  declare_parameter("odometry_switch_angle_threshold", 0.5);
  declare_parameter("odometry_switch_velocity_linear_threshold", 0.5);
  declare_parameter("odometry_switch_velocity_angular_threshold", 1.0);
  declare_parameter("sensor_timeout", 0.5);
  declare_parameter("ekf_verbosity", "INFO");

  declare_parameter("multirotor_parameters.mass", 1.0);
  declare_parameter("multirotor_parameters.inertia", std::vector<double>{0.01, 0.01, 0.01});
  declare_parameter("multirotor_parameters.G1", std::vector<double>{0.1, -0.1, -0.1, 0.1, 0.1, 0.1, -0.1, -0.1});

  declare_parameter("process_noise_gains.position", 0.01);
  declare_parameter("process_noise_gains.orientation", 0.01);
  declare_parameter("process_noise_gains.linear_velocity", 0.1);
  declare_parameter("process_noise_gains.angular_velocity", 0.1);

  declare_parameter("measurement_noise_gains.px4_odometry.position", 1.0);
  declare_parameter("measurement_noise_gains.px4_odometry.orientation", 1.0);
  declare_parameter("measurement_noise_gains.px4_odometry.linear_velocity", 1.0);
  declare_parameter("measurement_noise_gains.px4_odometry.angular_velocity", 1.0);
  declare_parameter("measurement_noise_gains.openvins.position", 1.0);
  declare_parameter("measurement_noise_gains.openvins.orientation", 1.0);
  declare_parameter("measurement_noise_gains.openvins.linear_velocity", 1.0);
  declare_parameter("measurement_noise_gains.openvins.angular_velocity", 1.0);
  declare_parameter("measurement_noise_gains.fast_lio.position", 1.0);
  declare_parameter("measurement_noise_gains.fast_lio.orientation", 1.0);
  declare_parameter("measurement_noise_gains.fast_lio.linear_velocity", 1.0);
  declare_parameter("measurement_noise_gains.fast_lio.angular_velocity", 1.0);
  declare_parameter("measurement_noise_gains.imu.position", 1.0);
  declare_parameter("measurement_noise_gains.imu.orientation", 1.0);
  declare_parameter("measurement_noise_gains.imu.linear_velocity", 1.0);
  declare_parameter("measurement_noise_gains.imu.angular_velocity", 1.0);
  declare_parameter("measurement_noise_gains.gps.position", 1.0);

  declare_parameter("px4_odom_tolerance", 0.1);
  declare_parameter("px4_odom_timeout", 0.5);
  declare_parameter("px4_odom_covariance", 1.0);
  declare_parameter("openvins_odom_tolerance", 0.1);
  declare_parameter("openvins_odom_timeout", 0.5);
  declare_parameter("openvins_odom_covariance", 1.0);
  declare_parameter("fast_lio_odom_tolerance", 0.1);
  declare_parameter("fast_lio_odom_timeout", 0.5);
  declare_parameter("fast_lio_odom_covariance", 1.0);
  declare_parameter("imu_tolerance", 0.1);
  declare_parameter("imu_timeout", 0.5);
  declare_parameter("imu_covariance", 1.0);
  declare_parameter("control_tolerance", 0.1);
  declare_parameter("control_timeout", 0.5);

  RCLCPP_INFO(get_logger(), "EstimationManager node initialized.");
}
//}

/* ~EstimationManager() //{ */
EstimationManager::~EstimationManager() {
}
//}

/* set_verbosity() //{ */
void EstimationManager::set_verbosity(const std::string &verbosity) {
  if (verbosity == "SILENT") {
    get_logger().set_level(rclcpp::Logger::Level::Fatal);
  } else if (verbosity == "ERROR") {
    get_logger().set_level(rclcpp::Logger::Level::Error);
  } else if (verbosity == "WARNING") {
    get_logger().set_level(rclcpp::Logger::Level::Warn);
  } else if (verbosity == "DEBUG") {
    get_logger().set_level(rclcpp::Logger::Level::Debug);
  } else {
    get_logger().set_level(rclcpp::Logger::Level::Info);
  }

  RCLCPP_INFO_STREAM(get_logger(), "Verbosity level set to: " << verbosity);
}
//}

/* on_configure() //{ */
CallbackReturn EstimationManager::on_configure(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Configuring EstimationManager...");

  getParameters();
  configPubSub();
  configTimers();
  configServices();
  setupEKF();

  return CallbackReturn::SUCCESS;
}
//}

/* on_activate() //{ */
CallbackReturn EstimationManager::on_activate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Activating EstimationManager...");
  odom_pub_->on_activate();
  predict_pub_->on_activate();
  diagnostics_pub_->on_activate();

  is_active_ = true;
  timer_->reset();
  diagnostics_timer_->reset();
  ekf_->reset();
  return CallbackReturn::SUCCESS;
}
//}

/* on_deactivate() //{ */
CallbackReturn EstimationManager::on_deactivate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Deactivating EstimationManager...");
  is_active_ = false;
  timer_->cancel();
  diagnostics_timer_->cancel();
  odom_pub_->on_deactivate();
  predict_pub_->on_deactivate();
  diagnostics_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}
//}

/* on_cleanup() //{ */
CallbackReturn EstimationManager::on_cleanup(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Cleaning up EstimationManager...");
  ekf_.reset();
  odom_pub_.reset();
  predict_pub_.reset();
  diagnostics_pub_.reset();
  odometry_px4_sub_.reset();
  odometry_fast_lio_sub_.reset();
  odometry_openvins_sub_.reset();
  imu_sub_.reset();
  control_sub_.reset();
  timer_.reset();
  diagnostics_timer_.reset();

  return CallbackReturn::SUCCESS;
}
//}

/* on_shutdown() //{ */
CallbackReturn EstimationManager::on_shutdown(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Shutting down EstimationManager...");
  return CallbackReturn::SUCCESS;
}
//}

/* getParameters() //{ */
void EstimationManager::getParameters() {
  RCLCPP_INFO(get_logger(), "Loading parameters...");
  get_parameter("frequency", frequency_);
  get_parameter("initial_odometry_source", current_active_odometry_name_);
  get_parameter("odometry_source_names", odometry_source_names_);
  get_parameter("odometry_switch_distance_threshold", odometry_switch_distance_threshold_);
  get_parameter("odometry_switch_angle_threshold", odometry_switch_angle_threshold_);
  get_parameter("odometry_switch_velocity_linear_threshold", odometry_switch_velocity_linear_threshold_);
  get_parameter("odometry_switch_velocity_angular_threshold", odometry_switch_velocity_angular_threshold_);
  get_parameter("sensor_timeout", sensor_timeout_);
  get_parameter("ekf_verbosity", ekf_verbosity_);

  set_verbosity(ekf_verbosity_);

  get_parameter("multirotor_parameters.mass", mass_);
  get_parameter("multirotor_parameters.inertia", inertia_vec_);

  std::vector<double> G1_vec;
  get_parameter("multirotor_parameters.G1", G1_vec);
  int num_cols       = G1_vec.size() / 4;
  allocation_matrix_ = Eigen::Map<const Eigen::MatrixXd>(G1_vec.data(), 4, num_cols);

  get_parameter("process_noise_gains.position", process_noise_gains_.position);
  get_parameter("process_noise_gains.orientation", process_noise_gains_.orientation);
  get_parameter("process_noise_gains.linear_velocity", process_noise_gains_.linear_velocity);
  get_parameter("process_noise_gains.angular_velocity", process_noise_gains_.angular_velocity);
  get_parameter("measurement_noise_gains.px4_odometry.position", measurement_noise_gains_.px4_odometry.position);
  get_parameter("measurement_noise_gains.px4_odometry.orientation", measurement_noise_gains_.px4_odometry.orientation);
  get_parameter("measurement_noise_gains.px4_odometry.linear_velocity", measurement_noise_gains_.px4_odometry.linear_velocity);
  get_parameter("measurement_noise_gains.px4_odometry.angular_velocity", measurement_noise_gains_.px4_odometry.angular_velocity);
  get_parameter("measurement_noise_gains.openvins.position", measurement_noise_gains_.openvins.position);
  get_parameter("measurement_noise_gains.openvins.orientation", measurement_noise_gains_.openvins.orientation);
  get_parameter("measurement_noise_gains.openvins.linear_velocity", measurement_noise_gains_.openvins.linear_velocity);
  get_parameter("measurement_noise_gains.openvins.angular_velocity", measurement_noise_gains_.openvins.angular_velocity);
  get_parameter("measurement_noise_gains.fast_lio.position", measurement_noise_gains_.fast_lio.position);
  get_parameter("measurement_noise_gains.fast_lio.orientation", measurement_noise_gains_.fast_lio.orientation);
  get_parameter("measurement_noise_gains.fast_lio.linear_velocity", measurement_noise_gains_.fast_lio.linear_velocity);
  get_parameter("measurement_noise_gains.fast_lio.angular_velocity", measurement_noise_gains_.fast_lio.angular_velocity);
  get_parameter("measurement_noise_gains.imu.position", measurement_noise_gains_.imu.position);
  get_parameter("measurement_noise_gains.imu.orientation", measurement_noise_gains_.imu.orientation);
  get_parameter("measurement_noise_gains.imu.linear_velocity", measurement_noise_gains_.imu.linear_velocity);
  get_parameter("measurement_noise_gains.imu.angular_velocity", measurement_noise_gains_.imu.angular_velocity);
  get_parameter("measurement_noise_gains.gps.position", measurement_noise_gains_.gps.position);

  double tolerance, timeout;

  get_parameter("px4_odom_tolerance", tolerance);
  get_parameter("px4_odom_timeout", timeout);
  get_parameter("px4_odom_covariance", px4_odom_covariance_);
  px4_odom_data_.tolerance = rclcpp::Duration::from_seconds(tolerance);
  px4_odom_data_.timeout   = rclcpp::Duration::from_seconds(timeout);

  get_parameter("openvins_odom_tolerance", tolerance);
  get_parameter("openvins_odom_timeout", timeout);
  get_parameter("openvins_odom_covariance", openvins_odom_covariance_);
  openvins_odom_data_.tolerance = rclcpp::Duration::from_seconds(tolerance);
  openvins_odom_data_.timeout   = rclcpp::Duration::from_seconds(timeout);

  get_parameter("fast_lio_odom_tolerance", tolerance);
  get_parameter("fast_lio_odom_timeout", timeout);
  get_parameter("fast_lio_odom_covariance", fast_lio_odom_covariance_);
  fast_lio_odom_data_.tolerance = rclcpp::Duration::from_seconds(tolerance);
  fast_lio_odom_data_.timeout   = rclcpp::Duration::from_seconds(timeout);

  get_parameter("imu_tolerance", tolerance);
  get_parameter("imu_timeout", timeout);
  get_parameter("imu_covariance", imu_covariance_);
  imu_data_.tolerance = rclcpp::Duration::from_seconds(tolerance);
  imu_data_.timeout   = rclcpp::Duration::from_seconds(timeout);

  get_parameter("control_tolerance", tolerance);
  get_parameter("control_timeout", timeout);
  control_data_.tolerance = rclcpp::Duration::from_seconds(tolerance);
  control_data_.timeout   = rclcpp::Duration::from_seconds(timeout);

  RCLCPP_INFO(get_logger(), "Parameters loaded.");
}
//}

/* configPubSub() //{ */
void EstimationManager::configPubSub() {
  RCLCPP_INFO(get_logger(), "Configuring publishers and subscribers...");
  odom_pub_        = create_publisher<nav_msgs::msg::Odometry>("odometry_out", 10);
  predict_pub_     = create_publisher<nav_msgs::msg::Odometry>("odometry_predict", 10);
  diagnostics_pub_ = create_publisher<laser_msgs::msg::EstimationManagerDiagnostics>("~/diagnostics", 10);

  odometry_px4_sub_ =
      create_subscription<nav_msgs::msg::Odometry>("odometry_in", 10, std::bind(&EstimationManager::odometryPx4Callback, this, std::placeholders::_1));
  odometry_fast_lio_sub_ = create_subscription<nav_msgs::msg::Odometry>("odometry_fast_lio_in", 10,
                                                                        std::bind(&EstimationManager::odometryFastLioCallback, this, std::placeholders::_1));
  odometry_openvins_sub_ = create_subscription<nav_msgs::msg::Odometry>("odometry_openvins_in", 10,
                                                                        std::bind(&EstimationManager::odometryOpenVinsCallback, this, std::placeholders::_1));
  imu_sub_               = create_subscription<sensor_msgs::msg::Imu>("imu_in", 10, std::bind(&EstimationManager::imuCallback, this, std::placeholders::_1));
  control_sub_           = create_subscription<laser_msgs::msg::UavControlDiagnostics>("control_in", 10,
                                                                             std::bind(&EstimationManager::controlCallback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Publishers and subscribers configured.");
}
//}

/* configTimers() //{ */
void EstimationManager::configTimers() {
  RCLCPP_INFO(get_logger(), "Configuring timers...");
  timer_             = create_wall_timer(std::chrono::duration<double>(1.0 / frequency_), std::bind(&EstimationManager::timerCallback, this));
  diagnostics_timer_ = create_wall_timer(std::chrono::duration<double>(1 / (frequency_ / 10)), std::bind(&EstimationManager::diagnosticsTimerCallback, this));

  RCLCPP_INFO(get_logger(), "Timers configured.");
}
//}

/* configServices() //{ */
void EstimationManager::configServices() {
  RCLCPP_INFO(get_logger(), "Configuring services... ");
  set_odometry_service_ = this->create_service<laser_msgs::srv::SetString>(
      "~/set_odometry", std::bind(&EstimationManager::setOdometryCallback, this, std::placeholders::_1, std::placeholders::_2));
}
//}

/* setupEKF() //{ */
void EstimationManager::setupEKF() {
  RCLCPP_INFO(get_logger(), "Configuring EKF...");
  Eigen::Matrix3d inertia = Eigen::Vector3d(inertia_vec_[0], inertia_vec_[1], inertia_vec_[2]).asDiagonal();

  ekf_ = std::make_unique<laser_uav_estimators::StateEstimator>(mass_, allocation_matrix_, inertia, ekf_verbosity_);


  RCLCPP_INFO(get_logger(), "Applying noise gains to EKF.");
  ekf_->set_process_noise_gains(process_noise_gains_);
  ekf_->set_measurement_noise_gains(measurement_noise_gains_);

  if (current_active_odometry_name_ == "px4_api_odom") {
    enable_px4_odom_      = true;
    enable_openvins_odom_ = false;
    enable_fast_lio_odom_ = false;
  } else if (current_active_odometry_name_ == "openvins_odom") {
    enable_px4_odom_      = false;
    enable_openvins_odom_ = true;
    enable_fast_lio_odom_ = false;
  } else if (current_active_odometry_name_ == "fast_lio_odom") {
    enable_px4_odom_      = false;
    enable_openvins_odom_ = false;
    enable_fast_lio_odom_ = true;
  } else {
    RCLCPP_WARN(get_logger(), "Invalid initial odometry source. Using 'px4_api_odom' as default.");
    current_active_odometry_name_ = "px4_api_odom";
    enable_px4_odom_              = true;
    enable_openvins_odom_         = false;
    enable_fast_lio_odom_         = false;
  }

  RCLCPP_INFO(get_logger(), "EKF configured.");
}
//}

/* odometryPx4Callback() //{ */
void EstimationManager::odometryPx4Callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(px4_odom_data_.mtx);
  px4_odom_data_.buffer[msg->header.stamp] = msg;
  RCLCPP_DEBUG(
      get_logger(), "Received PX4 odometry message at time %.3f s, frequency: %.2f Hz", msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9,
      ((px4_odom_data_.last_msg != nullptr) ? (1.0 / (rclcpp::Time(msg->header.stamp) - rclcpp::Time(px4_odom_data_.last_msg->header.stamp)).seconds()) : 0.0));
  px4_odom_data_.last_msg = msg;
}
//}

/* odometryOpenVinsCallback() //{ */
void EstimationManager::odometryOpenVinsCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(openvins_odom_data_.mtx);
  openvins_odom_data_.buffer[msg->header.stamp] = msg;
  if (enable_openvins_odom_)
    odom_pub_->publish(*msg);
  RCLCPP_DEBUG(get_logger(), "Received OpenVins odometry message at time %.3f s, frequency: %.2f Hz", msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9,
               ((openvins_odom_data_.last_msg != nullptr)
                    ? (1.0 / (rclcpp::Time(msg->header.stamp) - rclcpp::Time(openvins_odom_data_.last_msg->header.stamp)).seconds())
                    : 0.0));
  openvins_odom_data_.last_msg = msg;
}
//}

/* odometryFastLioCallback() //{ */
void EstimationManager::odometryFastLioCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(fast_lio_odom_data_.mtx);
  fast_lio_odom_data_.buffer[msg->header.stamp] = msg;
  RCLCPP_DEBUG(get_logger(), "Received Fast-LIO odometry message at time %.3f s, frequency: %.2f Hz", msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9,
               ((fast_lio_odom_data_.last_msg != nullptr)
                    ? (1.0 / (rclcpp::Time(msg->header.stamp) - rclcpp::Time(fast_lio_odom_data_.last_msg->header.stamp)).seconds())
                    : 0.0));
  fast_lio_odom_data_.last_msg = msg;
}
//}

/* imuCallback() //{ */
void EstimationManager::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(imu_data_.mtx);
  imu_data_.buffer[msg->header.stamp] = msg;
  RCLCPP_DEBUG(get_logger(), "Received IMU message at time %.3f s, frequency: %.2f Hz", msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9,
               ((imu_data_.last_msg != nullptr) ? (1.0 / (rclcpp::Time(msg->header.stamp) - rclcpp::Time(imu_data_.last_msg->header.stamp)).seconds()) : 0.0));
  imu_data_.last_msg = msg;
}
//}

/* controlCallback() //{ */
void EstimationManager::controlCallback(const laser_msgs::msg::UavControlDiagnostics::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(control_data_.mtx);
  control_data_.buffer[msg->header.stamp] = msg;
  RCLCPP_DEBUG(
      get_logger(), "Received control message at time %.3f s, frequency: %.2f Hz", msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9,
      ((control_data_.last_msg != nullptr) ? (1.0 / (rclcpp::Time(msg->header.stamp) - rclcpp::Time(control_data_.last_msg->header.stamp)).seconds()) : 0.0));
  control_data_.last_msg = msg;
}
//}

/* setOdometryCallback() //{ */
void EstimationManager::setOdometryCallback(const std::shared_ptr<laser_msgs::srv::SetString::Request> request,
                                            std::shared_ptr<laser_msgs::srv::SetString::Response>      response) {
  RCLCPP_INFO(get_logger(), "SetOdometry service called with request: %s", request->data.c_str());

  std::lock_guard<std::mutex> lock(mtx_);

  const std::string new_source = request->data;

  if (std::find(odometry_source_names_.begin(), odometry_source_names_.end(), new_source) == odometry_source_names_.end()) {
    response->success = false;
    response->message = "Invalid odometry source: '" + new_source + "'. Valid sources are: ";
    for (const auto &name : odometry_source_names_) {
      response->message += "'" + name + "' ";
    }
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    return;
  }

  if (new_source == current_active_odometry_name_) {
    response->success = true;
    response->message = "Odometry source '" + new_source + "' is already active.";
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
    return;
  }

  SensorDataBuffer<nav_msgs::msg::Odometry> *selected_odom_data = nullptr;
  if (new_source == "openvins_odom")
    selected_odom_data = &openvins_odom_data_;
  else if (new_source == "fast_lio_odom")
    selected_odom_data = &fast_lio_odom_data_;
  else if (new_source == "px4_api_odom")
    selected_odom_data = &px4_odom_data_;

  std::lock_guard<std::mutex> odom_lock(selected_odom_data->mtx);
  if (selected_odom_data->buffer.empty()) {
    response->success = false;
    response->message = "Switch failed. Odometry buffer for '" + new_source + "' is empty.";
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    return;
  }

  auto             newest_msg_it       = selected_odom_data->buffer.rbegin();
  rclcpp::Duration time_since_last_msg = this->get_clock()->now() - newest_msg_it->first;
  if (time_since_last_msg > selected_odom_data->timeout) {
    response->success = false;
    response->message =
        "Switch failed. Timeout on odometry '" + new_source + "'. Last message received " + std::to_string(time_since_last_msg.seconds()) + "s ago.";
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    return;
  }

  const auto &current_state = ekf_->get_state();
  const auto &new_odom_pose = newest_msg_it->second->pose.pose;

  Eigen::Vector3d current_position(current_state(laser_uav_estimators::State::PX), current_state(laser_uav_estimators::State::PY),
                                   current_state(laser_uav_estimators::State::PZ));
  Eigen::Vector3d new_odom_position(new_odom_pose.position.x, new_odom_pose.position.y, new_odom_pose.position.z);
  double          distance = (current_position - new_odom_position).norm();

  Eigen::Quaterniond new_orientation(new_odom_pose.orientation.w, new_odom_pose.orientation.x, new_odom_pose.orientation.y, new_odom_pose.orientation.z);
  Eigen::Quaterniond current_orientation(current_state(laser_uav_estimators::State::QW), current_state(laser_uav_estimators::State::QX),
                                         current_state(laser_uav_estimators::State::QY), current_state(laser_uav_estimators::State::QZ));
  Eigen::AngleAxisd  angle_axis_diff(new_orientation * current_orientation.inverse());

  Eigen::Vector3d new_odom_linear_velocity(newest_msg_it->second->twist.twist.linear.x, newest_msg_it->second->twist.twist.linear.y,
                                           newest_msg_it->second->twist.twist.linear.z);
  Eigen::Vector3d current_linear_velocity(current_state(laser_uav_estimators::State::VX), current_state(laser_uav_estimators::State::VY),
                                          current_state(laser_uav_estimators::State::VZ));
  double          velocity_diff = (current_linear_velocity - new_odom_linear_velocity).norm();

  Eigen::Vector3d new_odom_angular_velocity(newest_msg_it->second->twist.twist.angular.x, newest_msg_it->second->twist.twist.angular.y,
                                            newest_msg_it->second->twist.twist.angular.z);
  Eigen::Vector3d current_angular_velocity(current_state(laser_uav_estimators::State::WX), current_state(laser_uav_estimators::State::WY),
                                           current_state(laser_uav_estimators::State::WZ));
  double          angular_velocity_diff = (current_angular_velocity - new_odom_angular_velocity).norm();

  if ((distance > odometry_switch_distance_threshold_) || (angle_axis_diff.angle() > odometry_switch_angle_threshold_) ||
      (velocity_diff > odometry_switch_velocity_linear_threshold_) || (angular_velocity_diff > odometry_switch_velocity_angular_threshold_)) {
    response->success = false;
    response->message = "Switch failed. Odometry '" + new_source + "' is too far from the current estimate. Diffs - Pos: " + std::to_string(distance) +
                        " m, Angle: " + std::to_string(angle_axis_diff.angle()) + " rad, LinVel: " + std::to_string(velocity_diff) +
                        " m/s, AngVel: " + std::to_string(angular_velocity_diff) + " rad/s.";
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    return;
  }

  if (!selected_odom_data->is_active) {
    response->success = false;
    response->message = "Switch failed. Odometry '" + new_source + "' is not active.";
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    return;
  }

  enable_px4_odom_              = (new_source == "px4_api_odom");
  enable_openvins_odom_         = (new_source == "openvins_odom");
  enable_fast_lio_odom_         = (new_source == "fast_lio_odom");
  current_active_odometry_name_ = new_source;

  response->success = true;
  response->message = "Odometry source switched to: " + current_active_odometry_name_;
  RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
}
//}

/* getSynchronizedMessage() //{ */
template <typename MsgT>
std::optional<MsgT> EstimationManager::getSynchronizedMessage(const rclcpp::Time &ref_time, SensorDataBuffer<MsgT> &sensor_data, std::string sensor_name) {
  std::lock_guard<std::mutex> lock(sensor_data.mtx);

  if (sensor_data.buffer.empty()) {
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000, "[%s]: Message buffer empty.", sensor_name.c_str());
    return std::nullopt;
  }

  auto newest_msg_it = sensor_data.buffer.rbegin();
  if ((ref_time - rclcpp::Time(sensor_data.last_msg->header.stamp)) > sensor_data.timeout) {
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000, "[%s]: Timeout detected. Last msg is %.2f s old. Timeout is %.2f s.", sensor_name.c_str(),
                          (ref_time - rclcpp::Time(sensor_data.last_msg->header.stamp)).seconds(), sensor_data.timeout.seconds());
    return std::nullopt;
  }

  typename std::map<rclcpp::Time, typename MsgT::SharedPtr>::iterator best_match_it = sensor_data.buffer.end();
  rclcpp::Duration                                                    min_diff      = rclcpp::Duration::max();

  for (auto it = sensor_data.buffer.begin(); it != sensor_data.buffer.end(); ++it) {
    rclcpp::Duration diff = ref_time - it->first;
    if (std::abs(diff.seconds()) < min_diff.seconds()) {
      min_diff      = rclcpp::Duration::from_seconds(std::abs(diff.seconds()));
      best_match_it = it;
    }
  }

  if (best_match_it == sensor_data.buffer.end())
    return std::nullopt;

  if (min_diff <= sensor_data.tolerance) {
    std::optional<MsgT> msg_copy = *best_match_it->second;
    sensor_data.is_active        = true;
    sensor_data.buffer.erase(best_match_it);
    RCLCPP_DEBUG(get_logger(), "[%s]: ACCEPTED: Best match (%.2f ms) within tolerance (%.2f ms).", sensor_name.c_str(), min_diff.seconds() * 1000.0,
                 sensor_data.tolerance.seconds() * 1000.0);
    return msg_copy;
  }

  RCLCPP_DEBUG(get_logger(), "[%s]: REJECTED: Best match (%.2f ms) is outside tolerance (%.2f ms).", sensor_name.c_str(), min_diff.seconds() * 1000.0,
               sensor_data.tolerance.seconds() * 1000.0);

  return std::nullopt;
}
//}

/* pruneSensorBuffer() //{ */
template <typename MsgT>
void EstimationManager::pruneSensorBuffer(const rclcpp::Time &now, SensorDataBuffer<MsgT> &sensor_data, std::string sensor_name) {
  std::lock_guard<std::mutex> lock(sensor_data.mtx);
  if (sensor_data.buffer.empty())
    return;

  const rclcpp::Time cutoff_time = now - (sensor_data.timeout * 2.0);

  auto first_to_keep_it = sensor_data.buffer.upper_bound(cutoff_time);

  RCLCPP_DEBUG(
      get_logger(), "[%s] Pruning sensor buffer. %s, first kept time: %.2f s", sensor_name.c_str(),
      (first_to_keep_it != sensor_data.buffer.begin() ? "Removing old messages." : "No messages to remove."),
      (first_to_keep_it != sensor_data.buffer.begin() && first_to_keep_it != sensor_data.buffer.end()) ? rclcpp::Time(first_to_keep_it->first).seconds() : 0.0);
  sensor_data.buffer.erase(sensor_data.buffer.begin(), first_to_keep_it);
}
//}

/* timerCallback() //{ */
void EstimationManager::timerCallback() {
  try {
    if (!is_active_)
      return;
    if (!is_initialized_) {
      RCLCPP_INFO(get_logger(), "Initializing EKF...");
      is_initialized_ = true;
      return;
    }

    rclcpp::Time reference_time = this->get_clock()->now();

    auto px4_odom_msg      = getSynchronizedMessage(reference_time, px4_odom_data_, "PX4_ODOMETRY");
    auto openvins_odom_msg = getSynchronizedMessage(reference_time, openvins_odom_data_, "OPENVINS_ODOMETRY");
    auto fast_lio_odom_msg = getSynchronizedMessage(reference_time, fast_lio_odom_data_, "FAST_LIO_ODOMETRY");
    auto imu_msg           = getSynchronizedMessage(reference_time, imu_data_, "IMU");
    auto control_msg       = getSynchronizedMessage(reference_time, control_data_, "CONTROL");

    RCLCPP_DEBUG(get_logger(), "Synchronized Messages - PX4 Odom: %s, OpenVINS Odom: %s, FastLIO Odom: %s, IMU: %s, Control: %s", px4_odom_msg ? "YES" : "NO",
                 openvins_odom_msg ? "YES" : "NO", fast_lio_odom_msg ? "YES" : "NO", imu_msg ? "YES" : "NO", control_msg ? "YES" : "NO");
    RCLCPP_DEBUG(get_logger(), "Buffer Sizes - PX4 Odom: %zu, OpenVINS Odom: %zu, FastLIO Odom: %zu, IMU: %zu, Control: %zu", px4_odom_data_.buffer.size(),
                 openvins_odom_data_.buffer.size(), fast_lio_odom_data_.buffer.size(), imu_data_.buffer.size(), control_data_.buffer.size());

    pruneSensorBuffer(reference_time, px4_odom_data_, "PX4_ODOMETRY");
    pruneSensorBuffer(reference_time, openvins_odom_data_, "OPENVINS_ODOMETRY");
    pruneSensorBuffer(reference_time, fast_lio_odom_data_, "FAST_LIO_ODOMETRY");
    pruneSensorBuffer(reference_time, imu_data_, "IMU");
    pruneSensorBuffer(reference_time, control_data_, "CONTROL");

    bool         has_prediction{false};
    const double MAX_CONTROL_VALUE = 1.0e2;

    if (control_msg) {
      if (!is_first_control_msg) {
        last_control_input_time_ = rclcpp::Time(control_msg->header.stamp);
        is_first_control_msg     = true;
      } else {
        rclcpp::Time current_time = rclcpp::Time(control_msg->header.stamp);
        double       dt_sec       = (current_time - last_control_input_time_).seconds();
        last_control_input_time_  = current_time;

        bool can_predict = true;

        if (dt_sec < 0 || dt_sec > 1.0) {
          can_predict = false;
        }

        if (can_predict && control_msg->last_control_input.data.size() != 4) {
          can_predict = false;
        }

        if (can_predict) {
          Eigen::Map<const Eigen::Vector4d> control_input(control_msg->last_control_input.data.data());

          if (!control_input.allFinite()) {
            RCLCPP_ERROR(get_logger(), "Control input contains non-finite values (inf or NaN).");
            can_predict = false;
          } else if ((control_input.array() < 0).any()) {
            RCLCPP_ERROR(get_logger(), "Control input contains negative values. Inputs: [%.2f, %.2f, %.2f, %.2f]", control_input[0], control_input[1],
                         control_input[2], control_input[3]);
            can_predict = false;
          } else if ((control_input.array() > MAX_CONTROL_VALUE).any()) {
            RCLCPP_ERROR(get_logger(), "Control input contains excessively large values. Inputs: [%.2f, %.2f, %.2f, %.2f]", control_input[0], control_input[1],
                         control_input[2], control_input[3]);
            can_predict = false;
          }

          if (can_predict) {
            ekf_->predict(control_input, dt_sec);
            rclcpp::Time stamp = rclcpp::Time(control_msg->header.stamp);
            publishOdometry(predict_pub_, stamp);
            has_prediction = true;
          }
        }
      }
    }

    laser_uav_estimators::MeasurementPackage pkg;
    bool                                     has_measurement{false};
    if (px4_odom_msg && enable_px4_odom_) {
      const auto                                   &px4_odom = *px4_odom_msg;
      Eigen::Map<const Eigen::Matrix<double, 6, 6>> px4_pose_cov(px4_odom.pose.covariance.data());
      Eigen::Map<const Eigen::Matrix<double, 6, 6>> px4_twist_cov(px4_odom.twist.covariance.data());

      if (px4_odom_covariance_ > px4_pose_cov.trace() && px4_odom_covariance_ > px4_twist_cov.trace()) {
        pkg.px4_odometry  = *px4_odom_msg;
        last_update_time_ = px4_odom_msg->header.stamp;
        has_measurement   = true;
      }
    } else if (openvins_odom_msg && enable_openvins_odom_) {
      const auto                                   &openvins_odom = *openvins_odom_msg;
      Eigen::Map<const Eigen::Matrix<double, 6, 6>> openvins_pose_cov(openvins_odom.pose.covariance.data());
      Eigen::Map<const Eigen::Matrix<double, 6, 6>> openvins_twist_cov(openvins_odom.twist.covariance.data());

      if (openvins_odom_covariance_ > openvins_pose_cov.trace() && openvins_odom_covariance_ > openvins_twist_cov.trace()) {
        pkg.openvins      = *openvins_odom_msg;
        last_update_time_ = openvins_odom_msg->header.stamp;
        has_measurement   = true;
      }
      // } else if (fast_lio_odom_msg && enable_fast_lio_odom_) {
    } else if ((px4_odom_msg || fast_lio_odom_msg) && enable_fast_lio_odom_) {
      if (px4_odom_msg) {
        const auto                                   &px4_odom = *px4_odom_msg;
        Eigen::Map<const Eigen::Matrix<double, 6, 6>> px4_pose_cov(px4_odom.pose.covariance.data());
        Eigen::Map<const Eigen::Matrix<double, 6, 6>> px4_twist_cov(px4_odom.twist.covariance.data());

        px4_odom_msg->pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
        px4_odom_msg->pose.pose.position.y = std::numeric_limits<double>::quiet_NaN();
        px4_odom_msg->pose.pose.position.z = std::numeric_limits<double>::quiet_NaN();

        px4_odom_msg->pose.pose.orientation.w = std::numeric_limits<double>::quiet_NaN();
        px4_odom_msg->pose.pose.orientation.x = std::numeric_limits<double>::quiet_NaN();
        px4_odom_msg->pose.pose.orientation.y = std::numeric_limits<double>::quiet_NaN();
        px4_odom_msg->pose.pose.orientation.z = std::numeric_limits<double>::quiet_NaN();

        if (px4_odom_covariance_ > px4_pose_cov.trace() && px4_odom_covariance_ > px4_twist_cov.trace()) {
          pkg.px4_odometry = *px4_odom_msg;
          has_measurement  = true;
        }
      }
      if (fast_lio_odom_msg) {
        const auto                                   &fast_lio_odom = *fast_lio_odom_msg;
        Eigen::Map<const Eigen::Matrix<double, 6, 6>> fast_lio_pose_cov(fast_lio_odom.pose.covariance.data());
        Eigen::Map<const Eigen::Matrix<double, 6, 6>> fast_lio_twist_cov(fast_lio_odom.twist.covariance.data());
        if (fast_lio_odom_covariance_ > fast_lio_pose_cov.trace() && fast_lio_odom_covariance_ > fast_lio_twist_cov.trace()) {
          pkg.fast_lio      = *fast_lio_odom_msg;
          last_update_time_ = fast_lio_odom_msg->header.stamp;
          has_measurement   = true;
        }
      }
    }

    bool has_measurement_imu{false};
    if (imu_msg) {
      if (last_imu_msg_) {
        pkg.dt              = (rclcpp::Time(imu_msg->header.stamp) - rclcpp::Time(last_imu_msg_->header.stamp)).seconds();
        pkg.imu             = *imu_msg;
        has_measurement_imu = true;
      }
      last_imu_msg_ = std::make_shared<sensor_msgs::msg::Imu>(*imu_msg);
    }

    if (has_measurement || has_measurement_imu)
      ekf_->correct(pkg);

    if ((has_prediction || has_measurement) && !enable_openvins_odom_) {
      publishOdometry(odom_pub_, last_update_time_);
    } else if (enable_openvins_odom_ && !openvins_odom_data_.last_msg) {
      auto msg          = std::make_shared<nav_msgs::msg::Odometry>();
      msg->header.stamp = this->get_clock()->now();

      if (px4_odom_data_.last_msg) {
        msg = px4_odom_data_.last_msg;
      }
      odom_pub_->publish(*msg);
    }
  }
  catch (const std::exception &e) {
    RCLCPP_ERROR(get_logger(), "Error in timerCallback: %s", e.what());
  }
}
//}

/* diagnosticsTimerCallback() //{ */
void EstimationManager::diagnosticsTimerCallback() {
  if (!is_active_)
    return;
  try {
    auto diag_msg          = std::make_unique<laser_msgs::msg::EstimationManagerDiagnostics>();
    diag_msg->header.stamp = this->get_clock()->now();

    if (current_active_odometry_name_ == "px4_api_odom" && px4_odom_data_.is_active && !px4_odom_data_.buffer.empty())
      diag_msg->header.frame_id = px4_odom_data_.buffer.begin()->second->header.frame_id;
    else if (current_active_odometry_name_ == "openvins_odom" && openvins_odom_data_.is_active && !openvins_odom_data_.buffer.empty())
      diag_msg->header.frame_id = openvins_odom_data_.buffer.begin()->second->header.frame_id;
    else if (current_active_odometry_name_ == "fast_lio_odom" && fast_lio_odom_data_.is_active && !fast_lio_odom_data_.buffer.empty())
      diag_msg->header.frame_id = fast_lio_odom_data_.buffer.begin()->second->header.frame_id;

    std::lock_guard<std::mutex> lock(mtx_);

    diag_msg->active_odometry_source = current_active_odometry_name_;
    diag_msg->is_initialized         = is_initialized_;

    auto fill_sensor_status = [&](laser_msgs::msg::SensorStatus &status, auto &sensor_data, const std::string &name) {
      std::lock_guard<std::mutex> lock(sensor_data.mtx);
      status.name        = name;
      status.is_active   = sensor_data.is_active;
      status.buffer_size = sensor_data.buffer.size();

      if (sensor_data.last_msg) {
        status.time_since_last_message = (this->get_clock()->now() - rclcpp::Time(sensor_data.last_msg->header.stamp)).seconds();
        status.has_timeout             = (this->get_clock()->now() - rclcpp::Time(sensor_data.last_msg->header.stamp)) > sensor_data.timeout;
        status.last_message_stamp      = sensor_data.last_msg->header.stamp;
      } else {
        status.has_timeout             = true;
        status.time_since_last_message = -1.0;
      }

      if (name == current_active_odometry_name_) {

        if (!sensor_data.last_msg) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Active odometry sensor ('%s') has not started yet (no message received).",
                               name.c_str());
        } else {

          if ((this->get_clock()->now() - rclcpp::Time(sensor_data.last_msg->header.stamp)) > sensor_data.timeout) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                  "Timeout! Active odometry sensor ('%s') stopped publishing. (Last msg: %.2f s ago)", name.c_str(),
                                  (this->get_clock()->now() - rclcpp::Time(sensor_data.last_msg->header.stamp)).seconds());
          } else {
            RCLCPP_INFO_ONCE(this->get_logger(), "Active odometry sensor ('%s') publishing. (Last msg: %.2f s ago)", name.c_str(),
                             (this->get_clock()->now() - rclcpp::Time(sensor_data.last_msg->header.stamp)).seconds());
          }
        }
      }
    };

    fill_sensor_status(diag_msg->odometry_sources.emplace_back(), px4_odom_data_, "px4_api_odom");
    fill_sensor_status(diag_msg->odometry_sources.emplace_back(), openvins_odom_data_, "openvins_odom");
    fill_sensor_status(diag_msg->odometry_sources.emplace_back(), fast_lio_odom_data_, "fast_lio_odom");
    fill_sensor_status(diag_msg->imu_status, imu_data_, "imu");

    diagnostics_pub_->publish(std::move(diag_msg));
  }
  catch (const std::exception &e) {
    RCLCPP_ERROR(get_logger(), "Error publishing diagnostics: %s", e.what());
  }
}
//}

/* publishOdometry() //{ */
void EstimationManager::publishOdometry(rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr pub, rclcpp::Time &pub_time) {
  const auto &state = ekf_->get_state();
  const auto &cov   = ekf_->get_covariance();

  if (state.size() < 13 || cov.rows() < 13 || cov.cols() < 13) {
    RCLCPP_ERROR(get_logger(), "EKF state or covariance has incorrect size.");
    return;
  }
  if (state.hasNaN() || cov.hasNaN()) {
    RCLCPP_ERROR(get_logger(), "State or covariance contains NaN.");
    rclcpp::shutdown();
    return;
  }

  nav_msgs::msg::Odometry odom_out_msg;
  odom_out_msg.header.stamp = pub_time;
  if (current_active_odometry_name_ == "px4_api_odom" && px4_odom_data_.is_active && !px4_odom_data_.buffer.empty())
    odom_out_msg.header.frame_id = px4_odom_data_.buffer.begin()->second->header.frame_id;
  else if (current_active_odometry_name_ == "openvins_odom" && openvins_odom_data_.is_active && !openvins_odom_data_.buffer.empty())
    odom_out_msg.header.frame_id = openvins_odom_data_.buffer.begin()->second->header.frame_id;
  else if (current_active_odometry_name_ == "fast_lio_odom" && fast_lio_odom_data_.is_active && !fast_lio_odom_data_.buffer.empty())
    odom_out_msg.header.frame_id = fast_lio_odom_data_.buffer.begin()->second->header.frame_id;

  odom_out_msg.pose.pose.position.x    = state(laser_uav_estimators::State::PX);
  odom_out_msg.pose.pose.position.y    = state(laser_uav_estimators::State::PY);
  odom_out_msg.pose.pose.position.z    = state(laser_uav_estimators::State::PZ);
  odom_out_msg.pose.pose.orientation.w = state(laser_uav_estimators::State::QW);
  odom_out_msg.pose.pose.orientation.x = state(laser_uav_estimators::State::QX);
  odom_out_msg.pose.pose.orientation.y = state(laser_uav_estimators::State::QY);
  odom_out_msg.pose.pose.orientation.z = state(laser_uav_estimators::State::QZ);

  odom_out_msg.twist.twist.linear.x  = state(laser_uav_estimators::State::VX);
  odom_out_msg.twist.twist.linear.y  = state(laser_uav_estimators::State::VY);
  odom_out_msg.twist.twist.linear.z  = state(laser_uav_estimators::State::VZ);
  odom_out_msg.twist.twist.angular.x = state(laser_uav_estimators::State::WX);
  odom_out_msg.twist.twist.angular.y = state(laser_uav_estimators::State::WY);
  odom_out_msg.twist.twist.angular.z = state(laser_uav_estimators::State::WZ);

  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      odom_out_msg.pose.covariance[i * 6 + j]  = cov(i, j);
      odom_out_msg.twist.covariance[i * 6 + j] = cov(i + 7, j + 7);
    }
  }

  pub->publish(odom_out_msg);
}
//}

}  // namespace laser_uav_managers

RCLCPP_COMPONENTS_REGISTER_NODE(laser_uav_managers::EstimationManager)
