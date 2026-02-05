#include <laser_uav_managers/mekf_estimation_manager_node.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace laser_uav_managers
{
/* MekfEstimationManager() //{ */
MekfEstimationManager::MekfEstimationManager(const rclcpp::NodeOptions &options) : rclcpp_lifecycle::LifecycleNode("state_estimator", options) {
  RCLCPP_INFO(get_logger(), "Creating MekfEstimationManager...");

  declare_parameter("frequency", 100.0);
  declare_parameter("odometry_switch_distance_threshold", 0.25);
  declare_parameter("odometry_switch_angle_threshold", 0.5);
  declare_parameter("odometry_switch_velocity_linear_threshold", 0.5);
  declare_parameter("odometry_switch_velocity_angular_threshold", 1.0);
  declare_parameter("sensor_timeout", 0.5);
  declare_parameter("ekf_verbosity", "INFO");

  declare_parameter("multirotor_parameters.mass", 1.0);
  declare_parameter("multirotor_parameters.inertia", std::vector<double>{0.01, 0.01, 0.01});
  declare_parameter("multirotor_parameters.c_thrust", 1.0);
  declare_parameter("multirotor_parameters.G1", std::vector<double>{0.1, -0.1, -0.1, 0.1, 0.1, 0.1, -0.1, -0.1});

  declare_parameter("process_noise_gains.position", 0.01);
  declare_parameter("process_noise_gains.orientation", 0.01);
  declare_parameter("process_noise_gains.linear_velocity", 0.1);
  declare_parameter("process_noise_gains.angular_velocity", 0.1);

  declare_parameter("measurement_noise_gains.position", 1.0);
  declare_parameter("measurement_noise_gains.orientation", 1.0);
  declare_parameter("measurement_noise_gains.linear_velocity", 1.0);
  declare_parameter("measurement_noise_gains.angular_velocity", 1.0);

  declare_parameter("odom_tolerance", 0.1);
  declare_parameter("odom_timeout", 0.5);
  declare_parameter("odom_covariance", 1.0);
  declare_parameter("control_tolerance", 0.1);
  declare_parameter("control_timeout", 0.5);

  RCLCPP_INFO(get_logger(), "MekfEstimationManager node initialized.");
}
//}

/* ~MekfEstimationManager() //{ */
MekfEstimationManager::~MekfEstimationManager() {
}
//}

/* set_verbosity() //{ */
void MekfEstimationManager::set_verbosity(const std::string &verbosity) {
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
CallbackReturn MekfEstimationManager::on_configure(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Configuring MekfEstimationManager...");

  getParameters();
  configPubSub();
  configTimers();
  configServices();
  setupEKF();

  return CallbackReturn::SUCCESS;
}
//}

/* on_activate() //{ */
CallbackReturn MekfEstimationManager::on_activate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Activating MekfEstimationManager...");
  odom_pub_->on_activate();
  predict_pub_->on_activate();
  diagnostics_pub_->on_activate();

  is_active_ = true;
  timer_->reset();
  diagnostics_timer_->reset();
  return CallbackReturn::SUCCESS;
}
//}

/* on_deactivate() //{ */
CallbackReturn MekfEstimationManager::on_deactivate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Deactivating MekfEstimationManager...");
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
CallbackReturn MekfEstimationManager::on_cleanup(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Cleaning up MekfEstimationManager...");
  mekf_.reset();
  odom_pub_.reset();
  predict_pub_.reset();
  diagnostics_pub_.reset();
  odometry_px4_sub_.reset();
  motor_sub_.reset();
  control_sub_.reset();
  timer_.reset();
  diagnostics_timer_.reset();

  return CallbackReturn::SUCCESS;
}
//}

/* on_shutdown() //{ */
CallbackReturn MekfEstimationManager::on_shutdown(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Shutting down MekfEstimationManager...");
  return CallbackReturn::SUCCESS;
}
//}

/* getParameters() //{ */
void MekfEstimationManager::getParameters() {
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
  get_parameter("estimation_verbosity", estimation_verbosity_);

  set_verbosity(estimation_verbosity_);

  get_parameter("multirotor_parameters.mass", mass_);
  get_parameter("multirotor_parameters.c_thrust", thrust_coefficient_);
  get_parameter("multirotor_parameters.inertia", inertia_vec_);

  std::vector<double> G1_vec;
  get_parameter("multirotor_parameters.G1", G1_vec);
  int num_cols       = G1_vec.size() / 4;
  allocation_matrix_ = Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(G1_vec.data(), 4, num_cols);

  get_parameter("process_noise_gains.position", process_noise_gains_.position);
  get_parameter("process_noise_gains.orientation", process_noise_gains_.orientation);
  get_parameter("process_noise_gains.linear_velocity", process_noise_gains_.velocity_linear);
  get_parameter("process_noise_gains.angular_velocity", process_noise_gains_.velocity_angular);
  get_parameter("measurement_noise_gains.position", measurement_noise_gains_.odometry.position);
  get_parameter("measurement_noise_gains.orientation", measurement_noise_gains_.odometry.orientation);
  get_parameter("measurement_noise_gains.linear_velocity", measurement_noise_gains_.odometry.velocity_linear);
  get_parameter("measurement_noise_gains.angular_velocity", measurement_noise_gains_.odometry.velocity_angular);
  get_parameter("measurement_noise_gains.imu.position", measurement_noise_gains_.imu.position);
  get_parameter("measurement_noise_gains.imu.orientation", measurement_noise_gains_.imu.orientation);
  get_parameter("measurement_noise_gains.imu.linear_velocity", measurement_noise_gains_.imu.velocity_linear);
  get_parameter("measurement_noise_gains.imu.angular_velocity", measurement_noise_gains_.imu.velocity_angular);

  double tolerance, timeout;

  get_parameter("odom_tolerance", tolerance);
  get_parameter("odom_timeout", timeout);
  get_parameter("odom_covariance", px4_odom_covariance_);
  odom_data_.tolerance = rclcpp::Duration::from_seconds(tolerance);
  odom_data_.timeout   = rclcpp::Duration::from_seconds(timeout);

  get_parameter("control_tolerance", tolerance);
  get_parameter("control_timeout", timeout);
  control_data_.tolerance = rclcpp::Duration::from_seconds(tolerance);
  control_data_.timeout   = rclcpp::Duration::from_seconds(timeout);

  RCLCPP_INFO(get_logger(), "Parameters loaded.");
}
//}

/* configPubSub() //{ */
void MekfEstimationManager::configPubSub() {
  RCLCPP_INFO(get_logger(), "Configuring publishers and subscribers...");
  odom_pub_        = create_publisher<nav_msgs::msg::Odometry>("odometry_out", 10);
  predict_pub_     = create_publisher<nav_msgs::msg::Odometry>("odometry_predict", 10);
  diagnostics_pub_ = create_publisher<laser_msgs::msg::EstimationManagerDiagnostics>("~/diagnostics", 10);

  odometry_px4_sub_ =
      create_subscription<nav_msgs::msg::Odometry>("odometry_in", 10, std::bind(&MekfEstimationManager::odometryPx4Callback, this, std::placeholders::_1));
  control_sub_ = create_subscription<laser_msgs::msg::UavControlDiagnostics>("control_in", 10,
                                                                             std::bind(&MekfEstimationManager::controlCallback, this, std::placeholders::_1));
  motor_sub_   = create_subscription<laser_msgs::msg::MotorSpeedStamped>("motor_speed_in", 10,
                                                                       std::bind(&MekfEstimationManager::motorSpeedCallback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Publishers and subscribers configured.");
}
//}

/* configTimers() //{ */
void MekfEstimationManager::configTimers() {
  RCLCPP_INFO(get_logger(), "Configuring timers...");
  timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / frequency_), std::bind(&MekfEstimationManager::timerCallback, this));
  diagnostics_timer_ =
      create_wall_timer(std::chrono::duration<double>(1 / (frequency_ / 10)), std::bind(&MekfEstimationManager::diagnosticsTimerCallback, this));

  RCLCPP_INFO(get_logger(), "Timers configured.");
}
//}

/* configServices() //{ */
void MekfEstimationManager::configServices() {
  RCLCPP_INFO(get_logger(), "Configuring services... ");
  set_odometry_service_ = this->create_service<laser_msgs::srv::SetString>(
      "~/set_odometry", std::bind(&MekfEstimationManager::setOdometryCallback, this, std::placeholders::_1, std::placeholders::_2));
}
//}

/* setupEKF() //{ */
void MekfEstimationManager::setupEKF() {
  RCLCPP_INFO(get_logger(), "Configuring EKF...");
  Eigen::Matrix3d inertia = Eigen::Vector3d(inertia_vec_[0], inertia_vec_[1], inertia_vec_[2]).asDiagonal();

  mekf_ =
      std::make_unique<laser_uav_estimators::MEKFEstimator>(mass_, allocation_matrix_, inertia, measurement_noise_gains_, process_noise_gains_, ekf_verbosity_);

  RCLCPP_INFO(get_logger(), "Applying noise gains to EKF.");

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
void MekfEstimationManager::odometryPx4Callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(odom_data_.mtx);
  odom_data_.buffer[msg->header.stamp] = msg;
  RCLCPP_DEBUG(
      get_logger(), "Received PX4 odometry message at time %.3f s, frequency: %.2f Hz", msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9,
      ((odom_data_.last_msg != nullptr) ? (1.0 / (rclcpp::Time(msg->header.stamp) - rclcpp::Time(odom_data_.last_msg->header.stamp)).seconds()) : 0.0));
  odom_data_.last_msg = msg;
}
//}

/* controlCallback() //{ */
void MekfEstimationManager::controlCallback(const laser_msgs::msg::UavControlDiagnostics::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(control_data_.mtx);
  control_data_.buffer[msg->header.stamp] = msg;
  RCLCPP_DEBUG(
      get_logger(), "Received control message at time %.3f s, frequency: %.2f Hz", msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9,
      ((control_data_.last_msg != nullptr) ? (1.0 / (rclcpp::Time(msg->header.stamp) - rclcpp::Time(control_data_.last_msg->header.stamp)).seconds()) : 0.0));
  control_data_.last_msg = msg;
}
//}

/* motorSpeedCallback() //{ */
void MekfEstimationManager::motorSpeedCallback(const laser_msgs::msg::MotorSpeedStamped::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(motor_speed_data_.mtx);
  motor_speed_data_.buffer[msg->header.stamp] = msg;
  // RCLCPP_DEBUG(
  //     get_logger(), "Received motor speed message at time %.3f s, frequency: %.2f Hz", msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9,
  //     ((motor_speed_data_.last_msg != nullptr) ? (1.0 / (rclcpp::Time(msg->header.stamp) - rclcpp::Time(motor_speed_data_.last_msg->header.stamp)).seconds())
  //                                              : 0.0));

  // std::cout << "Motor speeds (RPM): ";
  // for (const auto &speed : msg->data.data) {
  //   std::cout << speed << " ";
  // }
  // std::cout << std::endl;

  for (size_t i = 0; i < msg->data.data.size(); i++) {
    msg->data.data[i] = (msg->data.data[i] * msg->data.data[i]) * thrust_coefficient_;
  }
  // std::cout << "Coeficient Thrust: " << thrust_coefficient_ << std::endl;

  // std::cout << "Motor thrusts (N): ";
  // for (const auto &thrust : msg->data.data) {
  //   std::cout << thrust << " ";
  // }
  // std::cout << std::endl;

  motor_speed_data_.last_msg = msg;
}
//}


/* setOdometryCallback() //{ */
void MekfEstimationManager::setOdometryCallback(const std::shared_ptr<laser_msgs::srv::SetString::Request> request,
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
    selected_odom_data = &odom_data_;

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

  const auto &current_state = mekf_->get_odometry();
  const auto &new_odom_pose = newest_msg_it->second->pose.pose;

  Eigen::Vector3d current_position(current_state.pose.pose.position.x, current_state.pose.pose.position.y, current_state.pose.pose.position.z);
  Eigen::Vector3d new_odom_position(new_odom_pose.position.x, new_odom_pose.position.y, new_odom_pose.position.z);
  double          distance = (current_position - new_odom_position).norm();

  Eigen::Quaterniond new_orientation(new_odom_pose.orientation.w, new_odom_pose.orientation.x, new_odom_pose.orientation.y, new_odom_pose.orientation.z);
  Eigen::Quaterniond current_orientation(current_state.pose.pose.orientation.w, current_state.pose.pose.orientation.x, current_state.pose.pose.orientation.y,
                                         current_state.pose.pose.orientation.z);
  Eigen::AngleAxisd  angle_axis_diff(new_orientation * current_orientation.inverse());

  Eigen::Vector3d new_odom_linear_velocity(newest_msg_it->second->twist.twist.linear.x, newest_msg_it->second->twist.twist.linear.y,
                                           newest_msg_it->second->twist.twist.linear.z);
  Eigen::Vector3d current_linear_velocity(current_state.twist.twist.linear.x, current_state.twist.twist.linear.y, current_state.twist.twist.linear.z);
  double          velocity_diff = (current_linear_velocity - new_odom_linear_velocity).norm();

  Eigen::Vector3d new_odom_angular_velocity(newest_msg_it->second->twist.twist.angular.x, newest_msg_it->second->twist.twist.angular.y,
                                            newest_msg_it->second->twist.twist.angular.z);
  Eigen::Vector3d current_angular_velocity(current_state.twist.twist.angular.x, current_state.twist.twist.angular.y, current_state.twist.twist.angular.z);
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
std::optional<MsgT> MekfEstimationManager::getSynchronizedMessage(const rclcpp::Time &ref_time, SensorDataBuffer<MsgT> &sensor_data, std::string sensor_name) {
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
void MekfEstimationManager::pruneSensorBuffer(const rclcpp::Time &now, SensorDataBuffer<MsgT> &sensor_data, std::string sensor_name) {
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
void MekfEstimationManager::timerCallback() {
  try {
    if (!is_active_)
      return;
    if (!is_initialized_) {
      RCLCPP_INFO(get_logger(), "Initializing EKF...");
      is_initialized_ = true;
      return;
    }

    rclcpp::Time reference_time = this->get_clock()->now();

    auto px4_odom_msg = getSynchronizedMessage(reference_time, odom_data_, "PX4_ODOMETRY");
    auto control_msg  = getSynchronizedMessage(reference_time, control_data_, "CONTROL");

    pruneSensorBuffer(reference_time, odom_data_, "PX4_ODOMETRY");
    pruneSensorBuffer(reference_time, control_data_, "CONTROL");

    bool         has_prediction{false};
    const double MAX_CONTROL_VALUE = 1.0e2;

    if (enable_px4_odom_ && !odom_data_.is_active && !imu_data_.is_active) {
      if (!odom_data_.is_active)
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "PX4 odometry input is inactive.");
      if (!imu_data_.is_active)
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "IMU input is inactive.");
      if (!is_ekf_active_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "EKF is active but no valid measurement inputs are available.");
        return;
      }
    } else {
      if (enable_px4_odom_) {
        RCLCPP_INFO_ONCE(get_logger(), "PX4 odometry input is active, IMU input is active, or PX4 odometry is enabled.");
      }
    }

    if (control_msg) {
      if (!is_first_control_msg) {
        last_control_input_time_ = rclcpp::Time(control_msg->header.stamp);
        is_first_control_msg     = true;
        return;
      } else {
        rclcpp::Time current_time = rclcpp::Time(control_msg->header.stamp);
        double       dt_sec       = (current_time - last_control_input_time_).seconds();
        last_control_input_time_  = current_time;

        bool can_predict = true;
        if (dt_sec < 0 || dt_sec > 1.0) {
          can_predict = false;
        }


        if (can_predict && control_msg->last_control_input.data.size() != allocation_matrix_.cols()) {
          control_msg->last_control_input.data.resize(allocation_matrix_.cols());
          RCLCPP_ERROR(get_logger(), "Control input size does not match number of motors (%d). Resizing input vector.", allocation_matrix_.cols());
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
            mekf_->predict(control_input, dt_sec);
            rclcpp::Time stamp = rclcpp::Time(control_msg->header.stamp);
            publishOdometry(predict_pub_, stamp);
            has_prediction = true;
            is_prediction  = true;
          }
        }
      }
    }

    bool has_measurement{false};

    if (is_prediction && px4_odom_msg.has_value() && enable_px4_odom_) {
      mekf_->correct(px4_odom_msg.value());
      has_measurement = true;
    }

    if (has_prediction || has_measurement) {
      publishOdometry(odom_pub_, last_update_time_);
      is_ekf_active_ = true;
    }
  }
  catch (const std::exception &e) {
    RCLCPP_ERROR(get_logger(), "Error in timerCallback: %s", e.what());
  }
}
//}

/* diagnosticsTimerCallback() //{ */
void MekfEstimationManager::diagnosticsTimerCallback() {
  if (!is_active_)
    return;
  try {
    auto diag_msg          = std::make_unique<laser_msgs::msg::EstimationManagerDiagnostics>();
    diag_msg->header.stamp = this->get_clock()->now();

    if (current_active_odometry_name_ == "px4_api_odom" && odom_data_.is_active && !odom_data_.buffer.empty())
      diag_msg->header.frame_id = odom_data_.buffer.begin()->second->header.frame_id;
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

    fill_sensor_status(diag_msg->odometry_sources.emplace_back(), odom_data_, "px4_api_odom");
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
void MekfEstimationManager::publishOdometry(rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr pub, rclcpp::Time &pub_time) {
  const nav_msgs::msg::Odometry &state = mekf_->get_odometry();
  const Eigen::MatrixXd         &cov   = mekf_->get_covariance();

  nav_msgs::msg::Odometry odom_out_msg;
  odom_out_msg.header.stamp = pub_time;
  if (current_active_odometry_name_ == "px4_api_odom" && odom_data_.is_active && !odom_data_.buffer.empty())
    odom_out_msg.header.frame_id = odom_data_.buffer.begin()->second->header.frame_id;
  else if (current_active_odometry_name_ == "openvins_odom" && openvins_odom_data_.is_active && !openvins_odom_data_.buffer.empty())
    odom_out_msg.header.frame_id = openvins_odom_data_.buffer.begin()->second->header.frame_id;
  else if (current_active_odometry_name_ == "fast_lio_odom" && fast_lio_odom_data_.is_active && !fast_lio_odom_data_.buffer.empty())
    odom_out_msg.header.frame_id = fast_lio_odom_data_.buffer.begin()->second->header.frame_id;

  odom_out_msg.pose.pose.position.x    = state.pose.pose.position.x;
  odom_out_msg.pose.pose.position.y    = state.pose.pose.position.y;
  odom_out_msg.pose.pose.position.z    = state.pose.pose.position.z;
  odom_out_msg.pose.pose.orientation.w = state.pose.pose.orientation.w;
  odom_out_msg.pose.pose.orientation.x = state.pose.pose.orientation.x;
  odom_out_msg.pose.pose.orientation.y = state.pose.pose.orientation.y;
  odom_out_msg.pose.pose.orientation.z = state.pose.pose.orientation.z;

  odom_out_msg.twist.twist.linear.x  = state.twist.twist.linear.x;
  odom_out_msg.twist.twist.linear.y  = state.twist.twist.linear.y;
  odom_out_msg.twist.twist.linear.z  = state.twist.twist.linear.z;
  odom_out_msg.twist.twist.angular.x = state.twist.twist.angular.x;
  odom_out_msg.twist.twist.angular.y = state.twist.twist.angular.y;
  odom_out_msg.twist.twist.angular.z = state.twist.twist.angular.z;

  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      odom_out_msg.pose.covariance[i * 6 + j]  = cov(i, j);
      odom_out_msg.twist.covariance[i * 6 + j] = cov(i + 6, j + 6);
    }
  }

  pub->publish(odom_out_msg);
}
//}

}  // namespace laser_uav_managers

RCLCPP_COMPONENTS_REGISTER_NODE(laser_uav_managers::MekfEstimationManager)
