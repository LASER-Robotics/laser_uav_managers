#include <laser_uav_managers/eskf_estimation_manager_node.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <type_traits>
#include <optional>

namespace laser_uav_managers
{
/* ErrorEstimationManager() //{ */
ErrorEstimationManager::ErrorEstimationManager(const rclcpp::NodeOptions &options) : rclcpp_lifecycle::LifecycleNode("state_estimator", options) {
  RCLCPP_INFO(get_logger(), "Creating ErrorEstimationManager...");

  declare_parameter("frequency", 100.0);
  declare_parameter("initial_odometry_source", "px4_api_odom");
  declare_parameter("odometry_source_names", std::vector<std::string>{});
  declare_parameter("odometry_switch_distance_threshold", 0.25);
  declare_parameter("odometry_switch_angle_threshold", 0.5);
  declare_parameter("odometry_switch_velocity_linear_threshold", 0.5);
  declare_parameter("odometry_switch_velocity_angular_threshold", 1.0);
  declare_parameter("sensor_timeout", 0.5);
  declare_parameter("ekf_verbosity", "INFO");
  declare_parameter("estimation_verbosity", "INFO");

  declare_parameter("process_noise_gains.velocity", 0.01);
  declare_parameter("process_noise_gains.orientation", 0.01);
  declare_parameter("process_noise_gains.accelerometer", 0.1);
  declare_parameter("process_noise_gains.gyroscope", 0.1);

  declare_parameter("limits.px4_api.position.x", 1.0);
  declare_parameter("limits.px4_api.position.y", 1.0);
  declare_parameter("limits.px4_api.position.z", 1.0);
  declare_parameter("limits.px4_api.orientation.roll", 1.0);
  declare_parameter("limits.px4_api.orientation.pitch", 1.0);
  declare_parameter("limits.px4_api.orientation.yaw", 1.0);

  declare_parameter("limits.lio.position.x", 1.0);
  declare_parameter("limits.lio.position.y", 1.0);
  declare_parameter("limits.lio.position.z", 1.0);
  declare_parameter("limits.lio.orientation.roll", 1.0);
  declare_parameter("limits.lio.orientation.pitch", 1.0);
  declare_parameter("limits.lio.orientation.yaw", 1.0);

  declare_parameter("limits.vio.position.x", 1.0);
  declare_parameter("limits.vio.position.y", 1.0);
  declare_parameter("limits.vio.position.z", 1.0);
  declare_parameter("limits.vio.orientation.roll", 1.0);
  declare_parameter("limits.vio.orientation.pitch", 1.0);
  declare_parameter("limits.vio.orientation.yaw", 1.0);

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

  RCLCPP_INFO(get_logger(), "ErrorEstimationManager node initialized.");
}
//}

/* ~ErrorEstimationManager() //{ */
ErrorEstimationManager::~ErrorEstimationManager() {
}
//}

/* set_verbosity() //{ */
void ErrorEstimationManager::set_verbosity(const std::string &verbosity) {
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
CallbackReturn ErrorEstimationManager::on_configure(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Configuring ErrorEstimationManager...");

  getParameters();
  configPubSub();
  configTimers();
  configServices();
  setupEKF();

  return CallbackReturn::SUCCESS;
}
//}

/* on_activate() //{ */
CallbackReturn ErrorEstimationManager::on_activate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Activating ErrorEstimationManager...");
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
CallbackReturn ErrorEstimationManager::on_deactivate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Deactivating ErrorEstimationManager...");
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
CallbackReturn ErrorEstimationManager::on_cleanup(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Cleaning up ErrorEstimationManager...");
  es_ekf_.reset();
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
CallbackReturn ErrorEstimationManager::on_shutdown(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Shutting down ErrorEstimationManager...");
  return CallbackReturn::SUCCESS;
}
//}

/* getParameters() //{ */
void ErrorEstimationManager::getParameters() {
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

  get_parameter("process_noise_gains.velocity", _process_noise_gains_.velocity_linear);
  get_parameter("process_noise_gains.orientation", _process_noise_gains_.orientation);
  get_parameter("process_noise_gains.accelerometer", _process_noise_gains_.accelerometer);
  get_parameter("process_noise_gains.gyroscope", _process_noise_gains_.gyroscope);

  double x, y, z, roll, pitch, yaw;
  get_parameter("limits.px4_api.position.x", x);
  get_parameter("limits.px4_api.position.y", y);
  get_parameter("limits.px4_api.position.z", z);
  get_parameter("limits.px4_api.orientation.roll", roll);
  get_parameter("limits.px4_api.orientation.pitch", pitch);
  get_parameter("limits.px4_api.orientation.yaw", yaw);

  _limits_[laser_uav_estimators::SensorIndex::SENSOR_INDEX_PX4].pos = Eigen::Vector3d(x, y, z);
  _limits_[laser_uav_estimators::SensorIndex::SENSOR_INDEX_PX4].rot = Eigen::Vector3d(roll, pitch, yaw);

  get_parameter("limits.lio.position.x", x);
  get_parameter("limits.lio.position.y", y);
  get_parameter("limits.lio.position.z", z);
  get_parameter("limits.lio.orientation.roll", roll);
  get_parameter("limits.lio.orientation.pitch", pitch);
  get_parameter("limits.lio.orientation.yaw", yaw);

  _limits_[laser_uav_estimators::SensorIndex::SENSOR_INDEX_LIDAR].pos = Eigen::Vector3d(x, y, z);
  _limits_[laser_uav_estimators::SensorIndex::SENSOR_INDEX_LIDAR].rot = Eigen::Vector3d(roll, pitch, yaw);

  get_parameter("limits.vio.position.x", x);
  get_parameter("limits.vio.position.y", y);
  get_parameter("limits.vio.position.z", z);
  get_parameter("limits.vio.orientation.roll", roll);
  get_parameter("limits.vio.orientation.pitch", pitch);
  get_parameter("limits.vio.orientation.yaw", yaw);

  _limits_[laser_uav_estimators::SensorIndex::SENSOR_INDEX_VIO].pos = Eigen::Vector3d(x, y, z);
  _limits_[laser_uav_estimators::SensorIndex::SENSOR_INDEX_VIO].rot = Eigen::Vector3d(roll, pitch, yaw);


  get_parameter("measurement_noise_gains.px4_odometry.position", _measurement_noise_gains_.odom[laser_uav_estimators::SensorIndex::SENSOR_INDEX_PX4].position);
  get_parameter("measurement_noise_gains.px4_odometry.orientation",
                _measurement_noise_gains_.odom[laser_uav_estimators::SensorIndex::SENSOR_INDEX_PX4].orientation);
  get_parameter("measurement_noise_gains.px4_odometry.linear_velocity",
                _measurement_noise_gains_.odom[laser_uav_estimators::SensorIndex::SENSOR_INDEX_PX4].velocity_linear);
  get_parameter("measurement_noise_gains.px4_odometry.angular_velocity",
                _measurement_noise_gains_.odom[laser_uav_estimators::SensorIndex::SENSOR_INDEX_PX4].velocity_angular);
  get_parameter("measurement_noise_gains.openvins.position", _measurement_noise_gains_.odom[laser_uav_estimators::SensorIndex::SENSOR_INDEX_VIO].position);
  get_parameter("measurement_noise_gains.openvins.orientation",
                _measurement_noise_gains_.odom[laser_uav_estimators::SensorIndex::SENSOR_INDEX_VIO].orientation);
  get_parameter("measurement_noise_gains.openvins.linear_velocity",
                _measurement_noise_gains_.odom[laser_uav_estimators::SensorIndex::SENSOR_INDEX_VIO].velocity_linear);
  get_parameter("measurement_noise_gains.openvins.angular_velocity",
                _measurement_noise_gains_.odom[laser_uav_estimators::SensorIndex::SENSOR_INDEX_VIO].velocity_angular);
  get_parameter("measurement_noise_gains.fast_lio.position", _measurement_noise_gains_.odom[laser_uav_estimators::SensorIndex::SENSOR_INDEX_LIDAR].position);
  get_parameter("measurement_noise_gains.fast_lio.orientation",
                _measurement_noise_gains_.odom[laser_uav_estimators::SensorIndex::SENSOR_INDEX_LIDAR].orientation);
  get_parameter("measurement_noise_gains.fast_lio.linear_velocity",
                _measurement_noise_gains_.odom[laser_uav_estimators::SensorIndex::SENSOR_INDEX_LIDAR].velocity_linear);
  get_parameter("measurement_noise_gains.fast_lio.angular_velocity",
                _measurement_noise_gains_.odom[laser_uav_estimators::SensorIndex::SENSOR_INDEX_LIDAR].velocity_angular);
  get_parameter("measurement_noise_gains.imu.position", _measurement_noise_gains_.imu.position);
  get_parameter("measurement_noise_gains.imu.orientation", _measurement_noise_gains_.imu.orientation);
  get_parameter("measurement_noise_gains.imu.linear_velocity", _measurement_noise_gains_.imu.velocity_linear);
  get_parameter("measurement_noise_gains.imu.angular_velocity", _measurement_noise_gains_.imu.velocity_angular);
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
void ErrorEstimationManager::configPubSub() {
  RCLCPP_INFO(get_logger(), "Configuring publishers and subscribers...");
  odom_pub_        = create_publisher<nav_msgs::msg::Odometry>("odometry_out", 10);
  predict_pub_     = create_publisher<nav_msgs::msg::Odometry>("odometry_predict", 10);
  diagnostics_pub_ = create_publisher<laser_msgs::msg::EstimationManagerDiagnostics>("~/diagnostics", 10);

  odometry_px4_sub_ =
      create_subscription<nav_msgs::msg::Odometry>("odometry_in", 10, std::bind(&ErrorEstimationManager::odometryPx4Callback, this, std::placeholders::_1));
  odometry_fast_lio_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odometry_fast_lio_in", 10, std::bind(&ErrorEstimationManager::odometryFastLioCallback, this, std::placeholders::_1));
  odometry_openvins_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odometry_openvins_in", 10, std::bind(&ErrorEstimationManager::odometryOpenVinsCallback, this, std::placeholders::_1));
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("imu_in", 10, std::bind(&ErrorEstimationManager::imuCallback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Publishers and subscribers configured.");
}
//}

/* configTimers() //{ */
void ErrorEstimationManager::configTimers() {
  RCLCPP_INFO(get_logger(), "Configuring timers...");
  timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / frequency_), std::bind(&ErrorEstimationManager::timerCallback, this));
  diagnostics_timer_ =
      create_wall_timer(std::chrono::duration<double>(1 / (frequency_ / 10)), std::bind(&ErrorEstimationManager::diagnosticsTimerCallback, this));

  RCLCPP_INFO(get_logger(), "Timers configured.");
}
//}

/* configServices() //{ */
void ErrorEstimationManager::configServices() {
  RCLCPP_INFO(get_logger(), "Configuring services... ");
  set_odometry_service_ = this->create_service<laser_msgs::srv::SetString>(
      "~/set_odometry", std::bind(&ErrorEstimationManager::setOdometryCallback, this, std::placeholders::_1, std::placeholders::_2));
}
//}

/* setupEKF() //{ */
void ErrorEstimationManager::setupEKF() {
  RCLCPP_INFO(get_logger(), "Configuring ES-EKF...");

  es_ekf_ = std::make_unique<laser_uav_estimators::ErrorStateEstimator>(laser_uav_estimators::SensorIndex::NUM_SENSORS, _process_noise_gains_,
                                                                        _measurement_noise_gains_, _limits_, ekf_verbosity_);

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
void ErrorEstimationManager::odometryPx4Callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(px4_odom_data_.mtx);
  px4_odom_data_.buffer[msg->header.stamp] = msg;
  // RCLCPP_DEBUG(
  //     get_logger(), "Received PX4 odometry message at time %.3f s, frequency: %.2f Hz", msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9,
  //     ((px4_odom_data_.last_msg != nullptr) ? (1.0 / (rclcpp::Time(msg->header.stamp) - rclcpp::Time(px4_odom_data_.last_msg->header.stamp)).seconds()) :
  //     0.0));
  px4_odom_data_.last_msg = msg;
}
//}

/* odometryOpenVinsCallback() //{ */
void ErrorEstimationManager::odometryOpenVinsCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(openvins_odom_data_.mtx);
  openvins_odom_data_.buffer[msg->header.stamp] = msg;
  if (enable_openvins_odom_)
    odom_pub_->publish(*msg);
  // RCLCPP_DEBUG(get_logger(), "Received OpenVins odometry message at time %.3f s, frequency: %.2f Hz", msg->header.stamp.sec + msg->header.stamp.nanosec *
  // 1e-9,
  //              ((openvins_odom_data_.last_msg != nullptr)
  //                   ? (1.0 / (rclcpp::Time(msg->header.stamp) - rclcpp::Time(openvins_odom_data_.last_msg->header.stamp)).seconds())
  //                   : 0.0));
  openvins_odom_data_.last_msg = msg;
}
//}

/* odometryFastLioCallback() //{ */
void ErrorEstimationManager::odometryFastLioCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
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
void ErrorEstimationManager::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(imu_data_.mtx);
  imu_data_.buffer[msg->header.stamp] = msg;
  // RCLCPP_DEBUG(get_logger(), "Received IMU message at time %.3f s, frequency: %.2f Hz", msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9,
  //              ((imu_data_.last_msg != nullptr) ? (1.0 / (rclcpp::Time(msg->header.stamp) - rclcpp::Time(imu_data_.last_msg->header.stamp)).seconds()) :
  //              0.0));
  imu_data_.last_msg = msg;
}
//}

/* setOdometryCallback() //{ */
void ErrorEstimationManager::setOdometryCallback(const std::shared_ptr<laser_msgs::srv::SetString::Request> request,
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

  nav_msgs::msg::Odometry current_state = es_ekf_->get_odometry();
  const auto             &new_odom_pose = newest_msg_it->second->pose.pose;

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
std::optional<MsgT> ErrorEstimationManager::getSynchronizedMessage(const rclcpp::Time &ref_time, SensorDataBuffer<MsgT> &sensor_data, std::string sensor_name) {
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
    // RCLCPP_DEBUG(get_logger(), "[%s]: ACCEPTED: Best match (%.2f ms) within tolerance (%.2f ms).", sensor_name.c_str(), min_diff.seconds() * 1000.0,
    //              sensor_data.tolerance.seconds() * 1000.0);
    return msg_copy;
  }

  RCLCPP_DEBUG(get_logger(), "[%s]: REJECTED: Best match (%.2f ms) is outside tolerance (%.2f ms).", sensor_name.c_str(), min_diff.seconds() * 1000.0,
               sensor_data.tolerance.seconds() * 1000.0);

  return std::nullopt;
}
//}

/* pruneSensorBuffer() //{ */
template <typename MsgT>
void ErrorEstimationManager::pruneSensorBuffer(const rclcpp::Time &now, SensorDataBuffer<MsgT> &sensor_data, std::string sensor_name) {
  std::lock_guard<std::mutex> lock(sensor_data.mtx);
  if (sensor_data.buffer.empty())
    return;

  const rclcpp::Time cutoff_time = now - (sensor_data.timeout * 2.0);

  auto first_to_keep_it = sensor_data.buffer.upper_bound(cutoff_time);

  // RCLCPP_DEBUG(
  //     get_logger(), "[%s] Pruning sensor buffer. %s, first kept time: %.2f s", sensor_name.c_str(),
  //     (first_to_keep_it != sensor_data.buffer.begin() ? "Removing old messages." : "No messages to remove."),
  //     (first_to_keep_it != sensor_data.buffer.begin() && first_to_keep_it != sensor_data.buffer.end()) ? rclcpp::Time(first_to_keep_it->first).seconds() :
  //     0.0);
  sensor_data.buffer.erase(sensor_data.buffer.begin(), first_to_keep_it);
}
//}

/* timerCallback() //{ */
void ErrorEstimationManager::timerCallback() {
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

    auto imu_msg = getSynchronizedMessage(reference_time, imu_data_, "IMU");

    // RCLCPP_DEBUG(get_logger(), "Synchronized Messages - PX4 Odom: %s, OpenVINS Odom: %s, FastLIO Odom: %s, IMU: %s", px4_odom_msg ? "YES" : "NO",
    //              openvins_odom_msg ? "YES" : "NO", fast_lio_odom_msg ? "YES" : "NO", imu_msg ? "YES" : "NO");
    // RCLCPP_DEBUG(get_logger(), "Buffer Sizes - PX4 Odom: %zu, OpenVINS Odom: %zu, FastLIO Odom: %zu, IMU: %zu", px4_odom_data_.buffer.size(),
    //              openvins_odom_data_.buffer.size(), fast_lio_odom_data_.buffer.size(), imu_data_.buffer.size());

    pruneSensorBuffer(reference_time, px4_odom_data_, "PX4_ODOMETRY");
    pruneSensorBuffer(reference_time, openvins_odom_data_, "OPENVINS_ODOMETRY");
    pruneSensorBuffer(reference_time, fast_lio_odom_data_, "FAST_LIO_ODOMETRY");
    pruneSensorBuffer(reference_time, imu_data_, "IMU");

    bool has_prediction{false};
    if (enable_px4_odom_ && !px4_odom_data_.is_active && !imu_data_.is_active) {
      if (!px4_odom_data_.is_active)
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

    if (enable_fast_lio_odom_ && !fast_lio_odom_data_.is_active && !imu_data_.is_active) {
      if (!fast_lio_odom_data_.is_active)
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Fast-LIO odometry input is inactive.");
      if (!imu_data_.is_active)
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "IMU input is inactive.");
      if (!is_ekf_active_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "EKF is active but no valid measurement inputs are available.");
        return;
      }
    } else {
      if (enable_fast_lio_odom_) {
        RCLCPP_INFO_ONCE(get_logger(), "Fast-LIO odometry input is active, IMU input is active, or Fast-LIO odometry is enabled.");
      }
    }

    if (enable_openvins_odom_ && !openvins_odom_data_.is_active && !imu_data_.is_active) {
      if (!openvins_odom_data_.is_active)
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "OpenVINS odometry input is inactive.");
      if (!imu_data_.is_active)
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "IMU input is inactive.");
      if (!is_ekf_active_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "EKF is active but no valid measurement inputs are available.");
        return;
      }
    } else {
      if (enable_openvins_odom_) {
        RCLCPP_INFO_ONCE(get_logger(), "OpenVINS odometry input is active, IMU input is active, or OpenVINS odometry is enabled.");
      }
    }
    std::cout << "teste 4" << std::endl;

    RCLCPP_INFO_ONCE(get_logger(), "Starting EKF updates.");

    std::cout << "IMU: " << (imu_msg ? "YES" : "NO");

    if (enable_openvins_odom_)
      std::cout << ", openvins: " << (openvins_odom_msg ? "YES" : "NO");
    if (enable_px4_odom_)
      std::cout << ", px4: " << (px4_odom_msg ? "YES" : "NO");
    if (enable_fast_lio_odom_)
      std::cout << ", fast_lio: " << (fast_lio_odom_msg ? "YES" : "NO");
    std::cout << ", ES-EKF Initialized: " << (is_initialized_es_ekf_ ? "YES" : "NO") << std::endl;

    bool any_odom_enabled = enable_px4_odom_ || enable_openvins_odom_ || enable_fast_lio_odom_;

    bool active_odom_received =
        (enable_px4_odom_ && px4_odom_msg) || (enable_openvins_odom_ && openvins_odom_msg) || (enable_fast_lio_odom_ && fast_lio_odom_msg);

    if (!imu_msg && !active_odom_received) {
      RCLCPP_WARN(get_logger(), "Waiting for sensors... IMU: %s, Odom: %s", imu_msg ? "OK" : "MISSING", active_odom_received ? "OK" : "MISSING");
      return;
    }

    if (!is_initialized_es_ekf_) {
      RCLCPP_INFO(get_logger(), "All required sensors received. Starting ES-EKF...");
      is_initialized_es_ekf_ = true;
    }

    std::cout << "teste 5" << std::endl;

    RCLCPP_DEBUG(get_logger(), "Performing EKF prediction step...");
    RCLCPP_DEBUG(get_logger(), "IMU: %s", imu_msg ? "YES" : "NO");

    if (imu_msg) {
      rclcpp::Time current_time = rclcpp::Time(imu_msg->header.stamp);
      double       dt_sec       = 0.0;
      bool         can_predict  = true;
      std::cout << "teste 5.1, predict: " << can_predict << std::endl;

      // Handle first message initialization to avoid clock type mismatch (System Time vs ROS Time)
      if (last_imu_time_.nanoseconds() == 0) {
        last_imu_time_ = current_time;
        can_predict    = false;
        return;
      } else {
        dt_sec         = (current_time - last_imu_time_).seconds();
        last_imu_time_ = current_time;
      }
      std::cout << "teste 5.2, predict: " << can_predict << std::endl;

      if (dt_sec < 0 || dt_sec > 1.0) {
        can_predict = false;
      }

      RCLCPP_DEBUG(get_logger(), "IMU dt: %.6f s, can_predict: %s", dt_sec, can_predict ? "YES" : "NO");
      std::cout << "teste 5.3, predict: " << can_predict << std::endl;

      if (can_predict) {
        es_ekf_->predict(*imu_msg, dt_sec);
        rclcpp::Time stamp = rclcpp::Time(imu_msg->header.stamp);
        publishOdometry(predict_pub_, stamp);
        has_prediction = true;
      }
      std::cout << "teste 5.4, predict: " << can_predict << ", has_prediction: " << has_prediction << std::endl;
    }

    std::cout << "teste 6" << std::endl;


    laser_uav_estimators::MeasurementPackage pkg;
    bool                                     has_measurement{false};
    if (px4_odom_msg && enable_px4_odom_) {
      const auto                                   &px4_odom = *px4_odom_msg;
      Eigen::Map<const Eigen::Matrix<double, 6, 6>> px4_pose_cov(px4_odom.pose.covariance.data());
      Eigen::Map<const Eigen::Matrix<double, 6, 6>> px4_twist_cov(px4_odom.twist.covariance.data());

      pkg.px4_api       = &(*px4_odom_msg);
      last_update_time_ = px4_odom_msg->header.stamp;
      has_measurement   = true;
    } else if (openvins_odom_msg && enable_openvins_odom_) {
      const auto                                   &openvins_odom = *openvins_odom_msg;
      Eigen::Map<const Eigen::Matrix<double, 6, 6>> openvins_pose_cov(openvins_odom.pose.covariance.data());
      Eigen::Map<const Eigen::Matrix<double, 6, 6>> openvins_twist_cov(openvins_odom.twist.covariance.data());

      pkg.openvins      = &(*openvins_odom_msg);
      last_update_time_ = openvins_odom_msg->header.stamp;
      has_measurement   = true;
    } else if (fast_lio_odom_msg && enable_fast_lio_odom_) {
      auto &fast_lio_odom = *fast_lio_odom_msg;

      fast_lio_odom.twist.twist.angular.x = std::numeric_limits<double>::quiet_NaN();
      fast_lio_odom.twist.twist.angular.y = std::numeric_limits<double>::quiet_NaN();
      fast_lio_odom.twist.twist.angular.z = std::numeric_limits<double>::quiet_NaN();

      Eigen::Map<const Eigen::Matrix<double, 6, 6>> fast_lio_pose_cov(fast_lio_odom.pose.covariance.data());
      Eigen::Map<const Eigen::Matrix<double, 6, 6>> fast_lio_twist_cov(fast_lio_odom.twist.covariance.data());
      pkg.fast_lio      = &fast_lio_odom;
      last_update_time_ = fast_lio_odom.header.stamp;
      has_measurement   = true;
    }

    RCLCPP_DEBUG(get_logger(), "EKF Updates - Prediction: %s, Measurement: %s", has_prediction ? "YES" : "NO", has_measurement ? "YES" : "NO");

    if (has_measurement)
      es_ekf_->correct(pkg);

    if ((has_prediction || has_measurement) && !enable_openvins_odom_) {
      publishOdometry(odom_pub_, last_update_time_);
      is_ekf_active_ = true;
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
  // count++;
  // if (count > stop) {
  //   rclcpp::shutdown();
  // }
}
//}

/* diagnosticsTimerCallback() //{ */
void ErrorEstimationManager::diagnosticsTimerCallback() {
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
void ErrorEstimationManager::publishOdometry(rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr pub, rclcpp::Time &pub_time) {
  nav_msgs::msg::Odometry odom_out_msg;
  odom_out_msg.header.stamp    = pub_time;
  odom_out_msg.header.frame_id = "Estimation_manager";

  nav_msgs::msg::Odometry current_state = es_ekf_->get_odometry();
  odom_out_msg.pose                     = current_state.pose;
  odom_out_msg.twist                    = current_state.twist;

  pub->publish(odom_out_msg);
}
//}

}  // namespace laser_uav_managers

RCLCPP_COMPONENTS_REGISTER_NODE(laser_uav_managers::ErrorEstimationManager)
