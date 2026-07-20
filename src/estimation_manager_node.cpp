#include <rclcpp_components/register_node_macro.hpp>

#include <laser_uav_managers/estimation_manager_node.hpp>

namespace laser_uav_managers
{
/* EstimationManager() //{ */
EstimationManager::EstimationManager(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("state_estimator", options)
{
  RCLCPP_INFO(get_logger(), "Creating EstimationManager...");

  declare_parameter("uav_name", "uav");
  declare_parameter("frequency", 100.0);
  declare_parameter("initial_odometry_source", "px4_api_odom");
  declare_parameter("odometry_source_names", std::vector<std::string>{});
  declare_parameter("odometry_switch_distance_threshold", 0.25);
  declare_parameter("odometry_switch_angle_threshold", 0.5);
  declare_parameter("odometry_switch_velocity_linear_threshold", 0.5);
  declare_parameter("odometry_switch_velocity_angular_threshold", 1.0);
  declare_parameter("sensor_timeout", 0.5);
  declare_parameter("estimation_verbosity", "INFO");
  declare_parameter("ekf_verbosity", "INFO");

  declare_parameter("multirotor_parameters.mass", 1.0);
  declare_parameter("multirotor_parameters.inertia", std::vector<double>{0.01, 0.01, 0.01});
  declare_parameter(
    "multirotor_parameters.G1", std::vector<double>{0.1, -0.1, -0.1, 0.1, 0.1, 0.1, -0.1, -0.1});

  declare_parameter("process_noise_gains.position_xy", 0.01);
  declare_parameter("process_noise_gains.position_z", 0.01);
  declare_parameter("process_noise_gains.orientation", 0.01);
  declare_parameter("process_noise_gains.linear_velocity_xy", 0.1);
  declare_parameter("process_noise_gains.linear_velocity_z", 0.1);
  declare_parameter("process_noise_gains.angular_velocity", 0.1);

  declare_parameter("measurement_noise_gains.px4_odometry.position_xy", 1.0);
  declare_parameter("measurement_noise_gains.px4_odometry.position_z", 1.0);
  declare_parameter("measurement_noise_gains.px4_odometry.orientation", 1.0);
  declare_parameter("measurement_noise_gains.px4_odometry.linear_velocity_xy", 1.0);
  declare_parameter("measurement_noise_gains.px4_odometry.linear_velocity_z", 1.0);
  declare_parameter("measurement_noise_gains.px4_odometry.angular_velocity", 1.0);

  declare_parameter("measurement_noise_gains.openvins.position_xy", 1.0);
  declare_parameter("measurement_noise_gains.openvins.position_z", 1.0);
  declare_parameter("measurement_noise_gains.openvins.orientation", 1.0);
  declare_parameter("measurement_noise_gains.openvins.linear_velocity_xy", 1.0);
  declare_parameter("measurement_noise_gains.openvins.linear_velocity_z", 1.0);
  declare_parameter("measurement_noise_gains.openvins.angular_velocity", 1.0);

  declare_parameter("measurement_noise_gains.fast_lio.position_xy", 1.0);
  declare_parameter("measurement_noise_gains.fast_lio.position_z", 1.0);
  declare_parameter("measurement_noise_gains.fast_lio.orientation", 1.0);
  declare_parameter("measurement_noise_gains.fast_lio.linear_velocity_xy", 1.0);
  declare_parameter("measurement_noise_gains.fast_lio.linear_velocity_z", 1.0);
  declare_parameter("measurement_noise_gains.fast_lio.angular_velocity", 1.0);

  declare_parameter("measurement_noise_gains.garmin.position_z", 1.0);

  declare_parameter("px4_odom_tolerance", 0.1);
  declare_parameter("px4_odom_timeout", 0.5);
  declare_parameter("px4_odom_covariance", 1.0);
  declare_parameter("openvins_odom_tolerance", 0.1);
  declare_parameter("openvins_odom_timeout", 0.5);
  declare_parameter("openvins_odom_covariance", 1.0);
  declare_parameter("fast_lio_odom_tolerance", 0.1);
  declare_parameter("fast_lio_odom_timeout", 0.5);
  declare_parameter("fast_lio_odom_covariance", 1.0);
  declare_parameter("control_tolerance", 0.1);
  declare_parameter("control_timeout", 0.5);
  declare_parameter("garmin_tolerance", 0.1);
  declare_parameter("garmin_timeout", 0.5);
  declare_parameter("garmin_covariance", 1.0);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  RCLCPP_INFO(get_logger(), "EstimationManager node initialized.");
}
/*//}*/

/* ~EstimationManager() //{ */
EstimationManager::~EstimationManager()
{}
/*//}*/

/* set_verbosity() //{ */
void EstimationManager::set_verbosity(const std::string & verbosity)
{
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
/*//}*/

/* on_configure() //{ */
CallbackReturn EstimationManager::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring EstimationManager...");

  get_parameters();
  configure_pub_sub();
  configure_timers();
  configure_services();
  setup_ekf();

  return CallbackReturn::SUCCESS;
}
/*//}*/

/* on_activate() //{ */
CallbackReturn EstimationManager::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating EstimationManager...");
  odom_pub_->on_activate();
  diagnostics_pub_->on_activate();

  is_active_ = true;
  timer_->reset();
  diagnostics_timer_->reset();
  return CallbackReturn::SUCCESS;
}
/*//}*/

/* on_deactivate() //{ */
CallbackReturn EstimationManager::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating EstimationManager...");
  is_active_ = false;
  timer_->cancel();
  diagnostics_timer_->cancel();
  odom_pub_->on_deactivate();
  diagnostics_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}
/*//}*/

/* on_cleanup() //{ */
CallbackReturn EstimationManager::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up EstimationManager...");
  odom_pub_.reset();
  diagnostics_pub_.reset();
  odometry_px4_sub_.reset();
  odometry_fast_lio_sub_.reset();
  odometry_openvins_sub_.reset();
  control_sub_.reset();
  timer_.reset();
  garmin_sub_.reset();
  diagnostics_timer_.reset();

  return CallbackReturn::SUCCESS;
}
/*//}*/

/* on_shutdown() //{ */
CallbackReturn EstimationManager::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down EstimationManager...");
  return CallbackReturn::SUCCESS;
}
/*//}*/

/* get_parameters() //{ */
void EstimationManager::get_parameters()
{
  RCLCPP_INFO(get_logger(), "Loading parameters...");
  get_parameter("uav_name", uav_name_);
  get_parameter("frequency", frequency_);
  get_parameter("initial_odometry_source", current_active_odometry_name_);
  get_parameter("odometry_source_names", odometry_source_names_);
  get_parameter("odometry_switch_distance_threshold", odometry_switch_distance_threshold_);
  get_parameter("odometry_switch_angle_threshold", odometry_switch_angle_threshold_);
  get_parameter(
    "odometry_switch_velocity_linear_threshold", odometry_switch_velocity_linear_threshold_);
  get_parameter(
    "odometry_switch_velocity_angular_threshold", odometry_switch_velocity_angular_threshold_);
  get_parameter("sensor_timeout", sensor_timeout_);
  get_parameter("estimation_verbosity", estimation_verbosity_);
  get_parameter("ekf_verbosity", ekf_verbosity_);

  set_verbosity(estimation_verbosity_);

  get_parameter("multirotor_parameters.mass", mass_);
  get_parameter("multirotor_parameters.inertia", inertia_vec_);
  get_parameter("multirotor_parameters.c_thrust", thrust_coefficient_);

  std::vector<double> G1_vec;
  get_parameter("multirotor_parameters.G1", G1_vec);
  int num_cols = G1_vec.size() / 4;
  allocation_matrix_ =
    Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      G1_vec.data(), 4, num_cols);

  get_parameter("process_noise_gains.position_xy", process_noise_gains_.position_xy);
  get_parameter("process_noise_gains.position_z", process_noise_gains_.position_z);
  get_parameter("process_noise_gains.orientation", process_noise_gains_.orientation);
  get_parameter("process_noise_gains.linear_velocity_xy", process_noise_gains_.velocity_linear_xy);
  get_parameter("process_noise_gains.linear_velocity_z", process_noise_gains_.velocity_linear_z);
  get_parameter("process_noise_gains.angular_velocity", process_noise_gains_.velocity_angular);

  get_parameter(
    "measurement_noise_gains.px4_odometry.position_xy",
    px4_measurement_noise_gains_.odometry.position_xy);
  get_parameter(
    "measurement_noise_gains.px4_odometry.position_z",
    px4_measurement_noise_gains_.odometry.position_z);
  get_parameter(
    "measurement_noise_gains.px4_odometry.orientation",
    px4_measurement_noise_gains_.odometry.orientation);
  get_parameter(
    "measurement_noise_gains.px4_odometry.linear_velocity_xy",
    px4_measurement_noise_gains_.odometry.velocity_linear_xy);
  get_parameter(
    "measurement_noise_gains.px4_odometry.linear_velocity",
    px4_measurement_noise_gains_.odometry.velocity_linear_z);
  get_parameter(
    "measurement_noise_gains.px4_odometry.angular_velocity",
    px4_measurement_noise_gains_.odometry.velocity_angular);

  get_parameter(
    "measurement_noise_gains.openvins.position_xy",
    openvins_measurement_noise_gains_.odometry.position_xy);
  get_parameter(
    "measurement_noise_gains.openvins.position_z",
    openvins_measurement_noise_gains_.odometry.position_z);
  get_parameter(
    "measurement_noise_gains.openvins.orientation",
    openvins_measurement_noise_gains_.odometry.orientation);
  get_parameter(
    "measurement_noise_gains.openvins.linear_velocity_xy",
    openvins_measurement_noise_gains_.odometry.velocity_linear_xy);
  get_parameter(
    "measurement_noise_gains.openvins.linear_velocity_z",
    openvins_measurement_noise_gains_.odometry.velocity_linear_z);
  get_parameter(
    "measurement_noise_gains.openvins.angular_velocity",
    openvins_measurement_noise_gains_.odometry.velocity_angular);

  get_parameter(
    "measurement_noise_gains.fast_lio.position_xy",
    fast_lio_measurement_noise_gains_.odometry.position_xy);
  get_parameter(
    "measurement_noise_gains.fast_lio.position_z",
    fast_lio_measurement_noise_gains_.odometry.position_z);
  get_parameter(
    "measurement_noise_gains.fast_lio.orientation",
    fast_lio_measurement_noise_gains_.odometry.orientation);
  get_parameter(
    "measurement_noise_gains.fast_lio.linear_velocity_xy",
    fast_lio_measurement_noise_gains_.odometry.velocity_linear_xy);
  get_parameter(
    "measurement_noise_gains.fast_lio.linear_velocity_z",
    fast_lio_measurement_noise_gains_.odometry.velocity_linear_z);
  get_parameter(
    "measurement_noise_gains.fast_lio.angular_velocity",
    fast_lio_measurement_noise_gains_.odometry.velocity_angular);

  double garmin_position_z_gain;
  get_parameter("measurement_noise_gains.garmin.position_z", garmin_position_z_gain);

  fast_lio_measurement_noise_gains_.garmin.position_z = garmin_position_z_gain;
  openvins_measurement_noise_gains_.garmin.position_z = garmin_position_z_gain;
  px4_measurement_noise_gains_.garmin.position_z = garmin_position_z_gain;

  double tolerance, timeout;

  get_parameter("px4_odom_tolerance", tolerance);
  get_parameter("px4_odom_timeout", timeout);
  get_parameter("px4_odom_covariance", px4_odom_covariance_);
  px4_odom_data_.tolerance = rclcpp::Duration::from_seconds(tolerance);
  px4_odom_data_.timeout = rclcpp::Duration::from_seconds(timeout);

  get_parameter("openvins_odom_tolerance", tolerance);
  get_parameter("openvins_odom_timeout", timeout);
  get_parameter("openvins_odom_covariance", openvins_odom_covariance_);
  openvins_odom_data_.tolerance = rclcpp::Duration::from_seconds(tolerance);
  openvins_odom_data_.timeout = rclcpp::Duration::from_seconds(timeout);

  get_parameter("fast_lio_odom_tolerance", tolerance);
  get_parameter("fast_lio_odom_timeout", timeout);
  get_parameter("fast_lio_odom_covariance", fast_lio_odom_covariance_);
  fast_lio_odom_data_.tolerance = rclcpp::Duration::from_seconds(tolerance);
  fast_lio_odom_data_.timeout = rclcpp::Duration::from_seconds(timeout);

  get_parameter("garmin_tolerance", tolerance);
  get_parameter("garmin_timeout", timeout);
  get_parameter("garmin_covariance", garmin_covariance_);
  garmin_data_.tolerance = rclcpp::Duration::from_seconds(tolerance);
  garmin_data_.timeout = rclcpp::Duration::from_seconds(timeout);

  get_parameter("control_tolerance", tolerance);
  get_parameter("control_timeout", timeout);
  control_data_.tolerance = rclcpp::Duration::from_seconds(tolerance);
  control_data_.timeout = rclcpp::Duration::from_seconds(timeout);

  RCLCPP_INFO(get_logger(), "Parameters loaded.");
}
/*//}*/

/* configure_pub_sub() //{ */
void EstimationManager::configure_pub_sub()
{
  RCLCPP_INFO(get_logger(), "Configuring publishers and subscribers...");
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odometry_out", 10);
  diagnostics_pub_ =
    create_publisher<laser_msgs::msg::EstimationManagerDiagnostics>("~/diagnostics", 10);

  odometry_px4_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "odometry_in", 10,
    std::bind(&EstimationManager::odometry_px4_callback, this, std::placeholders::_1));
  odometry_fast_lio_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "odometry_fast_lio_in", 10,
    std::bind(&EstimationManager::odometry_fast_lio_callback, this, std::placeholders::_1));
  odometry_openvins_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "odometry_openvins_in", 10,
    std::bind(&EstimationManager::odometry_openvins_callback, this, std::placeholders::_1));
  control_sub_ = create_subscription<laser_msgs::msg::UavControlDiagnostics>(
    "control_in", 10, std::bind(&EstimationManager::control_callback, this, std::placeholders::_1));
  garmin_sub_ = create_subscription<sensor_msgs::msg::Range>(
    "garmin_in", 10,
    std::bind(&EstimationManager::garmin_range_callback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Publishers and subscribers configured.");
}
/*//}*/

/* configure_timers() //{ */
void EstimationManager::configure_timers()
{
  RCLCPP_INFO(get_logger(), "Configuring timers...");
  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / frequency_),
    std::bind(&EstimationManager::timer_callback, this));
  diagnostics_timer_ = create_wall_timer(
    std::chrono::duration<double>(1 / (frequency_ / 10)),
    std::bind(&EstimationManager::diagnostics_timer_callback, this));

  RCLCPP_INFO(get_logger(), "Timers configured.");
}
/*//}*/

/* configure_services() //{ */
void EstimationManager::configure_services()
{
  RCLCPP_INFO(get_logger(), "Configuring services... ");
  set_odometry_service_ = this->create_service<laser_msgs::srv::SetString>(
    "~/set_odometry", std::bind(
                        &EstimationManager::set_odometry_callback, this, std::placeholders::_1,
                        std::placeholders::_2));
}
/*//}*/

/* setup_ekf() //{ */
void EstimationManager::setup_ekf()
{
  RCLCPP_INFO(get_logger(), "Configuring EKF...");
  Eigen::Matrix3d inertia =
    Eigen::Vector3d(inertia_vec_[0], inertia_vec_[1], inertia_vec_[2]).asDiagonal();

  if (current_active_odometry_name_ == "px4_api_odom") {
    mekf_ = std::make_unique<laser_uav_estimators::MEKFEstimator>(
      mass_, allocation_matrix_, inertia, px4_measurement_noise_gains_, process_noise_gains_,
      ekf_verbosity_);
    enable_px4_odom_ = true;
    enable_openvins_odom_ = false;
    enable_fast_lio_odom_ = false;
    RCLCPP_INFO(get_logger(), "Initial odometry source set to \'px4_api_odom\'.");
  } else if (current_active_odometry_name_ == "openvins_odom") {
    mekf_ = std::make_unique<laser_uav_estimators::MEKFEstimator>(
      mass_, allocation_matrix_, inertia, openvins_measurement_noise_gains_, process_noise_gains_,
      ekf_verbosity_);
    enable_px4_odom_ = false;
    enable_openvins_odom_ = true;
    enable_fast_lio_odom_ = false;
    RCLCPP_INFO(get_logger(), "Initial odometry source set to \'openvins_odom\'.");
  } else if (current_active_odometry_name_ == "fast_lio_odom") {
    mekf_ = std::make_unique<laser_uav_estimators::MEKFEstimator>(
      mass_, allocation_matrix_, inertia, fast_lio_measurement_noise_gains_, process_noise_gains_,
      ekf_verbosity_);
    enable_px4_odom_ = false;
    enable_openvins_odom_ = false;
    enable_fast_lio_odom_ = true;
    RCLCPP_INFO(get_logger(), "Initial odometry source set to \'fast_lio_odom\'.");
  } else {
    current_active_odometry_name_ = "px4_api_odom";
    mekf_ = std::make_unique<laser_uav_estimators::MEKFEstimator>(
      mass_, allocation_matrix_, inertia, px4_measurement_noise_gains_, process_noise_gains_,
      ekf_verbosity_);
    enable_px4_odom_ = true;
    enable_openvins_odom_ = false;
    enable_fast_lio_odom_ = false;
    RCLCPP_WARN(get_logger(), "Invalid initial odometry source. Using 'px4_api_odom' as default.");
  }

  RCLCPP_INFO(get_logger(), "EKF configured.");
}
/*//}*/

/* odometry_px4_callback() //{ */
void EstimationManager::odometry_px4_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(px4_odom_data_.mtx);
  RCLCPP_DEBUG_THROTTLE(
    get_logger(), *get_clock(), 5000,
    "Received PX4 odometry message at time %.3f s, frequency: %.2f Hz",
    static_cast<double>(msg->header.stamp.sec) +
      static_cast<double>(msg->header.stamp.nanosec) * 1e-9,
    ((px4_odom_data_.last_msg != nullptr)
       ? (1.0 /
          (rclcpp::Time(msg->header.stamp) - rclcpp::Time(px4_odom_data_.last_msg->header.stamp))
            .seconds())
       : 0.0));
  px4_odom_data_.is_active = true;
  px4_odom_data_.last_msg = msg;
}
/*//}*/

/* odometry_openvins_callback() //{ */
void EstimationManager::odometry_openvins_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(openvins_odom_data_.mtx);
  if (enable_openvins_odom_)
    odom_pub_->publish(*msg);
  RCLCPP_DEBUG_THROTTLE(
    get_logger(), *get_clock(), 5000,
    "Received OpenVins odometry message at time %.3f s, frequency: %.2f Hz",
    msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9,
    ((openvins_odom_data_.last_msg != nullptr)
       ? (1.0 / (rclcpp::Time(msg->header.stamp) -
                 rclcpp::Time(openvins_odom_data_.last_msg->header.stamp))
                  .seconds())
       : 0.0));
  openvins_odom_data_.is_active = true;
  openvins_odom_data_.last_msg = msg;
}
/*//}*/

/* odometry_fast_lio_callback() //{ */
void EstimationManager::odometry_fast_lio_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(fast_lio_odom_data_.mtx);
  RCLCPP_DEBUG_THROTTLE(
    get_logger(), *get_clock(), 5000,
    "Received Fast-LIO odometry message at time %.3f s, frequency: %.2f Hz",
    msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9,
    ((fast_lio_odom_data_.last_msg != nullptr)
       ? (1.0 / (rclcpp::Time(msg->header.stamp) -
                 rclcpp::Time(fast_lio_odom_data_.last_msg->header.stamp))
                  .seconds())
       : 0.0));
  fast_lio_odom_data_.is_active = true;
  fast_lio_odom_data_.last_msg = msg;
}
/*//}*/

/* control_callback() //{ */
void EstimationManager::control_callback(
  const laser_msgs::msg::UavControlDiagnostics::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(control_data_.mtx);
  RCLCPP_DEBUG_THROTTLE(
    get_logger(), *get_clock(), 5000, "Received control message at time %.3f s, frequency: %.2f Hz",
    msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9,
    ((control_data_.last_msg != nullptr)
       ? (1.0 /
          (rclcpp::Time(msg->header.stamp) - rclcpp::Time(control_data_.last_msg->header.stamp))
            .seconds())
       : 0.0));

  last_control_msg_ = control_data_.last_msg;
  control_data_.last_msg = msg;
  control_data_.is_active = true;
  mekf_->set_mass(msg->estimated_mass);
}
/*//}*/

/* garmin_range_callback() //{ */
void EstimationManager::garmin_range_callback(const sensor_msgs::msg::Range::SharedPtr msg)
{
  if (!garmin_offset_calibrated_) {
    RCLCPP_DEBUG(get_logger(), "Calibrating Garmin sensor... %d/500", garmin_counter_);
    if (garmin_counter_ < 500) {
      garmin_counter_++;
      garmin_calibrated_offset_ += msg->range;
    }
    if (garmin_counter_ == 500) {
      garmin_calibrated_offset_ /= 500.0;
      RCLCPP_INFO(get_logger(), "Calibrated Garmin offset: %.3f m", garmin_calibrated_offset_);
      garmin_offset_calibrated_ = true;
    }
    return;
  }
  msg->range -= garmin_calibrated_offset_;

  std::lock_guard<std::mutex> lock(garmin_data_.mtx);
  garmin_data_.is_active = true;
  RCLCPP_DEBUG_THROTTLE(
    get_logger(), *get_clock(), 10000,
    "Received Garmin range message at time %.3f s, frequency: %.2f Hz",
    msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9,
    ((garmin_data_.last_msg != nullptr) ? (1.0 / (rclcpp::Time(msg->header.stamp) -
                                                  rclcpp::Time(garmin_data_.last_msg->header.stamp))
                                                   .seconds())
                                        : 0.0));
  garmin_data_.last_msg = msg;
}
/*//}*/

/* set_odometry_callback() //{ */
void EstimationManager::set_odometry_callback(
  const std::shared_ptr<laser_msgs::srv::SetString::Request> request,
  std::shared_ptr<laser_msgs::srv::SetString::Response> response)
{
  RCLCPP_INFO(get_logger(), "SetOdometry service called with request: %s", request->data.c_str());

  std::lock_guard<std::mutex> lock(mtx_);

  const std::string new_source = request->data;

  if (
    std::find(odometry_source_names_.begin(), odometry_source_names_.end(), new_source) ==
    odometry_source_names_.end()) {
    response->success = false;
    response->message = "Invalid odometry source: '" + new_source + "'. Valid sources are: ";
    for (const auto & name : odometry_source_names_) {
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

  SensorDataBuffer<nav_msgs::msg::Odometry> * selected_odom_data = nullptr;
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

  auto newest_msg_it = selected_odom_data->buffer.rbegin();
  rclcpp::Duration time_since_last_msg = this->get_clock()->now() - newest_msg_it->first;
  if (time_since_last_msg > selected_odom_data->timeout) {
    response->success = false;
    response->message = "Switch failed. Timeout on odometry '" + new_source +
                        "'. Last message received " +
                        std::to_string(time_since_last_msg.seconds()) + "s ago.";
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    return;
  }

  const nav_msgs::msg::Odometry & current_state = mekf_->get_odometry();
  const auto & new_odom_pose = newest_msg_it->second->pose.pose;

  Eigen::Vector3d current_position(
    current_state.pose.pose.position.x, current_state.pose.pose.position.y,
    current_state.pose.pose.position.z);
  Eigen::Vector3d new_odom_position(
    new_odom_pose.position.x, new_odom_pose.position.y, new_odom_pose.position.z);
  double distance = (current_position - new_odom_position).norm();

  Eigen::Quaterniond new_orientation(
    new_odom_pose.orientation.w, new_odom_pose.orientation.x, new_odom_pose.orientation.y,
    new_odom_pose.orientation.z);
  Eigen::Quaterniond current_orientation(
    current_state.pose.pose.orientation.w, current_state.pose.pose.orientation.x,
    current_state.pose.pose.orientation.y, current_state.pose.pose.orientation.z);

  Eigen::AngleAxisd angle_axis_diff(new_orientation * current_orientation.inverse());

  Eigen::Vector3d new_odom_linear_velocity(
    newest_msg_it->second->twist.twist.linear.x, newest_msg_it->second->twist.twist.linear.y,
    newest_msg_it->second->twist.twist.linear.z);
  Eigen::Vector3d current_linear_velocity(
    current_state.twist.twist.linear.x, current_state.twist.twist.linear.y,
    current_state.twist.twist.linear.z);
  double velocity_diff = (current_linear_velocity - new_odom_linear_velocity).norm();

  Eigen::Vector3d new_odom_angular_velocity(
    newest_msg_it->second->twist.twist.angular.x, newest_msg_it->second->twist.twist.angular.y,
    newest_msg_it->second->twist.twist.angular.z);
  Eigen::Vector3d current_angular_velocity(
    current_state.twist.twist.angular.x, current_state.twist.twist.angular.y,
    current_state.twist.twist.angular.z);
  double angular_velocity_diff = (current_angular_velocity - new_odom_angular_velocity).norm();

  if (
    (distance > odometry_switch_distance_threshold_) ||
    (angle_axis_diff.angle() > odometry_switch_angle_threshold_) ||
    (velocity_diff > odometry_switch_velocity_linear_threshold_) ||
    (angular_velocity_diff > odometry_switch_velocity_angular_threshold_)) {
    response->success = false;
    response->message =
      "Switch failed. Odometry '" + new_source +
      "' is too far from the current estimate. Diffs - Pos: " + std::to_string(distance) +
      " m, Angle: " + std::to_string(angle_axis_diff.angle()) +
      " rad, LinVel: " + std::to_string(velocity_diff) +
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

  enable_px4_odom_ = (new_source == "px4_api_odom");
  enable_openvins_odom_ = (new_source == "openvins_odom");
  enable_fast_lio_odom_ = (new_source == "fast_lio_odom");
  current_active_odometry_name_ = new_source;

  if (new_source == "px4_api_odom") {
    mekf_->set_measurement_noise_gains(px4_measurement_noise_gains_);
  } else if (new_source == "openvins_odom") {
    mekf_->set_measurement_noise_gains(openvins_measurement_noise_gains_);
  } else if (new_source == "fast_lio_odom") {
    mekf_->set_measurement_noise_gains(fast_lio_measurement_noise_gains_);
  }

  response->success = true;
  response->message = "Odometry source switched to: " + current_active_odometry_name_;
  RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
}
/*//}*/

/**
 * @brief Thread-safe buffer integrity check considering delays and timeouts.
 */
/* is_buffer_valid() //{ */
template <typename MsgT>
bool EstimationManager::is_buffer_valid(
  SensorDataBuffer<MsgT> & sensor_buffer, const std::string & sensor_name,
  const rclcpp::Time & reference_time, rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock)
{
  std::lock_guard<std::mutex> lock(sensor_buffer.mtx);

  if (!sensor_buffer.last_msg) {
    RCLCPP_DEBUG(logger, "[%s]: Nenhuma mensagem recebida ainda.", sensor_name.c_str());
    return false;
  }

  if (
    sensor_buffer.last_msg->header.stamp.sec == 0 &&
    sensor_buffer.last_msg->header.stamp.nanosec == 0) {
    RCLCPP_WARN(
      logger, "[%s]: Mensagem com timestamp zerado (0) detectada. Ignorando.", sensor_name.c_str());
    return false;
  }

  const rclcpp::Time msg_time(sensor_buffer.last_msg->header.stamp);
  rclcpp::Duration age = reference_time - msg_time;

  if (age > sensor_buffer.timeout) {
    RCLCPP_WARN(
      logger,
      "[%s]: Timeout detectado! Mensagem tem %.3f s de atraso. Limite max (timeout) eh %.3f s.",
      sensor_name.c_str(), age.seconds(), sensor_buffer.timeout.seconds());
    sensor_buffer.is_active = false;
    return false;
  }
  if (age > sensor_buffer.tolerance) {
    RCLCPP_WARN(
      logger,
      "[%s]: Timeout detectado! Mensagem tem %.3f s de atraso. Limite max (timeout) eh %.3f s.",
      sensor_name.c_str(), age.seconds(), sensor_buffer.timeout.seconds());
    return false;
  }

  return true;
}
/*//}*/

/**
 * @brief High-frequency loop evaluating measurements and applying
 * EKF prediction/correction steps.
 */
/* timer_callback() //{ */
void EstimationManager::timer_callback()
{
  RCLCPP_DEBUG(
    get_logger(),
    "\n_________________________________________________________________________________________");
  RCLCPP_DEBUG(get_logger(), "Timer callback()");
  try {
    if (!is_active_)
      return;
    if (!is_initialized_) {
      RCLCPP_INFO(get_logger(), "Initializing EKF...");
      is_initialized_ = true;
      return;
    }

    rclcpp::Time reference_time;

    reference_time = this->get_clock()->now();

    RCLCPP_DEBUG(get_logger(), "Timer: %.3f s", reference_time.seconds());

    auto px4_odom_msg = px4_odom_data_.last_msg
                          ? std::optional<nav_msgs::msg::Odometry>(*px4_odom_data_.last_msg)
                          : std::nullopt;
    auto openvins_odom_msg =
      openvins_odom_data_.last_msg
        ? std::optional<nav_msgs::msg::Odometry>(*openvins_odom_data_.last_msg)
        : std::nullopt;
    auto fast_lio_odom_msg =
      fast_lio_odom_data_.last_msg
        ? std::optional<nav_msgs::msg::Odometry>(*fast_lio_odom_data_.last_msg)
        : std::nullopt;
    auto garmin_range_msg = garmin_data_.last_msg
                              ? std::optional<sensor_msgs::msg::Range>(*garmin_data_.last_msg)
                              : std::nullopt;
    auto control_msg = last_control_msg_
                         ? std::optional<laser_msgs::msg::UavControlDiagnostics>(*last_control_msg_)
                         : std::nullopt;

    if (!is_buffer_valid(px4_odom_data_, "PX4_Odom", reference_time, get_logger(), get_clock()))
      px4_odom_msg = std::nullopt;
    if (!is_buffer_valid(
          openvins_odom_data_, "OpenVINS_Odom", reference_time, get_logger(), get_clock()))
      openvins_odom_msg = std::nullopt;
    if (!is_buffer_valid(
          fast_lio_odom_data_, "FastLIO_Odom", reference_time, get_logger(), get_clock()))
      fast_lio_odom_msg = std::nullopt;
    if (!is_buffer_valid(garmin_data_, "Garmin", reference_time, get_logger(), get_clock()))
      garmin_range_msg = std::nullopt;
    if (!is_buffer_valid(control_data_, "Control", reference_time, get_logger(), get_clock()))
      control_msg = std::nullopt;

    auto stamp_to_seconds = [](const auto & msg) {
      return static_cast<double>(msg.header.stamp.sec) +
             static_cast<double>(msg.header.stamp.nanosec) * 1e-9;
    };

    RCLCPP_DEBUG(
      get_logger(), "PX4 odometry message: %s, time: %.3f s", px4_odom_msg ? "YES" : "NO",
      px4_odom_msg ? stamp_to_seconds(px4_odom_msg.value()) : 0.0);
    RCLCPP_DEBUG(
      get_logger(), "OpenVINS odometry message: %s, time: %.3f s", openvins_odom_msg ? "YES" : "NO",
      openvins_odom_msg ? stamp_to_seconds(openvins_odom_msg.value()) : 0.0);
    RCLCPP_DEBUG(
      get_logger(), "Fast-LIO odometry message: %s, time: %.3f s", fast_lio_odom_msg ? "YES" : "NO",
      fast_lio_odom_msg ? stamp_to_seconds(fast_lio_odom_msg.value()) : 0.0);
    RCLCPP_DEBUG(
      get_logger(), "Garmin range message: %s, time: %.3f s", garmin_range_msg ? "YES" : "NO",
      garmin_range_msg ? stamp_to_seconds(garmin_range_msg.value()) : 0.0);
    RCLCPP_DEBUG(
      get_logger(), "Control message: %s, time: %.3f s", control_msg ? "YES" : "NO",
      control_msg ? stamp_to_seconds(control_msg.value()) : 0.0);

    bool has_prediction{false};
    const double MAX_CONTROL_VALUE = 1.0e2;

    if (enable_px4_odom_ && !px4_odom_data_.is_active) {
      if (!px4_odom_data_.is_active)
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "PX4 odometry input is inactive.");
      if (!is_ekf_active_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 10000,
          "EKF is active but no valid measurement inputs are available.");
        return;
      }
    } else {
      if (enable_px4_odom_) {
        RCLCPP_INFO_ONCE(get_logger(), "PX4 odometry input is active and enabled.");
      }
    }

    if (enable_fast_lio_odom_ && !fast_lio_odom_data_.is_active) {
      if (!fast_lio_odom_data_.is_active)
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 10000, "Fast-LIO odometry input is inactive.");
      if (!is_ekf_active_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 10000,
          "EKF is active but no valid measurement inputs are available.");
        return;
      }
    } else {
      if (enable_fast_lio_odom_) {
        RCLCPP_INFO_ONCE(get_logger(), "Fast-LIO odometry input is active and enabled.");
      }
    }

    if (enable_openvins_odom_ && !openvins_odom_data_.is_active) {
      if (!openvins_odom_data_.is_active)
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 10000, "OpenVINS odometry input is inactive.");
      if (!is_ekf_active_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 10000,
          "EKF is active but no valid measurement inputs are available.");
      }
    } else {
      if (enable_openvins_odom_) {
        RCLCPP_INFO_ONCE(get_logger(), "OpenVINS odometry input is active and enabled.");
      }
    }

    RCLCPP_INFO_ONCE(get_logger(), "Starting EKF updates.");

    bool has_measurement{false};
    laser_uav_estimators::MeasurementPackage measurement;

    if (px4_odom_msg && enable_px4_odom_) {
      has_measurement = true;
      measurement.odometry = &(*px4_odom_msg);
      last_update_time_ = rclcpp::Time(px4_odom_msg->header.stamp);
    } else if (openvins_odom_msg && enable_openvins_odom_) {
      has_measurement = true;
      measurement.odometry = &(*openvins_odom_msg);
      last_update_time_ = rclcpp::Time(openvins_odom_msg->header.stamp);
    } else if (fast_lio_odom_msg && enable_fast_lio_odom_) {
      has_measurement = true;
      measurement.odometry = &(*fast_lio_odom_msg);
      last_update_time_ = rclcpp::Time(fast_lio_odom_msg->header.stamp);
    }

    if (garmin_range_msg) {
      measurement.garmin = &(*garmin_range_msg);
    }

    RCLCPP_DEBUG(
      get_logger(), "Prediction and Correction - control_msg: %s, update_msg: %s",
      control_msg ? "YES" : "NO", has_measurement ? "YES" : "NO");

    if (control_msg && has_measurement) {
      RCLCPP_INFO_ONCE(get_logger(), "Running Prediction with Control Manager Thrust.");

      if (!is_first_control_msg_) {
        last_control_input_time_ = rclcpp::Time(control_msg->header.stamp);
        is_first_control_msg_ = true;
        return;
      } else {
        rclcpp::Time current_time = last_update_time_;
        rclcpp::Time control_input_time = rclcpp::Time(control_msg->header.stamp);
        double dt_last_time = (last_update_time_ - control_input_time).seconds();

        last_control_input_time_ = current_time;

        RCLCPP_DEBUG(get_logger(), "Current time: %.3f s", current_time.seconds());
        RCLCPP_DEBUG(
          get_logger(), "Last control input time: %.3f s", last_control_input_time_.seconds());
        RCLCPP_DEBUG(get_logger(), "Update Time: %.3f s", last_update_time_.seconds());
        RCLCPP_DEBUG(get_logger(), "Time since last update: %.3f s", dt_last_time);

        if (measurement.garmin != nullptr) {
          double dt_garmin_time =
            (last_update_time_ - rclcpp::Time(measurement.garmin->header.stamp)).seconds();
          RCLCPP_DEBUG(get_logger(), "Time since last Garmin measurement: %.3f s", dt_garmin_time);
        }

        bool can_predict = true;
        if (dt_last_time < 0 || dt_last_time > 1.0) {
          can_predict = false;
        }

        if (
          can_predict && control_msg->last_control_input.data.size() !=
                           static_cast<size_t>(allocation_matrix_.cols())) {
          control_msg->last_control_input.data.resize(allocation_matrix_.cols());
          RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Control input size does not match number of motors (%ld). Resizing input vector.",
            allocation_matrix_.cols());
        }

        if (can_predict) {
          Eigen::Map<const Eigen::Vector4d> control_input(
            control_msg->last_control_input.data.data());

          if (!control_input.allFinite()) {
            RCLCPP_ERROR(get_logger(), "Control input contains non-finite values (inf or NaN).");
            can_predict = false;
          } else if ((control_input.array() < 0).any()) {
            RCLCPP_ERROR(
              get_logger(),
              "Control input contains negative values. Inputs: [%.2f, %.2f, %.2f, %.2f]",
              control_input[0], control_input[1], control_input[2], control_input[3]);
            can_predict = false;
          } else if ((control_input.array() > MAX_CONTROL_VALUE).any()) {
            RCLCPP_ERROR(
              get_logger(),
              "Control input contains excessively large values. Inputs: [%.2f, %.2f, %.2f, %.2f]",
              control_input[0], control_input[1], control_input[2], control_input[3]);
            can_predict = false;
          }

          if (can_predict) {
            mekf_->predict(control_input, dt_last_time);
            rclcpp::Time stamp = rclcpp::Time(control_msg->header.stamp);
            has_prediction = true;
            is_predicted_ = true;
          }

          if (is_first_control_msg_) {
            if (has_measurement) {
              mekf_->correct(measurement);
              if (measurement.odometry != nullptr) {
                last_update_time_ = rclcpp::Time(measurement.odometry->header.stamp);
              }
            }
          }
        }
      }
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Running Prediction with Hover Thrust Setpoint. Waiting for Control Input From Control "
        "Manager.");

      const Eigen::Vector4d control_input(
        Eigen::Vector4d::Constant((9.81 * mekf_->get_mass()) / 4));

      mekf_->predict(control_input, 0.01);

      if (has_measurement) {
        mekf_->correct(measurement);
        if (measurement.odometry != nullptr) {
          last_update_time_ = rclcpp::Time(measurement.odometry->header.stamp);
        }
      }

      rclcpp::Time stamp = this->get_clock()->now();
      has_prediction = true;
      is_predicted_ = true;
    }

    if ((has_prediction || has_measurement) && !enable_openvins_odom_) {
      publish_odometry(odom_pub_, last_update_time_);
      is_ekf_active_ = true;
    } else if (enable_openvins_odom_ && !openvins_odom_data_.last_msg) {
      if (px4_odom_data_.last_msg) {
        auto msg = px4_odom_data_.last_msg;
        msg->header.stamp = this->get_clock()->now();
        odom_pub_->publish(*msg);
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "OpenVins waiting for data... using PX4 fallback.");
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Error in timer_callback: %s", e.what());
  }
}
/*//}*/

/* diagnostics_timer_callback() //{ */
void EstimationManager::diagnostics_timer_callback()
{
  if (!is_active_)
    return;
  try {
    auto diag_msg = std::make_unique<laser_msgs::msg::EstimationManagerDiagnostics>();
    diag_msg->header.stamp = this->get_clock()->now();

    if (
      current_active_odometry_name_ == "px4_api_odom" && px4_odom_data_.is_active &&
      !px4_odom_data_.buffer.empty())
      diag_msg->header.frame_id = px4_odom_data_.buffer.begin()->second->header.frame_id;
    else if (
      current_active_odometry_name_ == "openvins_odom" && openvins_odom_data_.is_active &&
      !openvins_odom_data_.buffer.empty())
      diag_msg->header.frame_id = openvins_odom_data_.buffer.begin()->second->header.frame_id;
    else if (
      current_active_odometry_name_ == "fast_lio_odom" && fast_lio_odom_data_.is_active &&
      !fast_lio_odom_data_.buffer.empty())
      diag_msg->header.frame_id = fast_lio_odom_data_.buffer.begin()->second->header.frame_id;

    std::lock_guard<std::mutex> lock(mtx_);

    diag_msg->active_odometry_source = current_active_odometry_name_;
    diag_msg->is_initialized = is_initialized_;

    auto fill_sensor_status =
      [&](laser_msgs::msg::SensorStatus & status, auto & sensor_data, const std::string & name) {
        std::lock_guard<std::mutex> lock(sensor_data.mtx);
        status.name = name;
        status.is_active = sensor_data.is_active;
        status.buffer_size = sensor_data.buffer.size();

        if (sensor_data.last_msg) {
          status.time_since_last_message =
            (this->get_clock()->now() - rclcpp::Time(sensor_data.last_msg->header.stamp)).seconds();
          status.has_timeout =
            (this->get_clock()->now() - rclcpp::Time(sensor_data.last_msg->header.stamp)) >
            sensor_data.timeout;
          status.last_message_stamp = sensor_data.last_msg->header.stamp;
        } else {
          status.has_timeout = true;
          status.time_since_last_message = -1.0;
        }

        if (name == current_active_odometry_name_) {
          if (!sensor_data.last_msg) {
            RCLCPP_WARN_THROTTLE(
              this->get_logger(), *this->get_clock(), 5000,
              "Active odometry sensor ('%s') has not started yet (no message received).",
              name.c_str());
          } else {
            if (
              (this->get_clock()->now() - rclcpp::Time(sensor_data.last_msg->header.stamp)) >
              sensor_data.timeout) {
              RCLCPP_ERROR_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "Timeout! Active odometry sensor ('%s') stopped publishing. (Last msg: %.2f s ago)",
                name.c_str(),
                (this->get_clock()->now() - rclcpp::Time(sensor_data.last_msg->header.stamp))
                  .seconds());
            } else {
              RCLCPP_INFO_ONCE(
                this->get_logger(),
                "Active odometry sensor ('%s') publishing. (Last msg: %.2f s ago)", name.c_str(),
                (this->get_clock()->now() - rclcpp::Time(sensor_data.last_msg->header.stamp))
                  .seconds());
            }
          }
        }
      };

    fill_sensor_status(diag_msg->odometry_sources.emplace_back(), px4_odom_data_, "px4_api_odom");
    fill_sensor_status(
      diag_msg->odometry_sources.emplace_back(), openvins_odom_data_, "openvins_odom");
    fill_sensor_status(
      diag_msg->odometry_sources.emplace_back(), fast_lio_odom_data_, "fast_lio_odom");

    diagnostics_pub_->publish(std::move(diag_msg));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Error publishing diagnostics: %s", e.what());
  }
}
/*//}*/

/* publish_odometry() //{ */
void EstimationManager::publish_odometry(
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr pub,
  rclcpp::Time & pub_time)
{
  const nav_msgs::msg::Odometry & state = mekf_->get_odometry();

  nav_msgs::msg::Odometry odom_out_msg = state;
  odom_out_msg.header.stamp = pub_time;
  odom_out_msg.header.frame_id = uav_name_ + "/odometry";
  odom_out_msg.child_frame_id = uav_name_ + "/fcu";
  pub->publish(odom_out_msg);

  try {
    Eigen::Quaterniond q_odom(
      state.pose.pose.orientation.w, state.pose.pose.orientation.x, state.pose.pose.orientation.y,
      state.pose.pose.orientation.z);

    tf2::Transform tf_direct;
    tf_direct.setOrigin(tf2::Vector3(
      state.pose.pose.position.x, state.pose.pose.position.y, state.pose.pose.position.z));
    tf_direct.setRotation(tf2::Quaternion(q_odom.x(), q_odom.y(), q_odom.z(), q_odom.w()));

    tf2::Transform tf_inv = tf_direct.inverse();

    geometry_msgs::msg::TransformStamped dynamic_tf;
    dynamic_tf.header.stamp = pub_time;
    dynamic_tf.header.frame_id = uav_name_ + "/fcu";
    dynamic_tf.child_frame_id = uav_name_ + "/odometry";
    dynamic_tf.transform = tf2::toMsg(tf_inv);

    tf_broadcaster_->sendTransform(dynamic_tf);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "Error on TF: %s", ex.what());
  }
}
/*//}*/
}  // namespace laser_uav_managers

RCLCPP_COMPONENTS_REGISTER_NODE(laser_uav_managers::EstimationManager)
