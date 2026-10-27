#include <laser_uav_managers/planning_manager_node.hpp>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>

namespace laser_uav_managers
{
namespace
{
geometry_msgs::msg::Quaternion yaw_quaternion(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.w = std::cos(0.5 * yaw);
  q.z = std::sin(0.5 * yaw);
  return q;
}

double yaw_of(const geometry_msgs::msg::Quaternion & q)
{
  const auto direction =
    Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized() * Eigen::Vector3d::UnitX();
  return std::atan2(direction.y(), direction.x());
}

double distance(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
{
  return std::hypot(std::hypot(a.x - b.x, a.y - b.y), a.z - b.z);
}

bool finite_state(const laser_msgs::msg::ReferenceState & r)
{
  const auto & p = r.pose.position;
  const auto & q = r.pose.orientation;
  const auto & v = r.twist.linear;
  const auto & w = r.twist.angular;
  for (double x : {p.x, p.y, p.z, q.w, q.x, q.y, q.z, v.x, v.y, v.z, w.x, w.y, w.z}) {
    if (!std::isfinite(x)) {
      return false;
    }
  }
  const double norm = Eigen::Quaterniond(q.w, q.x, q.y, q.z).norm();
  return std::isfinite(norm) && norm > 1e-9;
}

laser_msgs::msg::ReferenceState hold_at(const laser_msgs::msg::ReferenceState & start)
{
  auto ref = start;
  ref.twist = geometry_msgs::msg::Twist();
  ref.pose.orientation = yaw_quaternion(yaw_of(ref.pose.orientation));
  ref.use_position = ref.use_orientation = ref.use_linear_velocity = ref.use_angular_velocity =
    true;
  ref.use_individual_thrust = false;
  return ref;
}

double duration(const std::vector<double> & times)
{
  return std::accumulate(times.begin(), times.end(), 0.0);
}

bool valid_pieces(
  uint32_t pieces, uint32_t order, const std::vector<double> & times,
  const std::vector<double> & coefficients)
{
  return pieces > 0 && pieces <= 1024 && order <= 12 && times.size() == pieces &&
         coefficients.size() == static_cast<std::size_t>(pieces) * (order + 1) &&
         std::isfinite(duration(times)) &&
         std::all_of(
    times.begin(), times.end(),
    [](double t) {return std::isfinite(t) && t > 0;}) &&
         std::all_of(
    coefficients.begin(), coefficients.end(), [](double c) {return std::isfinite(c);});
}

std::pair<std::size_t, double> piece_at(const std::vector<double> & times, double time)
{
  double t = std::max(0.0, time);
  for (std::size_t i = 0; i < times.size(); ++i) {
    if (t <= times[i]) {
      return {i, t};
    }
    t -= times[i];
  }
  return {times.size() - 1, times.back()};
}

std::pair<double, double> evaluate(
  const std::vector<double> & coefficients, uint32_t order, std::size_t piece, double time)
{
  const std::size_t offset = piece * (order + 1);
  double value = coefficients[offset];
  double derivative = 0.0;
  for (uint32_t i = 1; i <= order; ++i) {
    derivative = derivative * time + value;
    value = value * time + coefficients[offset + i];
  }
  return {value, derivative};
}
}  // namespace

bool PlanningManagerNode::store_super_trajectory(const Polynomial & msg, std::string & reason)
{
  if ((msg.type & Polynomial::POSITION_TRAJ) == 0 || !std::isfinite(msg.start_wt_pos) ||
    !valid_pieces(msg.piece_num_pos, msg.order_pos, msg.time_pos, msg.coef_pos_x) ||
    !valid_pieces(msg.piece_num_pos, msg.order_pos, msg.time_pos, msg.coef_pos_y) ||
    !valid_pieces(msg.piece_num_pos, msg.order_pos, msg.time_pos, msg.coef_pos_z) ||
    !std::isfinite(msg.start_wt_pos + duration(msg.time_pos)))
  {
    reason = "invalid SUPER position polynomial";
    return false;
  }
  if ((msg.type & Polynomial::YAW_TRAJ) != 0 &&
    (!std::isfinite(msg.start_wt_yaw) ||
    !valid_pieces(msg.piece_num_yaw, msg.order_yaw, msg.time_yaw, msg.coef_yaw) ||
    !std::isfinite(msg.start_wt_yaw + duration(msg.time_yaw))))
  {
    reason = "invalid SUPER yaw polynomial";
    return false;
  }
  if ((msg.type & Polynomial::YAW_COMMAND) != 0 &&
    (!std::isfinite(msg.yaw) || !std::isfinite(msg.yaw_rate)))
  {
    reason = "invalid SUPER yaw command";
    return false;
  }
  super_trajectory_ = msg;
  have_super_trajectory_ = true;
  return true;
}

bool PlanningManagerNode::matches_super_trajectory(const Polynomial & heartbeat) const
{
  return have_super_trajectory_ &&
         std::isfinite(heartbeat.start_wt_pos) &&
         std::abs(heartbeat.start_wt_pos - super_trajectory_.start_wt_pos) <= 1e-6;
}

double PlanningManagerNode::super_end_time() const
{
  if (!have_super_trajectory_) {
    return 0.0;
  }
  double end = super_trajectory_.start_wt_pos + duration(super_trajectory_.time_pos);
  if ((super_trajectory_.type & Polynomial::YAW_TRAJ) != 0) {
    end = std::max(end, super_trajectory_.start_wt_yaw + duration(super_trajectory_.time_yaw));
  }
  return end;
}

bool PlanningManagerNode::sample_super_trajectory(
  double now, int count, double dt, double fallback_yaw,
  std::vector<laser_msgs::msg::ReferenceState> & horizon, bool & terminal) const
{
  if (!have_super_trajectory_ || count <= 0 || !std::isfinite(now) || !std::isfinite(dt) ||
    dt <= 0)
  {
    return false;
  }
  terminal = now >= super_end_time();
  const auto & traj = super_trajectory_;
  horizon.clear();
  horizon.reserve(static_cast<std::size_t>(count));
  for (int i = 0; i < count; ++i) {
    const double time = now + i * dt;
    const double t = time - traj.start_wt_pos;
    const auto piece = piece_at(traj.time_pos, t);
    const auto x = evaluate(traj.coef_pos_x, traj.order_pos, piece.first, piece.second);
    const auto y = evaluate(traj.coef_pos_y, traj.order_pos, piece.first, piece.second);
    const auto z = evaluate(traj.coef_pos_z, traj.order_pos, piece.first, piece.second);
    laser_msgs::msg::ReferenceState ref;
    ref.pose.position.x = x.first;
    ref.pose.position.y = y.first;
    ref.pose.position.z = z.first;
    if (t >= 0 && t < duration(traj.time_pos)) {
      ref.twist.linear.x = x.second;
      ref.twist.linear.y = y.second;
      ref.twist.linear.z = z.second;
    }
    double yaw = fallback_yaw;
    if ((traj.type & Polynomial::YAW_TRAJ) != 0) {
      const double ty = time - traj.start_wt_yaw;
      const auto py = piece_at(traj.time_yaw, ty);
      const auto result = evaluate(traj.coef_yaw, traj.order_yaw, py.first, py.second);
      yaw = result.first;
      if (ty >= 0 && ty < duration(traj.time_yaw)) {
        ref.twist.angular.z = result.second;
      }
    } else if ((traj.type & Polynomial::YAW_COMMAND) != 0) {
      yaw = traj.yaw;
      ref.twist.angular.z = time >= super_end_time() ? 0.0 : traj.yaw_rate;
    }
    ref.pose.orientation = yaw_quaternion(yaw);
    ref.use_position = ref.use_orientation = true;
    ref.use_linear_velocity = ref.use_angular_velocity = true;
    if (!std::isfinite(x.first) || !std::isfinite(y.first) || !std::isfinite(z.first) ||
      !std::isfinite(ref.twist.linear.x) || !std::isfinite(ref.twist.linear.y) ||
      !std::isfinite(ref.twist.linear.z) || !std::isfinite(yaw) ||
      !std::isfinite(ref.twist.angular.z))
    {
      horizon.clear();
      return false;
    }
    horizon.push_back(ref);
  }
  return true;
}
PlanningManagerNode::PlanningManagerNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("planning_manager", "", options)
{
  rcl_interfaces::msg::ParameterDescriptor setup_only;
  setup_only.read_only = true;
  declare_parameter<std::string>("planner", "pmm", setup_only);
  declare_parameter("rate.reference_horizon", 100.0);
  declare_parameter("control_state_timeout", 0.5);
  declare_parameter("planning_timeout", 5.0);
  declare_parameter("super.timeout", 0.5);
  declare_parameter("super.sampling_step", 0.01);
  declare_parameter("goal.position_tolerance", 0.15);
  declare_parameter("goal.heading_tolerance", 0.15);
  declare_parameter("waypoint_hold_time", 3.0);
  declare_parameter("multirotor_parameters.mass", 0.0);
  declare_parameter("multirotor_parameters.n_motors", 0);
  declare_parameter("multirotor_parameters.G1", std::vector<double>{});
  declare_parameter("multirotor_parameters.inertia", std::vector<double>{});
  declare_parameter("multirotor_parameters.omega_max", std::vector<double>{});
  for (const auto & entry : std::vector<std::pair<std::string, double>>{
      {"multirotor_parameters.max_accel", 22.0}, {"multirotor_parameters.max_vel", 16.0},
      {"multirotor_parameters.default_vel", 1.0}, {"ltd_opt.thrust_decomp_acc_precision", 0.01},
      {"first_vel_opt.alpha", 10.0}, {"first_vel_opt.alpha_reduction_factor", 0.2},
      {"first_vel_opt.alpha_min_threshold", 0.001}, {"second_vel_opt.alpha", 35.0},
      {"second_vel_opt.alpha_reduction_factor", 0.1}, {"second_vel_opt.alpha_min_threshold", 0.01},
      {"time.dt_precision", 0.001}, {"time.sampling_step", 0.01}})
  {
    declare_parameter("agile_planner." + entry.first, entry.second);
  }
  declare_parameter("agile_planner.ltd_opt.use_drag", false);
  declare_parameter("agile_planner.ltd_opt.thrust_decomp_max_iter", 20);
  declare_parameter("agile_planner.first_vel_opt.max_iter", 30);
  declare_parameter("agile_planner.second_vel_opt.run", true);
  declare_parameter("agile_planner.second_vel_opt.max_iter", 10);
}

void PlanningManagerNode::read_parameters()
{
  get_parameter("planner", planner_);
  get_parameter("rate.reference_horizon", publish_rate_);
  get_parameter("control_state_timeout", state_timeout_);
  get_parameter("planning_timeout", planning_timeout_);
  get_parameter("super.timeout", super_timeout_);
  get_parameter("super.sampling_step", super_sampling_step_);
  get_parameter("goal.position_tolerance", position_tolerance_);
  get_parameter("goal.heading_tolerance", heading_tolerance_);
  get_parameter("waypoint_hold_time", waypoint_hold_time_);
  get_parameter("multirotor_parameters.mass", physical_.mass);
  get_parameter("multirotor_parameters.n_motors", physical_.n_motors);
  const auto allocation = get_parameter("multirotor_parameters.G1").as_double_array();
  const auto inertia = get_parameter("multirotor_parameters.inertia").as_double_array();
  const auto rates = get_parameter("multirotor_parameters.omega_max").as_double_array();
  if ((planner_ != "pmm" && planner_ != "super") || physical_.mass <= 0 ||
    physical_.n_motors <= 0 ||
    allocation.size() != static_cast<std::size_t>(4 * physical_.n_motors) ||
    inertia.size() != 3 || rates.size() != 3)
  {
    throw std::invalid_argument("invalid planner or physical parameters");
  }
  physical_.G1 = Eigen::Map<const Eigen::Matrix<double, 4, Eigen::Dynamic, Eigen::RowMajor>>(
    allocation.data(), 4, physical_.n_motors);
  physical_.inertia_matrix = Eigen::Map<const Eigen::Vector3d>(inertia.data()).asDiagonal();
  yaw_rate_limit_ = rates[2];
  for (double value : {publish_rate_, state_timeout_, planning_timeout_, super_timeout_,
      super_sampling_step_, position_tolerance_, heading_tolerance_, yaw_rate_limit_,
      physical_.mass})
  {
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument("rates, timeouts, tolerances and mass must be positive and finite");
    }
  }
  if (!std::isfinite(waypoint_hold_time_) || waypoint_hold_time_ < 0.0 ||
    !physical_.G1.allFinite() || !physical_.inertia_matrix.allFinite() ||
    physical_.inertia_matrix.diagonal().minCoeff() <= 0.0)
  {
    throw std::invalid_argument("invalid physical matrix or waypoint hold time");
  }
  if (planner_ == "pmm" &&
    (physical_.n_motors != 4 || std::abs(physical_.G1.determinant()) < 1e-12))
  {
    throw std::invalid_argument("PMM requires an invertible four-motor allocation");
  }
  get_parameter("agile_planner.multirotor_parameters.max_accel", pmm_parameters_.max_accel_norm);
  get_parameter("agile_planner.multirotor_parameters.max_vel", pmm_parameters_.max_vel_norm);
  get_parameter(
    "agile_planner.multirotor_parameters.default_vel",
    pmm_parameters_.default_vel_norm);
  get_parameter("agile_planner.ltd_opt.use_drag", pmm_parameters_.use_drag);
  get_parameter(
    "agile_planner.ltd_opt.thrust_decomp_acc_precision",
    pmm_parameters_.thrust_decomp_acc_precision);
  get_parameter(
    "agile_planner.ltd_opt.thrust_decomp_max_iter",
    pmm_parameters_.thrust_decomp_max_iter);
  get_parameter("agile_planner.first_vel_opt.alpha", pmm_parameters_.first_run_alpha);
  get_parameter(
    "agile_planner.first_vel_opt.alpha_reduction_factor",
    pmm_parameters_.first_run_alpha_reduction_factor);
  get_parameter(
    "agile_planner.first_vel_opt.alpha_min_threshold",
    pmm_parameters_.first_run_alpha_min_threshold);
  get_parameter("agile_planner.first_vel_opt.max_iter", pmm_parameters_.first_run_max_iter);
  get_parameter("agile_planner.second_vel_opt.run", pmm_parameters_.run_second_opt);
  get_parameter("agile_planner.second_vel_opt.alpha", pmm_parameters_.second_run_alpha);
  get_parameter(
    "agile_planner.second_vel_opt.alpha_reduction_factor",
    pmm_parameters_.second_run_alpha_reduction_factor);
  get_parameter(
    "agile_planner.second_vel_opt.alpha_min_threshold",
    pmm_parameters_.second_run_alpha_min_threshold);
  get_parameter("agile_planner.second_vel_opt.max_iter", pmm_parameters_.second_run_max_iter);
  get_parameter("agile_planner.time.dt_precision", pmm_parameters_.dt_precision);
  get_parameter("agile_planner.time.sampling_step", pmm_parameters_.sampling_step);
  for (double value : {pmm_parameters_.max_accel_norm, pmm_parameters_.max_vel_norm,
      pmm_parameters_.default_vel_norm, pmm_parameters_.sampling_step, pmm_parameters_.dt_precision,
      pmm_parameters_.thrust_decomp_acc_precision, pmm_parameters_.first_run_alpha,
      pmm_parameters_.first_run_alpha_min_threshold, pmm_parameters_.second_run_alpha,
      pmm_parameters_.second_run_alpha_min_threshold})
  {
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument("PMM limits and sampling parameters must be positive and finite");
    }
  }
  if (pmm_parameters_.max_accel_norm <= 9.81 || pmm_parameters_.thrust_decomp_max_iter <= 0 ||
    pmm_parameters_.first_run_max_iter <= 0 || pmm_parameters_.second_run_max_iter <= 0 ||
    !(pmm_parameters_.first_run_alpha_reduction_factor > 0.0 &&
    pmm_parameters_.first_run_alpha_reduction_factor < 1.0) ||
    !(pmm_parameters_.second_run_alpha_reduction_factor > 0.0 &&
    pmm_parameters_.second_run_alpha_reduction_factor < 1.0))
  {
    throw std::invalid_argument("invalid PMM optimization limits");
  }
}

PlanningManagerNode::CallbackReturn PlanningManagerNode::on_configure(
  const rclcpp_lifecycle::State &)
{
  try {
    read_parameters();
    const auto stream_qos = rclcpp::QoS(1).best_effort();
    const auto state_qos = rclcpp::QoS(1).reliable().transient_local();
    pub_horizon_ = create_publisher<laser_msgs::msg::ReferenceHorizon>(
      "reference_horizon_out",
      stream_qos);
    pub_status_ = create_publisher<Status>("diagnostics_out", state_qos);
    pub_super_goal_ = create_publisher<geometry_msgs::msg::PoseStamped>("super_goal_out", 1);
    sub_control_state_ = create_subscription<ControlState>(
      "control_state_in", state_qos,
      std::bind(&PlanningManagerNode::control_state_callback, this, std::placeholders::_1));
    sub_odometry_ = create_subscription<nav_msgs::msg::Odometry>(
      "odometry_in", rclcpp::SensorDataQoS(),
      std::bind(&PlanningManagerNode::odometry_callback, this, std::placeholders::_1));
    sub_goto_ = create_subscription<Goal>(
      "goto_in", 1,
      std::bind(&PlanningManagerNode::goto_callback, this, std::placeholders::_1));
    sub_goto_relative_ = create_subscription<Goal>(
      "goto_relative_in", 1,
      std::bind(&PlanningManagerNode::goto_relative_callback, this, std::placeholders::_1));
    sub_path_ = create_subscription<laser_msgs::msg::TrajectoryPath>(
      "trajectory_path_in", 1,
      std::bind(&PlanningManagerNode::trajectory_path_callback, this, std::placeholders::_1));
    srv_cancel_ = create_service<std_srvs::srv::Trigger>(
      "cancel",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        response->success = active_;
        response->message = active_ ? "mission cancelled" : "planning manager is inactive";
        if (active_) {
          abort_mission("operator cancelled the mission");
        }
      });
    if (planner_ == "super") {
      sub_super_ = create_subscription<Polynomial>(
        "super_trajectory_in", stream_qos,
        std::bind(&PlanningManagerNode::super_trajectory_callback, this, std::placeholders::_1));
    }
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_),
      std::bind(&PlanningManagerNode::timer_callback, this));
    diagnostics_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&PlanningManagerNode::publish_status, this));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Planning configuration failed: %s", e.what());
    return CallbackReturn::FAILURE;
  }
  return CallbackReturn::SUCCESS;
}

PlanningManagerNode::CallbackReturn PlanningManagerNode::on_activate(
  const rclcpp_lifecycle::State &)
{
  pub_horizon_->on_activate();
  pub_status_->on_activate();
  pub_super_goal_->on_activate();
  active_ = true;
  publish_status();
  return CallbackReturn::SUCCESS;
}

PlanningManagerNode::CallbackReturn PlanningManagerNode::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  abort_mission("planning manager deactivated");
  active_ = false;
  pub_horizon_->on_deactivate();
  pub_status_->on_deactivate();
  pub_super_goal_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

PlanningManagerNode::CallbackReturn PlanningManagerNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  if (generation_.valid()) {
    if (generation_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
      RCLCPP_WARN(get_logger(), "PMM worker is still finishing; cleanup can be retried");
      return CallbackReturn::FAILURE;
    }
    poll_generation();
  }
  timer_.reset();
  diagnostics_timer_.reset();
  sub_control_state_.reset();
  sub_odometry_.reset();
  sub_goto_.reset();
  sub_goto_relative_.reset();
  sub_path_.reset();
  sub_super_.reset();
  srv_cancel_.reset();
  pub_horizon_.reset();
  pub_status_.reset();
  pub_super_goal_.reset();
  control_.reset();
  odometry_.reset();
  plan_.reset();
  have_super_trajectory_ = false;
  mission_ = Mission{};
  mission_state_ = Status::IDLE;
  return CallbackReturn::SUCCESS;
}

PlanningManagerNode::CallbackReturn PlanningManagerNode::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  if (active_) {
    abort_mission("planning manager shutdown");
  }
  active_ = false;
  return CallbackReturn::SUCCESS;
}

bool PlanningManagerNode::control_is_fresh() const
{
  return control_ && control_->ready_for_tracking && control_->flight_id != 0 &&
         SteadyClock::now() - control_rx_ < std::chrono::duration<double>(state_timeout_);
}

void PlanningManagerNode::control_state_callback(const ControlState & msg)
{
  if (!active_ || msg.horizon_length == 0 || msg.horizon_length > 512 ||
    !std::isfinite(msg.horizon_dt) || msg.horizon_dt <= 0 || msg.header.frame_id.empty() ||
    !std::isfinite(msg.estimated_mass) || msg.estimated_mass <= 0 || !finite_state(msg.reference))
  {
    return;
  }
  if (control_ && msg.flight_id != 0 && msg.flight_id < control_->flight_id) {
    return;
  }
  if (mission_.active && (msg.flight_id != mission_.flight_id || !msg.ready_for_tracking ||
    (msg.emergency_hold && msg.mission_id >= mission_.id)))
  {
    abort_mission("control state no longer permits this mission");
  }
  if (control_ && msg.flight_id != control_->flight_id) {
    plan_.reset();
    have_super_trajectory_ = false;
    plan_mission_id_ = 0;
  }
  control_ = msg;
  control_rx_ = SteadyClock::now();
}

void PlanningManagerNode::odometry_callback(const nav_msgs::msg::Odometry & msg)
{
  const auto & p = msg.pose.pose.position;
  const auto & q = msg.pose.pose.orientation;
  if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
    std::isfinite(q.w) && std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) &&
    Eigen::Quaterniond(q.w, q.x, q.y, q.z).norm() > 1e-9)
  {
    odometry_ = msg;
    odometry_rx_ = SteadyClock::now();
  }
}

void PlanningManagerNode::goto_callback(const Goal & msg)
{
  start_mission({msg}, 0.0, false);
}

void PlanningManagerNode::goto_relative_callback(const Goal & msg)
{
  if (!control_is_fresh()) {
    reject("control is not ready");
    return;
  }
  const auto & pose = control_->reference.pose;
  const Eigen::Vector3d offset = Eigen::Quaterniond(
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z).normalized() *
    Eigen::Vector3d(msg.position.x, msg.position.y, msg.position.z);
  Goal goal;
  goal.position.x = pose.position.x + offset.x();
  goal.position.y = pose.position.y + offset.y();
  goal.position.z = pose.position.z + offset.z();
  goal.heading = std::remainder(yaw_of(pose.orientation) + msg.heading, 2.0 * M_PI);
  start_mission({goal}, 0.0, false);
}

void PlanningManagerNode::trajectory_path_callback(const laser_msgs::msg::TrajectoryPath & msg)
{
  start_mission(msg.waypoints, msg.speed, msg.stop_on_waypoints);
}

void PlanningManagerNode::reject(const std::string & reason)
{
  RCLCPP_WARN(get_logger(), "Rejected planning request: %s", reason.c_str());
}

bool PlanningManagerNode::start_mission(const std::vector<Goal> & goals, double speed, bool stop)
{
  if (!active_ || !control_is_fresh() || !odometry_ ||
    SteadyClock::now() - odometry_rx_ > std::chrono::duration<double>(state_timeout_) ||
    odometry_->header.frame_id != control_->header.frame_id)
  {
    reject("fresh control state and odometry in the same frame are required");
    return false;
  }
  if (mission_.active || generation_.valid()) {
    reject("finish or cancel the current mission before submitting another");
    return false;
  }
  if (goals.empty() || goals.size() > 256 || !std::isfinite(speed) || speed < 0.0) {
    reject("invalid waypoint count or speed");
    return false;
  }
  for (std::size_t i = 0; i < goals.size(); ++i) {
    const auto & p = goals[i].position;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
      !std::isfinite(goals[i].heading) ||
      (!stop && i > 0 && distance(goals[i - 1].position, p) < 1e-8))
    {
      reject("non-finite or coincident fly-through waypoints");
      return false;
    }
  }
  if (planner_ == "super" && goals.size() > 1 && !stop) {
    reject("SUPER accepts one goal or a sequence with stops");
    return false;
  }
  if (planner_ == "super" && speed > 0.0) {
    reject("SUPER speed is configured at startup; use speed=0 in the mission");
    return false;
  }
  const auto & velocity = odometry_->twist.twist.linear;
  if (!std::isfinite(velocity.x) || !std::isfinite(velocity.y) || !std::isfinite(velocity.z) ||
    std::hypot(std::hypot(velocity.x, velocity.y), velocity.z) > 0.15)
  {
    reject("a new mission must start from hover");
    return false;
  }
  mission_.flight_id = control_->flight_id;
  next_mission_id_ = std::max(next_mission_id_, control_->mission_id) + 1;
  mission_.id = next_mission_id_;
  mission_.active = true;
  mission_.frame = control_->header.frame_id;
  mission_.goals = goals;
  mission_.index = 0;
  mission_.speed = speed;
  mission_.stop = stop;
  mission_.start = get_clock()->now();
  start_segment();
  return true;
}

void PlanningManagerNode::start_segment()
{
  dwelling_since_.reset();
  mission_state_ = Status::GENERATING;
  detail_.clear();
  generation_started_ = SteadyClock::now();
  if (planner_ == "super") {
    have_super_trajectory_ = false;
    super_hold_ = hold_at(control_->reference);
    plan_mission_id_ = mission_.id;
    plan_flight_id_ = mission_.flight_id;
    super_goal_time_ = get_clock()->now();
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = super_goal_time_;
    msg.header.frame_id = mission_.frame;
    msg.pose.position = mission_.goals[mission_.index].position;
    msg.pose.orientation = yaw_quaternion(mission_.goals[mission_.index].heading);
    pub_super_goal_->publish(msg);
  } else {
    auto physical = physical_;
    physical.mass = control_->estimated_mass;
    const auto parameters = pmm_parameters_;
    const auto start = hold_at(control_->reference);
    const auto goals = mission_.goals;
    const auto index = mission_.index;
    const auto stop = mission_.stop;
    const auto speed = mission_.speed;
    const auto yaw_limit = yaw_rate_limit_;
    generation_mission_id_ = mission_.id;
    generation_ = std::async(
      std::launch::async,
      [physical, parameters, start, goals, index, stop, speed, yaw_limit]() {
        const auto began = SteadyClock::now();
        GeneratedPlan result;
        const auto & goal = stop ? goals[index] : goals.back();
        if ((stop || goals.size() == 1) && distance(start.pose.position, goal.position) < 1e-8) {
          const double yaw = yaw_of(start.pose.orientation);
          const double delta = std::remainder(goal.heading - yaw, 2.0 * M_PI);
          const double duration =
          std::max(parameters.sampling_step, 1.875 * std::abs(delta) / yaw_limit);
          const int count = static_cast<int>(std::ceil(duration / parameters.sampling_step));
          for (int i = 0; i <= count; ++i) {
            const double t = std::min(i * parameters.sampling_step, duration);
            const double u = t / duration;
            const double blend = u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
            const double rate = 30.0 * u * u * (1.0 - u) * (1.0 - u) / duration;
            auto ref = start;
            ref.pose.orientation = yaw_quaternion(yaw + delta * blend);
            ref.twist.angular.z = delta * rate;
            result.times.push_back(t);
            result.states.push_back(ref);
          }
        } else {
          laser_uav_planners::AgilePlanner planner(physical, parameters);
          if (stop || goals.size() == 1) {
            planner.generateTrajectory(start, goal, static_cast<float>(speed), speed > 0.0);
          } else {
            planner.generateTrajectory(start, goals, static_cast<float>(speed));
          }
          const auto & times = planner.getTrajectoryTimes();
          const auto & states = planner.getTrajectory();
          if (times.size() != states.size()) {
            throw std::runtime_error("PMM returned mismatched time/state arrays");
          }
          for (std::size_t i = 0; i < times.size(); ++i) {
            if (!std::isfinite(times[i]) || times[i] < 0 ||
            (!result.times.empty() && times[i] < result.times.back() - 1e-9))
            {
              throw std::runtime_error("PMM returned invalid sample times");
            }
            if (!result.times.empty() && std::abs(times[i] - result.times.back()) <= 1e-9) {
              result.states.back() = states[i];
            } else {
              result.times.push_back(times[i]);
              result.states.push_back(states[i]);
            }
          }
        }
        if (result.times.size() < 2 || result.times.size() > 250000) {
          throw std::runtime_error("PMM returned an empty or excessively large trajectory");
        }
        if (!std::all_of(result.states.begin(), result.states.end(), finite_state)) {
          throw std::runtime_error("PMM returned a non-finite reference");
        }
        result.states.back().twist = geometry_msgs::msg::Twist();
        result.elapsed_ms = std::chrono::duration<double, std::milli>(
          SteadyClock::now() - began).count();
        return result;
      });
  }
  publish_status();
}

void PlanningManagerNode::poll_generation()
{
  if (!generation_.valid() ||
    generation_.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
  {
    return;
  }
  try {
    auto result = generation_.get();
    if (!active_ || !mission_.active || mission_.id != generation_mission_id_ ||
      !control_is_fresh() ||
      control_->flight_id != mission_.flight_id)
    {
      return;
    }
    planning_duration_ms_ = result.elapsed_ms;
    plan_ = std::move(result);
    plan_mission_id_ = mission_.id;
    plan_flight_id_ = mission_.flight_id;
    plan_final_ = !mission_.stop || mission_.index + 1 == mission_.goals.size();
    plan_start_ = get_clock()->now();
    mission_state_ = Status::TRACKING;
    publish_status();
  } catch (const std::exception & e) {
    if (active_ && mission_.active && mission_.id == generation_mission_id_) {
      abort_mission(e.what(), Status::FAILED);
    }
  }
}

void PlanningManagerNode::super_trajectory_callback(const Polynomial & msg)
{
  if (!active_ || planner_ != "super" || !mission_.active || !control_is_fresh() ||
    msg.header.stamp.sec < 0 || msg.header.stamp.nanosec >= 1000000000U ||
    msg.header.frame_id != mission_.frame || rclcpp::Time(msg.header.stamp) < super_goal_time_)
  {
    return;
  }
  if ((msg.type & Polynomial::EMER_STOP) != 0) {
    if (!have_super_trajectory_ || matches_super_trajectory(msg)) {
      abort_mission("SUPER emergency stop");
    }
    return;
  }
  if ((msg.type & Polynomial::POSITION_TRAJ) != 0) {
    if (msg.start_wt_pos + duration(msg.time_pos) < super_goal_time_.seconds()) {
      return;
    }
    std::string reason;
    if (!store_super_trajectory(msg, reason)) {
      RCLCPP_ERROR(get_logger(), "%s", reason.c_str());
      return;
    }
    super_last_rx_ = SteadyClock::now();
    planning_duration_ms_ = std::chrono::duration<double, std::milli>(
      SteadyClock::now() - generation_started_).count();
    mission_state_ = Status::TRACKING;
    publish_status();
  } else if ((msg.type & Polynomial::HEART_BEAT) != 0 && matches_super_trajectory(msg)) {
    super_last_rx_ = SteadyClock::now();
  }
}

bool PlanningManagerNode::at_goal(const Goal & goal) const
{
  return odometry_ && odometry_->header.frame_id == mission_.frame &&
         distance(odometry_->pose.pose.position, goal.position) < position_tolerance_ &&
         std::abs(
    std::remainder(
      yaw_of(odometry_->pose.pose.orientation) - goal.heading,
      2.0 * M_PI)) < heading_tolerance_;
}

void PlanningManagerNode::timer_callback()
{
  poll_generation();
  if (!active_) {
    return;
  }
  if (!control_is_fresh() || !odometry_ ||
    SteadyClock::now() - odometry_rx_ > std::chrono::duration<double>(state_timeout_))
  {
    if (mission_.active) {
      abort_mission("control state or odometry timeout");
    }
    return;
  }
  if (mission_.active && mission_state_ == Status::GENERATING &&
    SteadyClock::now() - generation_started_ > std::chrono::duration<double>(planning_timeout_))
  {
    abort_mission("planning deadline exceeded", Status::FAILED);
    return;
  }
  const auto now = get_clock()->now();
  const double coverage = (control_->horizon_length + 2) * control_->horizon_dt;
  laser_msgs::msg::ReferenceHorizon output;
  output.header.stamp = now;
  output.header.frame_id = control_->header.frame_id;
  output.flight_id = plan_flight_id_;
  output.mission_id = plan_mission_id_;
  bool segment_terminal = false;
  if (planner_ == "pmm" && plan_) {
    const double elapsed = (now - plan_start_).seconds();
    const auto & times = plan_->times;
    const auto & states = plan_->states;
    if (elapsed < 0.0) {
      abort_mission("ROS clock moved backwards");
      return;
    }
    output.start_time = plan_start_;
    auto first = std::upper_bound(times.begin(), times.end(), elapsed);
    if (first != times.begin()) {
      --first;
    }
    auto last = std::lower_bound(first, times.end(), elapsed + coverage);
    if (last != times.end()) {
      ++last;
    }
    for (auto it = first; it != last; ++it) {
      output.time_from_start.push_back(*it);
      output.reference_horizon.push_back(states[static_cast<std::size_t>(it - times.begin())]);
    }
    segment_terminal = elapsed >= times.back();
    if (output.time_from_start.back() < elapsed + coverage) {
      output.time_from_start.push_back(elapsed + coverage);
      auto terminal = states.back();
      terminal.twist = geometry_msgs::msg::Twist();
      output.reference_horizon.push_back(terminal);
    }
    output.terminal = segment_terminal && plan_final_;
  } else if (planner_ == "super" && plan_mission_id_ != 0) {
    if (have_super_trajectory_) {
      if (now.seconds() < super_end_time() &&
        SteadyClock::now() - super_last_rx_ > std::chrono::duration<double>(super_timeout_))
      {
        abort_mission("SUPER heartbeat timeout");
        return;
      }
      const double origin = std::floor(now.seconds() / super_sampling_step_) * super_sampling_step_;
      output.start_time = rclcpp::Time(static_cast<int64_t>(origin * 1e9), RCL_ROS_TIME);
      const double sample_count = std::ceil(coverage / super_sampling_step_) + 2;
      if (!std::isfinite(sample_count) || sample_count > 65536) {
        abort_mission("SUPER sample window exceeds the message limit", Status::FAILED);
        return;
      }
      const int count = static_cast<int>(sample_count);
      if (!sample_super_trajectory(
          origin, count, super_sampling_step_,
          yaw_of(super_hold_.pose.orientation), output.reference_horizon, segment_terminal))
      {
        abort_mission("invalid SUPER numerical samples", Status::FAILED);
        return;
      }
      for (int i = 0; i < count; ++i) {
        output.time_from_start.push_back(i * super_sampling_step_);
      }
      output.terminal = segment_terminal && mission_.index + 1 == mission_.goals.size();
    } else {
      if (!mission_.active) {
        return;
      }
      output.start_time = now;
      output.time_from_start = {0.0, coverage};
      output.reference_horizon = {super_hold_, super_hold_};
    }
  } else {
    return;
  }
  if (output.flight_id != control_->flight_id || output.mission_id == 0) {
    return;
  }
  const auto wall_now = SteadyClock::now();
  if (last_publication_) {
    reference_period_ms_ =
      std::chrono::duration<double, std::milli>(wall_now - *last_publication_).count();
  }
  last_publication_ = wall_now;
  pub_horizon_->publish(output);
  if (!mission_.active || output.mission_id != mission_.id || !segment_terminal) {
    return;
  }
  const Goal & target = mission_.stop ? mission_.goals[mission_.index] : mission_.goals.back();
  if (!at_goal(target)) {
    dwelling_since_.reset();
    return;
  }
  if (mission_.stop && mission_.index + 1 < mission_.goals.size()) {
    if (!dwelling_since_) {
      dwelling_since_ = now;
    }
    if ((now - *dwelling_since_).seconds() >= waypoint_hold_time_) {
      ++mission_.index;
      start_segment();
    }
  } else {
    mission_.active = false;
    mission_state_ = Status::COMPLETED;
    detail_ = "mission completed";
    publish_status();
  }
}

void PlanningManagerNode::abort_mission(const std::string & reason, uint8_t state)
{
  if (mission_.active) {
    mission_.active = false;
    mission_state_ = state;
    detail_ = reason;
    publish_status();
  }
  plan_.reset();
  have_super_trajectory_ = false;
  plan_mission_id_ = 0;
  dwelling_since_.reset();
}

void PlanningManagerNode::publish_status()
{
  if (!active_) {
    return;
  }
  Status msg;
  msg.header.stamp = get_clock()->now();
  msg.header.frame_id = mission_.frame.empty() &&
    control_ ? control_->header.frame_id : mission_.frame;
  msg.flight_id = mission_.flight_id;
  msg.mission_id = mission_.id;
  msg.start_time = mission_.start;
  msg.planner = planner_;
  msg.state = mission_state_;
  msg.waypoint_count = static_cast<uint32_t>(mission_.goals.size());
  if (!mission_.goals.empty()) {
    msg.goal = mission_.goals.back();
  }
  msg.detail = detail_;
  msg.planning_duration_ms = planning_duration_ms_;
  msg.reference_period_ms = reference_period_ms_;
  pub_status_->publish(msg);
}
}  // namespace laser_uav_managers

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(laser_uav_managers::PlanningManagerNode)
