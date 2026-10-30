#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <limits>
#include <thread>
#include <unistd.h>

#include <gtest/gtest.h>
#include <rcl/time.h>

#include "laser_uav_managers/control_manager_node.hpp"
#include "laser_uav_managers/planning_manager_node.hpp"

namespace laser_uav_managers
{
using Horizon = std::vector<laser_msgs::msg::ReferenceState>;
using Window = laser_msgs::msg::ReferenceHorizon;
using Status = laser_msgs::msg::UavPlanningDiagnostics;
using Polynomial = mars_quadrotor_msgs::msg::PolynomialTrajectory;

void set_time(rclcpp_lifecycle::LifecycleNode & node, double seconds)
{
  ASSERT_EQ(rcl_enable_ros_time_override(node.get_clock()->get_clock_handle()), RCL_RET_OK);
  ASSERT_EQ(
    rcl_set_ros_time_override(
      node.get_clock()->get_clock_handle(),
      static_cast<int64_t>(seconds * 1e9)), RCL_RET_OK);
}

struct ControlManagerNodeTestAccess
{
  static void initialize(ControlManagerNode & node)
  {
    node.get_parameters();
    node.validate_parameters();
    node.is_active_ = true;
  }
  static void odometry(ControlManagerNode & node, double x, double z, double yaw, double y = -2.0)
  {
    nav_msgs::msg::Odometry msg;
    msg.header.frame_id = "map";
    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    msg.pose.pose.position.z = z;
    msg.pose.pose.orientation = node.heading_to_quaternion(yaw);
    node.odometry_callback(msg);
  }
  static bool request(ControlManagerNode & node, bool land = false)
  {
    auto response = std::make_shared<std_srvs::srv::Trigger::Response>();
    if (land) {
      node.land_service_callback(nullptr, response);
    } else {
      node.takeoff_service_callback(nullptr, response);
      laser_msgs::msg::ApiPx4Diagnostics api;
      api.armed = true;
      api.offboard_mode = true;
      node.api_diagnostics_callback(api);
    }
    return response->success;
  }
  static Horizon tick(ControlManagerNode & node, double time)
  {
    EXPECT_TRUE(
      node.update_reference_horizon(
        rclcpp::Time(
          static_cast<int64_t>(time * 1e9), RCL_ROS_TIME)));
    return node.current_horizon_path_;
  }
  static void flying(ControlManagerNode & node)
  {
    node.requested_takeoff_ = false;
    node.takeoff_done_ = true;
    node.diagnostics_.is_fly = true;
  }
  static void receive(ControlManagerNode & node, const Window & msg)
  {
    node.reference_horizon_callback(msg);
  }
  static void status(ControlManagerNode & node, const Status & msg)
  {
    node.planning_diagnostics_callback(msg);
  }
  static uint64_t flight(const ControlManagerNode & node) {return node.flight_id_;}
  static void age(ControlManagerNode & node)
  {
    node.last_reference_rx_ = node.timeout_clock_->now() - rclcpp::Duration::from_seconds(10.0);
  }
  static void simulation(ControlManagerNode & node)
  {
    ASSERT_TRUE(node.set_parameter(rclcpp::Parameter("use_sim_time", true)).successful);
    node.get_parameters();
    set_time(node, 120.0);
  }
  static void timers(ControlManagerNode & node) {node.configure_timers();}
  static bool timer_ready(ControlManagerNode & node)
  {
    return node.tmr_external_loop_control_->is_ready();
  }
  static bool has_external(const ControlManagerNode & node) {return node.have_external_reference_;}
  static bool rmse_active(const ControlManagerNode & node) {return node.rmse_run_active_;}
  static bool validate(ControlManagerNode & node, const Window & msg)
  {
    std::string reason;
    return node.validate_reference_horizon(msg, reason);
  }
  static void safe_area(ControlManagerNode & node)
  {
    node.safe_area_ = {true, {-0.5, 0.5}, {-5.0, 5.0}, {0.1, 2.0}};
  }
  static std::string frame(const ControlManagerNode & node) {return node.reference_frame_id_;}
  static void configure_control(ControlManagerNode & node)
  {
    node.configure_classes();
    node.configure_pub_sub();
    node.on_activate(rclcpp_lifecycle::State());
  }
  static void control_tick(ControlManagerNode & node, int seconds)
  {
    set_time(node, seconds);
    node.external_loop_timer_callback();
  }
  static Horizon horizon(const ControlManagerNode & node) {return node.current_horizon_path_;}
  static bool takeoff_done(const ControlManagerNode & node) {return node.takeoff_done_;}
  static Eigen::Vector3d rates(const ControlManagerNode & node) {return node.nmpc_solution_.first;}
  static void benchmark(ControlManagerNode & node, const std::string & path)
  {
    node.benchmark_.enabled = true;
    node.benchmark_.csv_path = path;
  }
  static void benchmark_tick(ControlManagerNode & node, int seconds)
  {
    node.update_benchmark(rclcpp::Time(seconds, 0, RCL_ROS_TIME));
  }
  static geometry_msgs::msg::TransformStamped debug_tf(
    const ControlManagerNode & node, const nav_msgs::msg::Odometry & msg)
  {
    return node.odometry_debug_transform(msg);
  }
};
using Access = ControlManagerNodeTestAccess;

struct PlanningManagerNodeTestAccess
{
  static void configure(PlanningManagerNode & node)
  {
    ASSERT_EQ(
      node.on_configure(
        rclcpp_lifecycle::State()), PlanningManagerNode::CallbackReturn::SUCCESS);
    ASSERT_EQ(
      node.on_activate(
        rclcpp_lifecycle::State()), PlanningManagerNode::CallbackReturn::SUCCESS);
  }
  static void ready(
    PlanningManagerNode & node, uint64_t flight, double x = 1.0, double velocity = 0.0)
  {
    laser_msgs::msg::UavControlState state;
    state.header.frame_id = "map";
    state.flight_id = flight;
    state.ready_for_tracking = true;
    state.horizon_length = 20;
    state.horizon_dt = 0.05;
    state.estimated_mass = 2.0;
    state.reference.pose.position.x = x;
    state.reference.pose.position.y = -2.0;
    state.reference.pose.position.z = 1.5;
    state.reference.pose.orientation.w = 1.0;
    state.reference.twist.linear.x = velocity;
    state.reference.use_position = state.reference.use_orientation = true;
    state.reference.use_linear_velocity = state.reference.use_angular_velocity = true;
    node.control_state_callback(state);
    nav_msgs::msg::Odometry odom;
    odom.header.frame_id = "map";
    odom.pose.pose = state.reference.pose;
    odom.twist.twist = state.reference.twist;
    node.odometry_callback(odom);
  }
  static void feedback(PlanningManagerNode & node, double x)
  {
    auto odom = *node.odometry_;
    odom.pose.pose.position.x = x;
    node.odometry_callback(odom);
  }
  static void goto_goal(PlanningManagerNode & node, double x, bool relative = false)
  {
    laser_msgs::msg::PoseWithHeading goal;
    goal.position.x = x;
    if (relative) {
      node.goto_relative_callback(goal);
    } else {
      goal.position.y = -2.0;
      goal.position.z = 1.5;
      node.goto_callback(goal);
    }
  }
  static const Horizon & refs(const PlanningManagerNode & node) {return node.plan_->states;}
  static uint64_t mission(const PlanningManagerNode & node) {return node.mission_.id;}
  static auto delayed_worker(PlanningManagerNode & node)
  {
    std::promise<PlanningManagerNode::GeneratedPlan> result;
    node.generation_ = result.get_future();
    node.generation_mission_id_ = node.mission_.id;
    return result;
  }
  static auto minco(
    const laser_msgs::msg::ReferenceState & start,
    const std::vector<laser_msgs::msg::PoseWithHeading> & goals)
  {
    return PlanningManagerNode::generate_minco(start, goals, 2.0, 2.0, 0.01, 1.0);
  }
  static bool goal(
    PlanningManagerNode & node, double x, bool stop = false, bool multiple = false,
    double speed = 0.0)
  {
    laser_msgs::msg::PoseWithHeading goal;
    goal.position.x = x;
    goal.position.y = -2.0;
    goal.position.z = 1.5;
    std::vector<laser_msgs::msg::PoseWithHeading> goals{goal};
    if (multiple) {
      goal.position.x += 1.0;
      goals.push_back(goal);
    }
    return node.start_mission(goals, speed, stop);
  }
  static void tick(PlanningManagerNode & node)
  {
    node.timer_callback();
  }
  static bool has_plan(const PlanningManagerNode & node) {return node.plan_.has_value();}
  static uint8_t state(const PlanningManagerNode & node) {return node.mission_state_;}
  static void receive(PlanningManagerNode & node, const Polynomial & msg)
  {
    node.super_trajectory_callback(msg);
  }
  static bool store(PlanningManagerNode & node, const Polynomial & msg)
  {
    std::string reason;
    return node.store_super_trajectory(msg, reason);
  }
  static Horizon sample(PlanningManagerNode & node, double time, bool & terminal)
  {
    Horizon refs;
    EXPECT_TRUE(node.sample_super_trajectory(time, 21, 0.05, 0.7, refs, terminal));
    return refs;
  }
  static void age_super(PlanningManagerNode & node)
  {
    node.super_last_rx_ = node.timeout_clock_->now() - rclcpp::Duration::from_seconds(10.0);
  }
  static void planning_timeout(PlanningManagerNode & node, double timeout)
  {
    node.planning_timeout_ = timeout;
  }
  static bool timer_ready(PlanningManagerNode & node) {return node.timer_->is_ready();}
  static void cancel(PlanningManagerNode & node) {node.abort_mission("test cancellation");}
  static double end_time(const PlanningManagerNode & node)
  {
    return node.plan_start_.seconds() + node.plan_->times.back();
  }
  static std::size_t waypoint(const PlanningManagerNode & node) {return node.mission_.index;}
  static void finish_dwell(PlanningManagerNode & node)
  {
    node.waypoint_hold_time_ = 0.0;
  }
  static void wait(PlanningManagerNode & node)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (node.generation_.valid() && std::chrono::steady_clock::now() < deadline) {
      node.control_rx_ = node.odometry_rx_ = node.timeout_clock_->now();
      node.timer_callback();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    ASSERT_FALSE(node.generation_.valid());
  }
};
using Planning = PlanningManagerNodeTestAccess;

class ReferenceHorizonsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("benchmark_metrics.enabled", false)});
    options.arguments(
      {"--ros-args", "--params-file", TEST_MANAGER_CONFIG_PATH,
        "--params-file", TEST_UAV_CONFIG_PATH, "--params-file", TEST_CONTROLLER_CONFIG_PATH,
        "-r", "__ns:=/laser_reference_test"});
    node = std::make_unique<ControlManagerNode>(options);
    Access::initialize(*node);
    set_time(*node, 120.0);
    Access::odometry(*node, 1.0, 0.2, 0.7);
  }
  void TearDown() override
  {
    planning.reset();
    sub_horizon.reset();
    observer.reset();
    node.reset();
    rclcpp::shutdown();
    if (!csv_path.empty()) {
      std::remove(csv_path.c_str());
    }
  }
  void hover()
  {
    ASSERT_TRUE(Access::request(*node));
    Access::tick(*node, 100.0);
    Access::tick(*node, 110.0);
    Access::flying(*node);
  }
  Window window(uint64_t id = 1, bool terminal = false)
  {
    Window msg;
    msg.header.frame_id = "map";
    msg.header.stamp = node->get_clock()->now();
    msg.start_time = rclcpp::Time(120, 0, RCL_ROS_TIME);
    msg.flight_id = Access::flight(*node);
    msg.mission_id = id;
    msg.terminal = terminal;
    msg.time_from_start = {0.0, 0.7, 3.5, 10.0};
    for (double time : msg.time_from_start) {
      laser_msgs::msg::ReferenceState ref;
      ref.pose.position.x = terminal ? 6.0 : 1.0 + 0.5 * time;
      ref.pose.position.y = -2.0;
      ref.pose.position.z = 1.5;
      ref.pose.orientation.w = std::cos(0.35);
      ref.pose.orientation.z = std::sin(0.35);
      ref.twist.linear.x = terminal ? 0.0 : 0.5;
      ref.use_position = ref.use_orientation = true;
      ref.use_linear_velocity = ref.use_angular_velocity = true;
      msg.reference_horizon.push_back(ref);
    }
    return msg;
  }
  Status status(uint8_t state, uint64_t id = 1)
  {
    Status msg;
    msg.header.frame_id = "map";
    msg.header.stamp = node->get_clock()->now();
    msg.start_time = rclcpp::Time(120, 0, RCL_ROS_TIME);
    msg.flight_id = Access::flight(*node);
    msg.mission_id = id;
    msg.state = state;
    msg.waypoint_count = 3;
    msg.goal.position.x = 2.0;
    msg.goal.position.y = -2.0;
    msg.goal.position.z = 1.5;
    msg.goal.heading = 0.7;
    return msg;
  }
  static Polynomial polynomial()
  {
    Polynomial msg;
    msg.header.frame_id = "map";
    msg.header.stamp.sec = 120;
    msg.type = msg.POSITION_TRAJ;
    msg.start_wt_pos = 120.0;
    msg.piece_num_pos = 1;
    msg.order_pos = 1;
    msg.time_pos = {10.0};
    msg.coef_pos_x = {0.5, 1.0};
    msg.coef_pos_y = {0.0, -2.0};
    msg.coef_pos_z = {0.0, 1.5};
    return msg;
  }
  void planner(const std::string & backend, bool use_sim_time = false)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("planner", backend),
      rclcpp::Parameter("use_sim_time", use_sim_time)});
    options.arguments(
      {"--ros-args", "--params-file", TEST_UAV_CONFIG_PATH,
        "--params-file", TEST_PLANNER_CONFIG_PATH, "-r", "__ns:=/laser_reference_test"});
    planning = std::make_unique<PlanningManagerNode>(options);
    set_time(*planning, 120.0);
    Planning::configure(*planning);
    Planning::ready(*planning, Access::flight(*node));
    observer = std::make_shared<rclcpp::Node>("reference_observer");
    sub_horizon = observer->create_subscription<Window>(
      "/laser_reference_test/reference_horizon_out", rclcpp::QoS(1).best_effort(),
      [this](const Window & msg) {published = msg;});
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (planning->count_subscribers("/laser_reference_test/reference_horizon_out") == 0 &&
      std::chrono::steady_clock::now() < deadline)
    {
      rclcpp::spin_some(observer);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    ASSERT_GT(planning->count_subscribers("/laser_reference_test/reference_horizon_out"), 0U);
  }
  Window planning_tick()
  {
    published.reset();
    Planning::tick(*planning);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (!published && std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(observer);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    EXPECT_TRUE(published.has_value());
    return published.value_or(Window{});
  }
  void prepare_benchmark()
  {
    char path[] = "/tmp/laser_control_manager_csv_test_XXXXXX";
    const int fd = mkstemp(path);
    ASSERT_GE(fd, 0);
    close(fd);
    csv_path = path;
    Access::benchmark(*node, csv_path);
    Access::odometry(*node, 1.0, 1.5, 0.7);
    Access::status(*node, status(Status::GENERATING));
  }
  std::string csv_contents()
  {
    std::ifstream input(csv_path);
    return std::string(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
  }
  std::unique_ptr<ControlManagerNode> node;
  std::unique_ptr<PlanningManagerNode> planning;
  rclcpp::Node::SharedPtr observer;
  rclcpp::Subscription<Window>::SharedPtr sub_horizon;
  std::optional<Window> published;
  std::string csv_path;
};

TEST_F(ReferenceHorizonsTest, NativeWindowInterpolatesIrregularTimesIntoControllerGrid)
{
  hover();
  Access::receive(*node, window());
  const auto refs = Access::tick(*node, 122.0);
  ASSERT_EQ(refs.size(), 21U);
  for (std::size_t i = 0; i < refs.size(); ++i) {
    EXPECT_NEAR(refs[i].pose.position.x, 2.0 + 0.025 * i, 1e-12);
    EXPECT_DOUBLE_EQ(refs[i].twist.linear.x, 0.5);
    EXPECT_NEAR(refs[i].pose.orientation.z, std::sin(0.35), 1e-12);
  }
}

TEST_F(ReferenceHorizonsTest, NativeWindowPreservesAttitudeRatesAndIndividualThrust)
{
  hover();
  auto msg = window();
  msg.time_from_start = {0.0, 1.0};
  msg.reference_horizon.resize(2);
  for (std::size_t i = 0; i < 2; ++i) {
    auto & r = msg.reference_horizon[i];
    r.pose.orientation.w = std::cos(0.5 * i);
    r.pose.orientation.z = std::sin(0.5 * i);
    r.twist.angular.z = 0.2 * i;
    r.use_individual_thrust = true;
    r.individual_thrust.unit_of_measurement = "N";
    r.individual_thrust.data.assign(4, 2.0 + i);
  }
  Access::receive(*node, msg);
  const auto refs = Access::tick(*node, 120.0);
  EXPECT_NEAR(refs[10].pose.orientation.z, std::sin(0.25), 1e-12);
  EXPECT_NEAR(refs[10].twist.angular.z, 0.1, 1e-12);
  EXPECT_NEAR(refs[10].individual_thrust.data[0], 2.5, 1e-12);
}

TEST_F(ReferenceHorizonsTest, TimeoutLatchesLastCommandAndRequiresNewMission)
{
  hover();
  Access::receive(*node, window());
  const auto tracking = Access::tick(*node, 122.0);
  Access::odometry(*node, 30.0, 15.0, -1.0);
  Access::age(*node);
  const auto held = Access::tick(*node, 122.1);
  EXPECT_EQ(held.front().pose.position, tracking.front().pose.position);
  EXPECT_NEAR(held.front().pose.orientation.z, tracking.front().pose.orientation.z, 1e-12);
  EXPECT_NEAR(held.front().pose.orientation.w, tracking.front().pose.orientation.w, 1e-12);
  EXPECT_DOUBLE_EQ(held.front().twist.linear.x, 0.0);
  set_time(*node, 123.0);
  Access::receive(*node, window());
  EXPECT_EQ(Access::tick(*node, 123.0), held);
  Access::receive(*node, window(2));
  EXPECT_DOUBLE_EQ(Access::tick(*node, 123.0).front().pose.position.x, 2.5);
}

TEST_F(ReferenceHorizonsTest, MissingFutureCoverageHoldsCommandWithoutExtrapolation)
{
  hover();
  Access::receive(*node, window());
  const auto tracked = Access::tick(*node, 128.0);
  const auto held = Access::tick(*node, 129.5);
  EXPECT_EQ(held.front().pose, tracked.front().pose);
  EXPECT_DOUBLE_EQ(held.front().twist.linear.x, 0.0);
}

TEST_F(ReferenceHorizonsTest, TerminalTargetSurvivesTimeoutAndOdometryDrift)
{
  hover();
  Access::receive(*node, window(1, true));
  const auto held = Access::tick(*node, 130.0);
  Access::age(*node);
  Access::odometry(*node, 50.0, 20.0, 2.0);
  EXPECT_EQ(Access::tick(*node, 200.0), held);
  EXPECT_DOUBLE_EQ(held.front().pose.position.x, 6.0);
}

TEST_F(ReferenceHorizonsTest, RejectsMalformedReferencesAndNonstationaryTerminal)
{
  hover();
  auto msg = window();
  msg.time_from_start[1] = 0.0;
  EXPECT_FALSE(Access::validate(*node, msg));
  msg = window();
  msg.reference_horizon[1].pose.position.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(Access::validate(*node, msg));
  msg = window();
  msg.reference_horizon[1].pose.orientation = geometry_msgs::msg::Quaternion();
  msg.reference_horizon[1].pose.orientation.w = 0.0;
  EXPECT_FALSE(Access::validate(*node, msg));
  msg = window();
  msg.reference_horizon[1].use_linear_velocity = false;
  EXPECT_FALSE(Access::validate(*node, msg));
  msg = window();
  msg.terminal = true;
  EXPECT_FALSE(Access::validate(*node, msg));
}

TEST_F(ReferenceHorizonsTest, RejectsStalePacketsWrongFrameAndPreviousFlight)
{
  hover();
  auto msg = window();
  msg.header.stamp.sec -= 1;
  Access::receive(*node, msg);
  EXPECT_FALSE(Access::has_external(*node));
  msg = window();
  msg.header.frame_id = "other";
  Access::receive(*node, msg);
  EXPECT_FALSE(Access::has_external(*node));
  msg = window();
  --msg.flight_id;
  Access::receive(*node, msg);
  EXPECT_FALSE(Access::has_external(*node));
}

TEST_F(ReferenceHorizonsTest, ExternalWindowsCannotInterruptTakeoffOrLanding)
{
  ASSERT_TRUE(Access::request(*node));
  Access::tick(*node, 100.0);
  Access::receive(*node, window());
  EXPECT_FALSE(Access::has_external(*node));
  Access::tick(*node, 110.0);
  Access::flying(*node);
  Access::receive(*node, window());
  const auto tracked = Access::tick(*node, 122.0);
  ASSERT_TRUE(Access::request(*node, true));
  Access::receive(*node, window(2));
  EXPECT_FALSE(Access::has_external(*node));
  const auto landing = Access::tick(*node, 123.0).front().pose;
  EXPECT_EQ(landing.position, tracked.front().pose.position);
  const auto & q = tracked.front().pose.orientation;
  const auto & actual = landing.orientation;
  EXPECT_NEAR(
    Eigen::Quaterniond(actual.w, actual.x, actual.y, actual.z).angularDistance(
      Eigen::Quaterniond(q.w, q.x, q.y, q.z)), 0.0, 1e-12);
}

TEST_F(ReferenceHorizonsTest, PublicationStampAllowsOneControlCycleOfClockSkew)
{
  hover();
  auto msg = window();
  msg.header.stamp.nanosec = 5000000;
  Access::receive(*node, msg);
  EXPECT_TRUE(Access::has_external(*node));
  msg = window(2);
  msg.header.stamp.nanosec = 20000000;
  for (auto & ref : msg.reference_horizon) {
    ref.pose.position.x += 20.0;
  }
  Access::receive(*node, msg);
  EXPECT_NEAR(Access::tick(*node, 121.0).front().pose.position.x, 1.5, 1e-9);
}

TEST_F(ReferenceHorizonsTest, FutureTrajectoryWaitsForCoverageWithoutClosingCurrentMission)
{
  hover();
  Access::receive(*node, window());
  const auto before = Access::tick(*node, 120.0);
  auto next = window(2);
  next.header.stamp.nanosec = next.start_time.nanosec = 5000000;
  next.reference_horizon.front().pose.position.x = 10.0;
  Access::receive(*node, next);
  EXPECT_EQ(Access::tick(*node, 120.0), before);
  set_time(*node, 120.005);
  Access::receive(*node, next);
  EXPECT_DOUBLE_EQ(Access::tick(*node, 120.005).front().pose.position.x, 10.0);
}

TEST_F(ReferenceHorizonsTest, AbortHoldsCommandAndRejectsDelayedMissionWindows)
{
  hover();
  Access::receive(*node, window());
  const auto tracked = Access::tick(*node, 121.0);
  Access::status(*node, status(Status::ABORTED));
  Access::odometry(*node, 20.0, 10.0, 2.0);
  Access::receive(*node, window());
  EXPECT_EQ(Access::tick(*node, 122.0).front().pose, tracked.front().pose);
  Access::receive(*node, window(2));
  EXPECT_DOUBLE_EQ(Access::tick(*node, 123.0).front().pose.position.x, 2.5);
}

TEST_F(ReferenceHorizonsTest, FailedReplacementImmediatelyHoldsTheCurrentMission)
{
  hover();
  Access::receive(*node, window());
  const auto tracked = Access::tick(*node, 121.0);
  Access::status(*node, status(Status::FAILED, 2));
  EXPECT_FALSE(Access::has_external(*node));
  Access::receive(*node, window());
  EXPECT_EQ(Access::tick(*node, 122.0).front().pose, tracked.front().pose);
}

TEST_F(ReferenceHorizonsTest, ReplacementStartsANewTrackingMetricsRun)
{
  hover();
  Access::receive(*node, window());
  Access::tick(*node, 120.0);
  ASSERT_TRUE(Access::rmse_active(*node));
  Access::receive(*node, window(2));
  EXPECT_FALSE(Access::rmse_active(*node));
  Access::tick(*node, 120.0);
  EXPECT_TRUE(Access::rmse_active(*node));
}

TEST_F(ReferenceHorizonsTest, BenchmarkAppendsThirteenColumnCsvOnceOnMissionCompletion)
{
  hover();
  prepare_benchmark();
  Access::odometry(*node, 1.5, 1.5, 0.7);
  Access::odometry(*node, 2.0, 1.5, 0.7);
  Access::benchmark_tick(*node, 123);
  EXPECT_TRUE(csv_contents().empty());
  Access::status(*node, status(Status::COMPLETED));
  Access::benchmark_tick(*node, 124);
  const auto csv = csv_contents();
  EXPECT_EQ(std::count(csv.begin(), csv.end(), '\n'), 2);
  EXPECT_NE(csv.find("1,120.000000,124.000000,4.000000,1.000000,0.250000,3,"), std::string::npos);
  Access::benchmark_tick(*node, 125);
  EXPECT_EQ(csv_contents(), csv);
}

TEST_F(ReferenceHorizonsTest, BenchmarkDoesNotRecordAbortedMission)
{
  hover();
  prepare_benchmark();
  Access::status(*node, status(Status::ABORTED));
  Access::status(*node, status(Status::COMPLETED));
  Access::benchmark_tick(*node, 125);
  EXPECT_TRUE(csv_contents().empty());
}

TEST_F(ReferenceHorizonsTest, PlannerSelectionIsReadOnly)
{
  hover();
  planner("pmm");
  EXPECT_FALSE(planning->set_parameter(rclcpp::Parameter("planner", "super")).successful);
}

TEST_F(ReferenceHorizonsTest, PmmWorkerPublishesRawTimesAndControllerResamples)
{
  hover();
  planner("pmm");
  ASSERT_TRUE(Planning::goal(*planning, 2.0));
  Planning::wait(*planning);
  ASSERT_EQ(Planning::state(*planning), Status::TRACKING);
  const auto raw = planning_tick();
  ASSERT_GT(raw.time_from_start.size(), 21U);
  EXPECT_NEAR(raw.time_from_start[1] - raw.time_from_start[0], 0.01, 1e-9);
  Access::receive(*node, raw);
  const auto refs = Access::tick(*node, 120.0);
  EXPECT_EQ(refs.size(), 21U);
  EXPECT_TRUE(refs.front().use_orientation);
  EXPECT_GT(refs.back().pose.position.x, refs.front().pose.position.x);
}

TEST_F(ReferenceHorizonsTest, CancelledWorkerCannotInstallItsOldResult)
{
  hover();
  planner("pmm");
  ASSERT_TRUE(Planning::goal(*planning, 3.0));
  Planning::cancel(*planning);
  Planning::wait(*planning);
  EXPECT_EQ(Planning::state(*planning), Status::ABORTED);
  EXPECT_FALSE(Planning::has_plan(*planning));
}

TEST_F(ReferenceHorizonsTest, PmmGotoReplacesFlyThroughFromCommandedPositionAndVelocity)
{
  hover();
  planner("pmm");
  ASSERT_TRUE(Planning::goal(*planning, 8.0, false, true, 6.0));
  Planning::wait(*planning);
  const auto old = planning_tick();
  Planning::ready(*planning, Access::flight(*node), 2.0, 6.0);
  Planning::feedback(*planning, 30.0);
  Planning::goto_goal(*planning, -2.0);
  ASSERT_GT(Planning::mission(*planning), old.mission_id);
  Planning::wait(*planning);
  ASSERT_EQ(Planning::state(*planning), Status::TRACKING);
  const auto & refs = Planning::refs(*planning);
  ASSERT_GE(refs.size(), 2U);
  EXPECT_NEAR(refs.front().pose.position.x, 2.0, 1e-8);
  EXPECT_NEAR(refs.front().twist.linear.x, 6.0, 1e-8);
  EXPECT_NEAR(refs.back().pose.position.x, -2.0, 1e-8);
  EXPECT_DOUBLE_EQ(refs.back().twist.linear.x, 0.0);
}

TEST_F(ReferenceHorizonsTest, LatestGotoWaitsForOldWorkerAndKeepsStreamingOldPlan)
{
  hover();
  planner("pmm");
  ASSERT_TRUE(Planning::goal(*planning, 3.0));
  Planning::wait(*planning);
  const auto old = planning_tick();
  auto worker = Planning::delayed_worker(*planning);
  Planning::goto_goal(*planning, 4.0);
  Planning::goto_goal(*planning, 5.0);
  EXPECT_EQ(planning_tick().mission_id, old.mission_id);
  EXPECT_EQ(Planning::state(*planning), Status::GENERATING);
  worker.set_exception(std::make_exception_ptr(std::runtime_error("superseded worker")));
  Planning::wait(*planning);
  ASSERT_EQ(Planning::state(*planning), Status::TRACKING);
  EXPECT_NEAR(Planning::refs(*planning).back().pose.position.x, 5.0, 1e-8);
  EXPECT_GT(planning_tick().mission_id, old.mission_id);
}

TEST_F(ReferenceHorizonsTest, RelativeGotoReplacesMissionAndInvalidGoalLeavesItRunning)
{
  hover();
  planner("pmm");
  ASSERT_TRUE(Planning::goal(*planning, 8.0));
  Planning::wait(*planning);
  const auto id = Planning::mission(*planning);
  Planning::goto_goal(*planning, std::numeric_limits<double>::quiet_NaN());
  EXPECT_EQ(Planning::mission(*planning), id);
  EXPECT_EQ(Planning::state(*planning), Status::TRACKING);
  Planning::ready(*planning, Access::flight(*node), 2.0, 0.5);
  Planning::goto_goal(*planning, 1.0, true);
  Planning::wait(*planning);
  ASSERT_EQ(Planning::state(*planning), Status::TRACKING);
  EXPECT_NEAR(Planning::refs(*planning).back().pose.position.x, 3.0, 1e-8);
}

TEST_F(ReferenceHorizonsTest, SuccessfulSupersededWorkerCannotInstallItsOldResult)
{
  hover();
  planner("pmm");
  ASSERT_TRUE(Planning::goal(*planning, 3.0));
  Planning::wait(*planning);
  auto worker = Planning::delayed_worker(*planning);
  Planning::goto_goal(*planning, 5.0);
  worker.set_value({});
  Planning::wait(*planning);
  ASSERT_EQ(Planning::state(*planning), Status::TRACKING);
  EXPECT_NEAR(Planning::refs(*planning).back().pose.position.x, 5.0, 1e-8);
}

TEST_F(ReferenceHorizonsTest, PmmWindowsKeepTheirGenerationEpochAndCoverClockSkew)
{
  hover();
  planner("pmm");
  ASSERT_TRUE(Planning::goal(*planning, 8.0));
  set_time(*planning, 120.065);
  Planning::wait(*planning);
  const auto raw = planning_tick();
  EXPECT_EQ(rclcpp::Time(raw.start_time).nanoseconds(), rclcpp::Time(120, 0, RCL_ROS_TIME).nanoseconds());
  EXPECT_LE(raw.time_from_start.front(), 0.015);
  set_time(*node, 120.06);
  Access::receive(*node, raw);
  EXPECT_TRUE(Access::has_external(*node));
  EXPECT_EQ(Access::tick(*node, 120.06).size(), 21U);
  EXPECT_TRUE(Access::has_external(*node));
}

TEST_F(ReferenceHorizonsTest, PmmStopsAtEachWaypointAndCompletesTheWholeMission)
{
  hover();
  planner("pmm");
  Planning::finish_dwell(*planning);
  ASSERT_TRUE(Planning::goal(*planning, 2.0, true, true));
  Planning::wait(*planning);
  set_time(*planning, Planning::end_time(*planning) + 0.01);
  Planning::ready(*planning, Access::flight(*node), 2.0);
  planning_tick();
  ASSERT_EQ(Planning::waypoint(*planning), 1U);
  Planning::wait(*planning);
  set_time(*planning, Planning::end_time(*planning) + 0.01);
  Planning::ready(*planning, Access::flight(*node), 3.0);
  const auto last = planning_tick();
  EXPECT_TRUE(last.terminal);
  EXPECT_EQ(Planning::state(*planning), Status::COMPLETED);
}

TEST_F(ReferenceHorizonsTest, PmmFlyThroughReachesFinalWaypointWithFiniteSamples)
{
  hover();
  planner("pmm");
  ASSERT_TRUE(Planning::goal(*planning, 2.0, false, true));
  Planning::wait(*planning);
  ASSERT_EQ(Planning::state(*planning), Status::TRACKING);
  set_time(*planning, Planning::end_time(*planning) + 0.01);
  Planning::ready(*planning, Access::flight(*node), 3.0);
  const auto last = planning_tick();
  ASSERT_FALSE(last.reference_horizon.empty());
  EXPECT_NEAR(last.reference_horizon.back().pose.position.x, 3.0, 1e-8);
  EXPECT_TRUE(last.terminal);
  EXPECT_EQ(Planning::state(*planning), Status::COMPLETED);
}

TEST_F(ReferenceHorizonsTest, SuperWaitingHoldDoesNotFollowUpdatedControlReference)
{
  hover();
  planner("super");
  ASSERT_TRUE(Planning::goal(*planning, 6.0));
  Planning::ready(*planning, Access::flight(*node), 20.0);
  const auto held = planning_tick();
  ASSERT_EQ(held.reference_horizon.size(), 2U);
  EXPECT_DOUBLE_EQ(held.reference_horizon.front().pose.position.x, 1.0);
}

TEST_F(ReferenceHorizonsTest, MincoMatchesTheMinimumSnapSepticAndRespectsLimits)
{
  auto start = window().reference_horizon.front();
  start.twist = geometry_msgs::msg::Twist();
  laser_msgs::msg::PoseWithHeading goal;
  goal.position = start.pose.position;
  goal.position.x = 3.0;
  goal.heading = 0.7;
  const auto plan = Planning::minco(start, {goal});
  ASSERT_GT(plan.states.size(), 2U);
  const double duration = plan.times.back();
  for (std::size_t i = 0; i < plan.states.size(); ++i) {
    const double u = plan.times[i] / duration;
    const double blend = std::pow(u, 4) * (35.0 + u * (-84.0 + u * (70.0 - 20.0 * u)));
    EXPECT_NEAR(plan.states[i].pose.position.x, 1.0 + 2.0 * blend, 1e-9);
    EXPECT_LE(std::abs(plan.states[i].twist.linear.x), 2.0 + 1e-5);
    if (i > 0) {
      const double dt = plan.times[i] - plan.times[i - 1];
      ASSERT_GT(dt, 0.0);
      EXPECT_LE(std::abs(plan.states[i].twist.linear.x -
          plan.states[i - 1].twist.linear.x) / dt, 2.0 + 1e-5);
    }
  }
  EXPECT_NEAR(plan.states.front().twist.linear.x, 0.0, 1e-9);
  EXPECT_NEAR(plan.states.back().twist.linear.x, 0.0, 1e-9);
}

TEST_F(ReferenceHorizonsTest, MincoFlyThroughPassesTheWaypointWithoutStoppingAndControlResamples)
{
  hover();
  planner("minco");
  ASSERT_TRUE(Planning::goal(*planning, 2.0, false, true, 0.7));
  Planning::wait(*planning);
  ASSERT_EQ(Planning::state(*planning), Status::TRACKING);
  const auto & refs = Planning::refs(*planning);
  const auto middle = std::min_element(refs.begin(), refs.end(), [](const auto & a, const auto & b) {
        return std::abs(a.pose.position.x - 2.0) < std::abs(b.pose.position.x - 2.0);
      });
  ASSERT_NE(middle, refs.end());
  EXPECT_NEAR(middle->pose.position.x, 2.0, 1e-8);
  EXPECT_GT(middle->twist.linear.x, 0.1);
  EXPECT_NEAR(refs.back().pose.position.x, 3.0, 1e-8);
  Access::receive(*node, planning_tick());
  EXPECT_EQ(Access::tick(*node, 120.0).size(), 21U);
}

TEST_F(ReferenceHorizonsTest, MincoMultiplePiecesRecoverTheGlobalMinimumSnapSolution)
{
  auto start = window().reference_horizon.front();
  start.twist = geometry_msgs::msg::Twist();
  laser_msgs::msg::PoseWithHeading middle;
  middle.position = start.pose.position;
  middle.position.x = 2.0;
  middle.heading = 0.7;
  auto last = middle;
  last.position.x = 3.0;
  const auto plan = Planning::minco(start, {middle, last});
  ASSERT_GT(plan.states.size(), 2U);
  for (std::size_t i = 0; i < plan.states.size(); ++i) {
    const double u = plan.times[i] / plan.times.back();
    const double blend = std::pow(u, 4) * (35.0 + u * (-84.0 + u * (70.0 - 20.0 * u)));
    EXPECT_NEAR(plan.states[i].pose.position.x, 1.0 + 2.0 * blend, 1e-8);
  }
}

TEST_F(ReferenceHorizonsTest, MincoStopsAtEachWaypointAndCompletesTheMission)
{
  hover();
  planner("minco");
  Planning::finish_dwell(*planning);
  ASSERT_TRUE(Planning::goal(*planning, 2.0, true, true));
  Planning::wait(*planning);
  set_time(*planning, Planning::end_time(*planning) + 0.01);
  Planning::ready(*planning, Access::flight(*node), 2.0);
  planning_tick();
  ASSERT_EQ(Planning::waypoint(*planning), 1U);
  Planning::wait(*planning);
  set_time(*planning, Planning::end_time(*planning) + 0.01);
  Planning::ready(*planning, Access::flight(*node), 3.0);
  EXPECT_TRUE(planning_tick().terminal);
  EXPECT_EQ(Planning::state(*planning), Status::COMPLETED);
}

TEST_F(ReferenceHorizonsTest, MincoGotoPreservesMovingReferenceAndRejectsExcessiveSpeed)
{
  hover();
  planner("minco");
  ASSERT_TRUE(Planning::goal(*planning, 8.0));
  Planning::wait(*planning);
  const auto id = Planning::mission(*planning);
  EXPECT_FALSE(Planning::goal(*planning, 3.0, false, false, 6.0));
  EXPECT_EQ(Planning::mission(*planning), id);
  Planning::ready(*planning, Access::flight(*node), 2.0, 3.0);
  Planning::feedback(*planning, 30.0);
  Planning::goto_goal(*planning, -2.0);
  Planning::wait(*planning);
  ASSERT_EQ(Planning::state(*planning), Status::TRACKING);
  EXPECT_NEAR(Planning::refs(*planning).front().pose.position.x, 2.0, 1e-8);
  EXPECT_NEAR(Planning::refs(*planning).front().twist.linear.x, 3.0, 1e-8);
  EXPECT_NEAR(Planning::refs(*planning).back().pose.position.x, -2.0, 1e-8);
}

TEST_F(ReferenceHorizonsTest, MincoYawOnlyUsesTheShortestRotationAcrossPi)
{
  auto start = window().reference_horizon.front();
  start.twist = geometry_msgs::msg::Twist();
  start.pose.orientation.w = std::cos(3.13 / 2.0);
  start.pose.orientation.z = std::sin(3.13 / 2.0);
  laser_msgs::msg::PoseWithHeading goal;
  goal.position = start.pose.position;
  goal.heading = -3.13;
  const auto plan = Planning::minco(start, {goal});
  ASSERT_GT(plan.states.size(), 2U);
  EXPECT_LT(plan.times.back(), 1.0);
  for (const auto & ref : plan.states) {
    EXPECT_NEAR(ref.pose.position.x, start.pose.position.x, 1e-8);
    EXPECT_LE(std::abs(ref.twist.angular.z), 1.0 + 1e-5);
    EXPECT_GE(ref.twist.angular.z, -1e-8);
  }
}

TEST_F(ReferenceHorizonsTest, SuperGotoKeepsTheCommittedTrajectoryUntilReplacementArrives)
{
  hover();
  planner("super");
  ASSERT_TRUE(Planning::goal(*planning, 6.0));
  Planning::receive(*planning, polynomial());
  const auto old = planning_tick();
  set_time(*planning, 120.1);
  Planning::ready(*planning, Access::flight(*node), 1.05, 0.5);
  Planning::goto_goal(*planning, -2.0);
  const auto pending = planning_tick();
  EXPECT_EQ(pending.mission_id, old.mission_id);
  EXPECT_NEAR(pending.reference_horizon.front().twist.linear.x, 0.5, 1e-9);
  Planning::receive(*planning, polynomial());
  EXPECT_EQ(Planning::state(*planning), Status::GENERATING);
  auto next = polynomial();
  next.header.stamp.nanosec = 100000000;
  next.start_wt_pos = 120.1;
  next.coef_pos_x[1] = 1.05;
  Planning::receive(*planning, next);
  EXPECT_EQ(Planning::state(*planning), Status::TRACKING);
  EXPECT_GT(planning_tick().mission_id, old.mission_id);
}

TEST_F(ReferenceHorizonsTest, SuperWaitingForNextWaypointDoesNotPublishAPrematureTerminal)
{
  hover();
  planner("super");
  Planning::finish_dwell(*planning);
  ASSERT_TRUE(Planning::goal(*planning, 2.0, true, true));
  auto first = polynomial();
  first.coef_pos_x[0] = 0.1;
  Planning::receive(*planning, first);
  set_time(*planning, 130.01);
  Planning::ready(*planning, Access::flight(*node), 2.0);
  planning_tick();
  ASSERT_EQ(Planning::waypoint(*planning), 1U);
  EXPECT_FALSE(planning_tick().terminal);
}

TEST_F(ReferenceHorizonsTest, SuperSamplingAndHeartbeatLiveOnlyInPlanningManager)
{
  hover();
  planner("super");
  ASSERT_TRUE(Planning::goal(*planning, 6.0));
  Planning::receive(*planning, polynomial());
  const auto raw = planning_tick();
  ASSERT_GT(raw.reference_horizon.size(), 21U);
  Access::receive(*node, raw);
  EXPECT_NEAR(Access::tick(*node, 120.0).back().pose.position.x, 1.5, 1e-9);
  Polynomial heartbeat;
  heartbeat.header = polynomial().header;
  heartbeat.type = heartbeat.HEART_BEAT;
  heartbeat.start_wt_pos = 121.0;
  Planning::age_super(*planning);
  Planning::receive(*planning, heartbeat);
  Planning::tick(*planning);
  EXPECT_EQ(Planning::state(*planning), Status::ABORTED);
}

TEST_F(ReferenceHorizonsTest, MatchingHeartbeatRefreshesSuperAndEmergencyStopAborts)
{
  hover();
  planner("super");
  ASSERT_TRUE(Planning::goal(*planning, 6.0));
  Planning::receive(*planning, polynomial());
  auto heartbeat = polynomial();
  heartbeat.type = heartbeat.HEART_BEAT;
  Planning::age_super(*planning);
  Planning::receive(*planning, heartbeat);
  Planning::tick(*planning);
  EXPECT_EQ(Planning::state(*planning), Status::TRACKING);
  heartbeat.type |= heartbeat.EMER_STOP;
  Planning::receive(*planning, heartbeat);
  EXPECT_EQ(Planning::state(*planning), Status::ABORTED);
}

TEST_F(ReferenceHorizonsTest, SuperRejectsUnsupportedFlyThroughBeforeStartingMission)
{
  hover();
  planner("super");
  EXPECT_FALSE(Planning::goal(*planning, 2.0, false, true));
  EXPECT_TRUE(Planning::goal(*planning, 2.0, true, true));
}

TEST_F(ReferenceHorizonsTest, SuperRejectsPerMissionSpeedInsteadOfSilentlyIgnoringIt)
{
  hover();
  planner("super");
  EXPECT_FALSE(Planning::goal(*planning, 2.0, false, false, 1.0));
  EXPECT_TRUE(Planning::goal(*planning, 2.0));
}

TEST_F(ReferenceHorizonsTest, SuperParserRejectsInvalidArraysAndSamplesYawAcrossPieces)
{
  hover();
  planner("super");
  auto msg = polynomial();
  msg.coef_pos_x.pop_back();
  EXPECT_FALSE(Planning::store(*planning, msg));
  msg = polynomial();
  msg.time_pos[0] = -1.0;
  EXPECT_FALSE(Planning::store(*planning, msg));
  msg = polynomial();
  msg.order_pos = 7;
  msg.piece_num_pos = 2;
  msg.time_pos = {2.0, 3.0};
  msg.coef_pos_x = {0, 0, 0, 0, 0, 0, 0.5, 1.0, 0, 0, 0, 0, 0, 0, 0.5, 2.0};
  msg.coef_pos_y = {0, 0, 0, 0, 0, 0, 0, -2.0, 0, 0, 0, 0, 0, 0, 0, -2.0};
  msg.coef_pos_z = {0, 0, 0, 0, 0, 0, 0, 1.5, 0, 0, 0, 0, 0, 0, 0, 1.5};
  msg.type |= msg.YAW_TRAJ;
  msg.start_wt_yaw = 120.0;
  msg.order_yaw = 1;
  msg.piece_num_yaw = 1;
  msg.time_yaw = {5.0};
  msg.coef_yaw = {0.1, 0.7};
  ASSERT_TRUE(Planning::store(*planning, msg));
  bool terminal;
  const auto refs = Planning::sample(*planning, 122.5, terminal);
  EXPECT_FALSE(terminal);
  EXPECT_NEAR(refs.front().pose.position.x, 2.25, 1e-10);
  EXPECT_NEAR(refs.front().pose.orientation.z, std::sin(0.475), 1e-10);
  EXPECT_NEAR(refs.front().twist.angular.z, 0.1, 1e-12);
  const auto held = Planning::sample(*planning, 126.0, terminal);
  EXPECT_TRUE(terminal);
  EXPECT_DOUBLE_EQ(held.front().pose.position.x, 3.5);
  EXPECT_DOUBLE_EQ(held.front().twist.angular.z, 0.0);
}

TEST_F(ReferenceHorizonsTest, TakeoffUsesInitialReferenceEvenIfOdometryMovesBeforeRequest)
{
  Access::odometry(*node, 20.0, 5.0, 2.0);
  ASSERT_TRUE(Access::request(*node));
  const auto start = Access::tick(*node, 100.0);
  ASSERT_EQ(start.size(), 21U);
  EXPECT_DOUBLE_EQ(start.front().pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(start.front().pose.position.z, 0.2);
  EXPECT_NEAR(start.front().pose.orientation.z, std::sin(0.35), 1e-12);
  EXPECT_EQ(Access::frame(*node), "map");
}

TEST_F(ReferenceHorizonsTest, TakeoffAndHoldDoNotFollowChangingOdometry)
{
  ASSERT_TRUE(Access::request(*node));
  for (double t : {100.0, 100.2, 101.0, 103.0, 110.0}) {
    const auto before = Access::tick(*node, t);
    Access::odometry(*node, t, 10.0, t / 10.0, t + 1.0);
    EXPECT_EQ(Access::tick(*node, t), before);
  }
  const auto terminal = Access::tick(*node, 110.0);
  Access::flying(*node);
  for (int i = 0; i < 100; ++i) {
    Access::odometry(*node, 20.0 + i, 2.0 + i, 0.1 * i, 10.0 + i);
    EXPECT_EQ(Access::tick(*node, 111.0 + i), terminal);
  }
}

TEST_F(ReferenceHorizonsTest, LandingStartsFromCommandedReferenceAndRemainsIndependentOfOdometry)
{
  hover();
  Access::odometry(*node, 20.0, 10.0, 2.0);
  ASSERT_TRUE(Access::request(*node, true));
  const auto start = Access::tick(*node, 120.0);
  EXPECT_DOUBLE_EQ(start.front().pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(start.front().pose.position.z, 1.5);
  for (double t : {120.0, 125.0, 130.0, 150.0}) {
    const auto before = Access::tick(*node, t);
    Access::odometry(*node, t, t, t / 10.0, t + 1.0);
    EXPECT_EQ(Access::tick(*node, t), before);
  }
  for (const auto & ref : Access::tick(*node, 150.0)) {
    EXPECT_NEAR(ref.pose.position.z, -1.0, 1e-12);
    EXPECT_DOUBLE_EQ(ref.twist.linear.z, 0.0);
  }
}

TEST_F(ReferenceHorizonsTest, VerticalProfilesRespectConfiguredPeakSpeeds)
{
  ASSERT_TRUE(Access::request(*node));
  Access::tick(*node, 100.0);
  const auto takeoff = Access::tick(*node, 101.21875);
  EXPECT_NEAR(takeoff.front().twist.linear.z, 1.0, 1e-12);
  for (const auto & ref : takeoff) {
    EXPECT_LE(std::abs(ref.twist.linear.z), 1.0 + 1e-12);
  }
  Access::tick(*node, 110.0);
  Access::flying(*node);
  ASSERT_TRUE(Access::request(*node, true));
  Access::tick(*node, 120.0);
  const auto landing = Access::tick(*node, 127.8125);
  EXPECT_NEAR(landing.front().twist.linear.z, -0.3, 1e-12);
  for (const auto & ref : landing) {
    EXPECT_LE(std::abs(ref.twist.linear.z), 0.3 + 1e-12);
  }
}

TEST_F(ReferenceHorizonsTest, SafeAreaHoldIsLatchedDuringTakeoff)
{
  Access::safe_area(*node);
  ASSERT_TRUE(Access::request(*node));
  const auto held = Access::tick(*node, 100.0);
  EXPECT_DOUBLE_EQ(held.front().pose.position.x, 0.45);
  Access::odometry(*node, 20.0, 10.0, 2.0);
  EXPECT_EQ(Access::tick(*node, 101.0), held);
  EXPECT_EQ(Access::tick(*node, 120.0), held);
}

TEST_F(ReferenceHorizonsTest, ActualNmpcLoopKeepsReferenceAcrossTakeoffCompletionAndLanding)
{
  Access::configure_control(*node);
  ASSERT_TRUE(Access::request(*node));
  Access::control_tick(*node, 100);
  Access::odometry(*node, 1.1, 1.5, 0.7);
  Access::control_tick(*node, 110);
  Access::control_tick(*node, 114);
  ASSERT_TRUE(Access::takeoff_done(*node));
  const auto terminal = Access::horizon(*node);
  Access::odometry(*node, 1.2, 1.5, 0.8);
  Access::control_tick(*node, 115);
  EXPECT_EQ(Access::horizon(*node), terminal);
  EXPECT_TRUE(Access::rates(*node).allFinite());
  EXPECT_GT(Access::rates(*node).norm(), 1e-5);
  ASSERT_TRUE(Access::request(*node, true));
  Access::control_tick(*node, 116);
  EXPECT_EQ(Access::horizon(*node).front().pose, terminal.front().pose);
}

TEST_F(ReferenceHorizonsTest, DebugTransformUsesMeasuredPoseWithoutChangingReference)
{
  hover();
  const auto held = Access::tick(*node, 120.0);
  nav_msgs::msg::Odometry msg;
  msg.header.stamp.sec = 123;
  msg.pose.pose.position.x = 20.0;
  msg.pose.pose.position.y = 30.0;
  msg.pose.pose.position.z = 10.0;
  msg.pose.pose.orientation.w = 1.0;
  const auto transform = Access::debug_tf(*node, msg);
  EXPECT_EQ(transform.header.frame_id, "laser_reference_test/cm_odometry_debug");
  EXPECT_EQ(transform.child_frame_id, "laser_reference_test/fcu");
  EXPECT_EQ(transform.header.stamp, msg.header.stamp);
  EXPECT_DOUBLE_EQ(transform.transform.translation.x, 20.0);
  EXPECT_DOUBLE_EQ(transform.transform.translation.y, 30.0);
  EXPECT_DOUBLE_EQ(transform.transform.translation.z, 10.0);
  EXPECT_EQ(transform.transform.rotation, msg.pose.pose.orientation);
  Access::odometry(*node, 20.0, 10.0, 0.0, 30.0);
  EXPECT_EQ(Access::tick(*node, 121.0), held);
}

}  // namespace laser_uav_managers

namespace laser_uav_managers
{
TEST_F(ReferenceHorizonsTest, PmmFirstTerminalWindowIsStationaryAndAccepted)
{
  hover();
  planner("pmm");
  ASSERT_TRUE(Planning::goal(*planning, 2.0));
  Planning::wait(*planning);
  const double end = Planning::end_time(*planning) + 0.001;
  set_time(*planning, end);
  Planning::ready(*planning, Access::flight(*node), 2.0);
  const auto raw = planning_tick();
  ASSERT_TRUE(raw.terminal);
  EXPECT_TRUE(Access::validate(*node, raw));
  set_time(*node, end);
  Access::receive(*node, raw);
  EXPECT_TRUE(Access::has_external(*node));
}

TEST_F(ReferenceHorizonsTest, MincoFirstTerminalWindowIsStationaryAndAccepted)
{
  hover();
  planner("minco");
  ASSERT_TRUE(Planning::goal(*planning, 2.0));
  Planning::wait(*planning);
  const double end = Planning::end_time(*planning) + 0.001;
  set_time(*planning, end);
  Planning::ready(*planning, Access::flight(*node), 2.0);
  const auto raw = planning_tick();
  ASSERT_TRUE(raw.terminal);
  EXPECT_TRUE(Access::validate(*node, raw));
  set_time(*node, end);
  Access::receive(*node, raw);
  EXPECT_TRUE(Access::has_external(*node));
}

TEST_F(ReferenceHorizonsTest, LandingClearsCruiseTiltAndIndividualThrust)
{
  hover();
  auto msg = window();
  const Eigen::Quaterniond q(
    Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(0.45, Eigen::Vector3d::UnitY()));
  for (auto & ref : msg.reference_horizon) {
    ref.pose.orientation.w = q.w();
    ref.pose.orientation.x = q.x();
    ref.pose.orientation.y = q.y();
    ref.pose.orientation.z = q.z();
    ref.use_individual_thrust = true;
    ref.individual_thrust.data = {6.0, 6.0, 6.0, 6.0};
    ref.individual_thrust.unit_of_measurement = "N";
  }
  ASSERT_TRUE(Access::validate(*node, msg));
  Access::receive(*node, msg);
  Access::tick(*node, 120.0);
  ASSERT_TRUE(Access::request(*node, true));
  for (double time : {121.0, 125.0, 150.0}) {
    const auto refs = Access::tick(*node, time);
    for (const auto & ref : refs) {
      const auto & orientation = ref.pose.orientation;
      const Eigen::Quaterniond actual(
        orientation.w, orientation.x, orientation.y, orientation.z);
      EXPECT_NEAR((actual * Eigen::Vector3d::UnitZ()).head<2>().norm(), 0.0, 1e-12);
      EXPECT_NEAR(ref.pose.orientation.z, std::sin(0.35), 1e-12);
      EXPECT_DOUBLE_EQ(ref.pose.position.x, 1.0);
      EXPECT_FALSE(ref.use_individual_thrust);
      EXPECT_TRUE(ref.individual_thrust.data.empty());
    }
  }
}

TEST_F(ReferenceHorizonsTest, SuperWindowCoversPermittedClockSkew)
{
  hover();
  planner("super");
  ASSERT_TRUE(Planning::goal(*planning, 6.0));
  Planning::receive(*planning, polynomial());
  set_time(*planning, 120.014);
  const auto raw = planning_tick();
  ASSERT_TRUE(Access::validate(*node, raw));
  set_time(*node, 120.005);
  Access::receive(*node, raw);
  EXPECT_TRUE(Access::has_external(*node));
  EXPECT_EQ(Access::tick(*node, 120.005).size(), 21U);
  EXPECT_TRUE(Access::has_external(*node));
}

TEST_F(ReferenceHorizonsTest, SuperFirstTerminalWindowIsStationaryAndAccepted)
{
  hover();
  planner("super");
  ASSERT_TRUE(Planning::goal(*planning, 6.0));
  Planning::receive(*planning, polynomial());
  set_time(*planning, 130.001);
  Planning::ready(*planning, Access::flight(*node), 6.0);
  const auto raw = planning_tick();
  ASSERT_TRUE(raw.terminal);
  EXPECT_TRUE(Access::validate(*node, raw));
  set_time(*node, 130.001);
  Access::receive(*node, raw);
  EXPECT_TRUE(Access::has_external(*node));
}

class PlannerPreemptionTest : public ReferenceHorizonsTest,
  public testing::WithParamInterface<const char *> {};

TEST_P(PlannerPreemptionTest, NewestPathReplacesPendingGotoAndDiscardsItsResult)
{
  hover();
  planner(GetParam());
  ASSERT_TRUE(Planning::goal(*planning, 3.0));
  Planning::wait(*planning);
  const auto previous = planning_tick();
  auto worker = Planning::delayed_worker(*planning);
  Planning::ready(*planning, Access::flight(*node), 2.0, 0.5);
  Planning::goto_goal(*planning, 4.0);
  ASSERT_TRUE(Planning::goal(*planning, 5.0, false, true, 1.0));
  const auto latest = Planning::mission(*planning);
  EXPECT_EQ(planning_tick().mission_id, previous.mission_id);
  worker.set_value({});
  Planning::wait(*planning);
  ASSERT_EQ(Planning::state(*planning), Status::TRACKING);
  EXPECT_EQ(planning_tick().mission_id, latest);
  EXPECT_NEAR(Planning::refs(*planning).front().pose.position.x, 2.0, 1e-8);
  EXPECT_NEAR(Planning::refs(*planning).front().twist.linear.x, 0.5, 1e-8);
  EXPECT_NEAR(Planning::refs(*planning).back().pose.position.x, 6.0, 1e-8);
}

TEST_P(PlannerPreemptionTest, GotoReplacesPathDuringWaypointHold)
{
  hover();
  planner(GetParam());
  ASSERT_TRUE(Planning::goal(*planning, 2.0, true, true));
  Planning::wait(*planning);
  set_time(*planning, Planning::end_time(*planning) + 0.001);
  Planning::ready(*planning, Access::flight(*node), 2.0);
  const auto previous = planning_tick();
  ASSERT_EQ(Planning::waypoint(*planning), 0U);
  Planning::goto_goal(*planning, -2.0);
  Planning::wait(*planning);
  ASSERT_EQ(Planning::state(*planning), Status::TRACKING);
  EXPECT_GT(planning_tick().mission_id, previous.mission_id);
  EXPECT_NEAR(Planning::refs(*planning).back().pose.position.x, -2.0, 1e-8);
}

INSTANTIATE_TEST_SUITE_P(Backends, PlannerPreemptionTest, testing::Values("pmm", "minco"));

TEST_F(ReferenceHorizonsTest, SuperNewestGotoDiscardsThePreviousPendingReplacement)
{
  hover();
  planner("super");
  ASSERT_TRUE(Planning::goal(*planning, 6.0));
  Planning::receive(*planning, polynomial());
  const auto previous = planning_tick();
  set_time(*planning, 120.1);
  Planning::goto_goal(*planning, 8.0);
  set_time(*planning, 120.2);
  Planning::goto_goal(*planning, -2.0);
  const auto latest = Planning::mission(*planning);
  auto superseded = polynomial();
  superseded.header.stamp.nanosec = 100000000;
  superseded.start_wt_pos = 120.1;
  Planning::receive(*planning, superseded);
  EXPECT_EQ(Planning::state(*planning), Status::GENERATING);
  EXPECT_EQ(planning_tick().mission_id, previous.mission_id);
  auto replacement = polynomial();
  replacement.header.stamp.nanosec = 200000000;
  replacement.start_wt_pos = 120.2;
  replacement.coef_pos_x = {-0.3, 1.0};
  Planning::receive(*planning, replacement);
  EXPECT_EQ(Planning::state(*planning), Status::TRACKING);
  EXPECT_EQ(planning_tick().mission_id, latest);
}
}  // namespace laser_uav_managers

namespace laser_uav_managers
{
class SimulationPauseTest : public ReferenceHorizonsTest,
  public testing::WithParamInterface<const char *> {};

TEST_P(SimulationPauseTest, PausedClockKeepsMissionAndReferenceAliveThenResumes)
{
  Access::simulation(*node);
  Access::timers(*node);
  hover();
  planner(GetParam(), true);
  ASSERT_TRUE(Planning::goal(*planning, 3.0));
  if (std::string(GetParam()) == "super") {
    Planning::receive(*planning, polynomial());
  } else {
    Planning::wait(*planning);
  }
  const auto before = planning_tick();
  Access::receive(*node, before);
  const auto held = Access::tick(*node, 120.0);
  ASSERT_TRUE(Access::has_external(*node));
  std::this_thread::sleep_for(std::chrono::milliseconds(600));
  EXPECT_FALSE(Access::timer_ready(*node));
  EXPECT_FALSE(Planning::timer_ready(*planning));
  Planning::tick(*planning);
  EXPECT_EQ(Planning::state(*planning), Status::TRACKING);
  EXPECT_EQ(Access::tick(*node, 120.0), held);
  EXPECT_TRUE(Access::has_external(*node));
  set_time(*planning, 120.01);
  set_time(*node, 120.01);
  Access::receive(*node, planning_tick());
  EXPECT_EQ(Planning::state(*planning), Status::TRACKING);
  EXPECT_EQ(Planning::mission(*planning), before.mission_id);
  EXPECT_EQ(Access::tick(*node, 120.01).size(), held.size());
  EXPECT_TRUE(Access::has_external(*node));
}

INSTANTIATE_TEST_SUITE_P(Backends, SimulationPauseTest, testing::Values("pmm", "minco", "super"));

TEST_F(ReferenceHorizonsTest, SimulationStillTimesOutWhenClockAdvancesWithoutData)
{
  Access::simulation(*node);
  hover();
  planner("pmm", true);
  ASSERT_TRUE(Planning::goal(*planning, 3.0));
  Planning::wait(*planning);
  Access::receive(*node, planning_tick());
  Access::tick(*node, 120.0);
  set_time(*node, 120.6);
  set_time(*planning, 120.6);
  Access::tick(*node, 120.6);
  Planning::tick(*planning);
  EXPECT_FALSE(Access::has_external(*node));
  EXPECT_EQ(Planning::state(*planning), Status::ABORTED);
}

TEST_F(ReferenceHorizonsTest, RealWatchdogsExpireEvenWithAFrozenRosClock)
{
  hover();
  planner("pmm");
  ASSERT_TRUE(Planning::goal(*planning, 3.0));
  Planning::wait(*planning);
  Access::receive(*node, planning_tick());
  Access::tick(*node, 120.0);
  std::this_thread::sleep_for(std::chrono::milliseconds(600));
  Access::tick(*node, 120.0);
  Planning::tick(*planning);
  EXPECT_FALSE(Access::has_external(*node));
  EXPECT_EQ(Planning::state(*planning), Status::ABORTED);
}

TEST_F(ReferenceHorizonsTest, SimulationPlanningDeadlinePausesButStillExpiresInSimulationTime)
{
  Access::simulation(*node);
  hover();
  planner("minco", true);
  ASSERT_TRUE(Planning::goal(*planning, 3.0));
  auto worker = Planning::delayed_worker(*planning);
  Planning::planning_timeout(*planning, 0.1);
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  Planning::tick(*planning);
  EXPECT_EQ(Planning::state(*planning), Status::GENERATING);
  set_time(*planning, 120.2);
  Planning::tick(*planning);
  EXPECT_EQ(Planning::state(*planning), Status::FAILED);
}

TEST_F(ReferenceHorizonsTest, SimulationSuperHeartbeatExpiresWhenSimulationTimeAdvances)
{
  Access::simulation(*node);
  hover();
  planner("super", true);
  ASSERT_TRUE(Planning::goal(*planning, 3.0));
  Planning::receive(*planning, polynomial());
  set_time(*planning, 120.6);
  Planning::ready(*planning, Access::flight(*node));
  Planning::tick(*planning);
  EXPECT_EQ(Planning::state(*planning), Status::ABORTED);
}
}  // namespace laser_uav_managers
