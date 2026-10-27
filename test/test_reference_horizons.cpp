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
    node.last_reference_rx_ = std::chrono::steady_clock::now() - std::chrono::seconds(10);
  }
  static bool has_external(const ControlManagerNode & node) {return node.have_external_reference_;}
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
  static void ready(PlanningManagerNode & node, uint64_t flight, double x = 1.0)
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
    node.control_state_callback(state);
    nav_msgs::msg::Odometry odom;
    odom.header.frame_id = "map";
    odom.pose.pose = state.reference.pose;
    node.odometry_callback(odom);
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
    node.super_last_rx_ = PlanningManagerNode::SteadyClock::now() - std::chrono::seconds(10);
  }
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
      node.control_rx_ = node.odometry_rx_ = PlanningManagerNode::SteadyClock::now();
      node.poll_generation();
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
  void planner(const std::string & backend)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("planner", backend)});
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
  EXPECT_EQ(Access::tick(*node, 123.0).front().pose, tracked.front().pose);
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
