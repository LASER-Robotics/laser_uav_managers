#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <limits>
#include <unistd.h>

#include <gtest/gtest.h>
#include <rcl/time.h>

#include "laser_uav_managers/control_manager_node.hpp"

namespace laser_uav_managers
{
using Horizon = std::vector<laser_msgs::msg::ReferenceState>;
using Polynomial = mars_quadrotor_msgs::msg::PolynomialTrajectory;

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
    // Isolate reference selection from the independent mass-calibration phase.
    node.requested_takeoff_ = false;
    node.takeoff_done_ = true;
    node.diagnostics_.is_fly = true;
  }
  static void receive(ControlManagerNode & node, const Polynomial & msg)
  {
    node.polynomial_trajectory_callback(msg);
  }
  static void age(ControlManagerNode & node)
  {
    node.last_external_rx_steady_ = std::chrono::steady_clock::now() - std::chrono::seconds(10);
  }
  static double age_seconds(const ControlManagerNode & node)
  {
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now() - node.last_external_rx_steady_).count();
  }
  static bool has_external(const ControlManagerNode & node)
  {
    return node.have_external_position_trajectory_;
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
    ASSERT_EQ(rcl_enable_ros_time_override(node.get_clock()->get_clock_handle()), RCL_RET_OK);
  }
  static void control_tick(ControlManagerNode & node, int seconds)
  {
    ASSERT_EQ(
      rcl_set_ros_time_override(
        node.get_clock()->get_clock_handle(), static_cast<int64_t>(seconds) * 1000000000),
      RCL_RET_OK);
    node.external_loop_timer_callback();
  }
  static Horizon horizon(const ControlManagerNode & node) {return node.current_horizon_path_;}
  static bool takeoff_done(const ControlManagerNode & node) {return node.takeoff_done_;}
  static Eigen::Vector3d rates(const ControlManagerNode & node) {return node.nmpc_solution_.first;}
  static void start_benchmark(ControlManagerNode & node, const std::string & path, int seconds)
  {
    node.benchmark_.enabled = true;
    node.benchmark_.csv_path = path;
    ASSERT_EQ(rcl_enable_ros_time_override(node.get_clock()->get_clock_handle()), RCL_RET_OK);
    ASSERT_EQ(
      rcl_set_ros_time_override(
        node.get_clock()->get_clock_handle(), static_cast<int64_t>(seconds) * 1000000000),
      RCL_RET_OK);
    node.benchmark_goal_callback(geometry_msgs::msg::PoseStamped());
  }
  static void benchmark_tick(ControlManagerNode & node, int seconds)
  {
    node.update_benchmark(rclcpp::Time(seconds, 0, RCL_ROS_TIME));
  }
  static void age_benchmark_heartbeat(ControlManagerNode & node)
  {
    node.benchmark_.last_heartbeat = std::chrono::steady_clock::now() - std::chrono::seconds(10);
  }
  static geometry_msgs::msg::TransformStamped debug_tf(
    const ControlManagerNode & node, const nav_msgs::msg::Odometry & msg)
  {
    return node.odometry_debug_transform(msg);
  }
};
using Access = ControlManagerNodeTestAccess;

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
        "--params-file", TEST_UAV_CONFIG_PATH,
        "--params-file", TEST_CONTROLLER_CONFIG_PATH,
        "-r", "__ns:=/laser_reference_test"});
    node = std::make_unique<ControlManagerNode>(options);
    Access::initialize(*node);
    Access::odometry(*node, 1.0, 0.2, 0.7);
  }
  void TearDown() override
  {
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
  static Polynomial trajectory(double duration = 10.0)
  {
    Polynomial msg;
    msg.type = msg.POSITION_TRAJ;
    msg.start_wt_pos = 120.0;
    msg.piece_num_pos = 1;
    msg.order_pos = 1;
    msg.time_pos = {duration};
    msg.coef_pos_x = {0.5, 1.0};
    msg.coef_pos_y = {0.0, -2.0};
    msg.coef_pos_z = {0.0, 1.5};
    return msg;
  }
  std::unique_ptr<ControlManagerNode> node;
  std::string csv_path;
  void prepare_benchmark()
  {
    char path[] = "/tmp/laser_control_manager_csv_test_XXXXXX";
    const int fd = mkstemp(path);
    ASSERT_GE(fd, 0);
    close(fd);
    csv_path = path;
    Access::odometry(*node, 1.0, 1.5, 0.7);
    Access::start_benchmark(*node, csv_path, 120);
  }
  std::string csv_contents()
  {
    std::ifstream input(csv_path);
    return std::string(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
  }
};

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

TEST_F(ReferenceHorizonsTest, SuperTrackingAfterTakeoffUsesExternalPolynomialAndFrozenYaw)
{
  hover();
  Access::odometry(*node, 20.0, 10.0, 2.0);
  Access::receive(*node, trajectory());
  const auto refs = Access::tick(*node, 122.0);
  for (std::size_t i = 0; i < refs.size(); ++i) {
    EXPECT_NEAR(refs[i].pose.position.x, 2.0 + 0.025 * i, 1e-10);
    EXPECT_DOUBLE_EQ(refs[i].twist.linear.x, 0.5);
    EXPECT_NEAR(refs[i].pose.orientation.z, std::sin(0.35), 1e-12);
    EXPECT_TRUE(refs[i].use_orientation);
  }
  auto msg = trajectory();
  msg.type |= msg.YAW_COMMAND;
  msg.yaw = 1.2;
  msg.yaw_rate = 0.1;
  Access::receive(*node, msg);
  const auto with_yaw = Access::tick(*node, 122.0);
  EXPECT_NEAR(with_yaw.front().pose.orientation.z, std::sin(0.6), 1e-12);
  EXPECT_DOUBLE_EQ(with_yaw.front().twist.angular.z, 0.1);
}

TEST_F(ReferenceHorizonsTest, SamplesSuperSeventhOrderPositionAndYawAcrossPieces)
{
  hover();
  auto msg = trajectory();
  msg.header.frame_id = "world";
  msg.order_pos = 7;
  msg.piece_num_pos = 2;
  msg.time_pos = {2.0, 3.0};
  msg.coef_pos_x = {0, 0, 0, 0, 0, 0, 0.5, 1.0, 0, 0, 0, 0, 0, 0, 0.5, 2.0};
  msg.coef_pos_y = {0, 0, 0, 0, 0, 0, 0, -2.0, 0, 0, 0, 0, 0, 0, 0, -2.0};
  msg.coef_pos_z = {0, 0, 0, 0, 0, 0, 0, 1.5, 0, 0, 0, 0, 0, 0, 0, 1.5};
  msg.type |= msg.YAW_TRAJ;
  msg.start_wt_yaw = 120.0;
  msg.order_yaw = 7;
  msg.piece_num_yaw = 1;
  msg.time_yaw = {5.0};
  msg.coef_yaw = {0, 0, 0, 0, 0, 0, 0.1, 0.7};
  Access::receive(*node, msg);
  const auto refs = Access::tick(*node, 122.5);
  for (std::size_t i = 0; i < refs.size(); ++i) {
    EXPECT_NEAR(refs[i].pose.position.x, 2.25 + 0.025 * i, 1e-10);
    EXPECT_NEAR(refs[i].pose.orientation.z, std::sin((0.95 + 0.005 * i) / 2), 1e-10);
    EXPECT_NEAR(refs[i].twist.angular.z, 0.1, 1e-12);
  }
  const auto terminal = Access::tick(*node, 126.0);
  EXPECT_DOUBLE_EQ(terminal.front().pose.position.x, 3.5);
  EXPECT_NEAR(terminal.front().pose.orientation.z, std::sin(0.6), 1e-12);
  EXPECT_DOUBLE_EQ(terminal.front().twist.angular.z, 0.0);
}

TEST_F(ReferenceHorizonsTest, LandingAfterExternalTrackingStartsAtLastCommand)
{
  hover();
  Access::receive(*node, trajectory());
  const auto tracking = Access::tick(*node, 122.0);
  Access::odometry(*node, 20.0, 10.0, 2.0);
  ASSERT_TRUE(Access::request(*node, true));
  const auto landing = Access::tick(*node, 123.0);
  EXPECT_EQ(landing.front().pose, tracking.front().pose);
  EXPECT_DOUBLE_EQ(landing.front().twist.linear.x, 0.0);
}

TEST_F(ReferenceHorizonsTest, TimeoutHoldsLastCommandInsteadOfCurrentOdometry)
{
  hover();
  Access::receive(*node, trajectory());
  const auto tracking = Access::tick(*node, 122.0);
  Access::odometry(*node, 20.0, 10.0, 2.0);
  Access::age(*node);
  const auto held = Access::tick(*node, 122.1);
  EXPECT_EQ(held.front().pose, tracking.front().pose);
  EXPECT_DOUBLE_EQ(held.front().twist.linear.x, 0.0);
  for (int i = 0; i < 20; ++i) {
    Access::odometry(*node, 20.0 + i, 10.0 + i, 0.1 * i);
    EXPECT_EQ(Access::tick(*node, 122.1 + i * 0.1), held);
  }
  Access::receive(*node, trajectory());
  EXPECT_DOUBLE_EQ(Access::tick(*node, 125.0).front().pose.position.x, 3.5);
}

TEST_F(ReferenceHorizonsTest, FinishedTrajectoryHoldsEndpointEvenWhenEstimateIsFarAndHeartbeatStops)
{
  hover();
  Access::receive(*node, trajectory(2.0));
  Access::tick(*node, 121.0);
  Access::age(*node);
  Access::odometry(*node, 20.0, 10.0, 2.0);
  const auto terminal = Access::tick(*node, 123.0);
  EXPECT_DOUBLE_EQ(terminal.front().pose.position.x, 2.0);
  EXPECT_DOUBLE_EQ(terminal.front().twist.linear.x, 0.0);
  Access::odometry(*node, 30.0, 15.0, -1.0);
  EXPECT_EQ(Access::tick(*node, 150.0), terminal);
  Access::odometry(*node, 2.0, 1.5, 0.7);
  EXPECT_EQ(Access::tick(*node, 151.0), terminal);
  EXPECT_FALSE(Access::has_external(*node));
  Access::odometry(*node, 40.0, 20.0, -2.0);
  EXPECT_EQ(Access::tick(*node, 152.0), terminal);
}

TEST_F(ReferenceHorizonsTest, EmergencyStopHoldsCommandAndNewPolynomialResumesTracking)
{
  hover();
  Access::receive(*node, trajectory());
  const auto tracking = Access::tick(*node, 121.0);
  Polynomial stop;
  stop.type = stop.EMER_STOP;
  Access::receive(*node, stop);
  Access::odometry(*node, 20.0, 10.0, 2.0);
  const auto held = Access::tick(*node, 122.0);
  EXPECT_EQ(held.front().pose, tracking.front().pose);
  Access::receive(*node, stop);
  Access::odometry(*node, 30.0, 15.0, -1.0);
  EXPECT_EQ(Access::tick(*node, 123.0), held);
  Access::receive(*node, trajectory());
  EXPECT_DOUBLE_EQ(Access::tick(*node, 124.0).front().pose.position.x, 3.0);
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

TEST_F(ReferenceHorizonsTest, ExternalPolynomialsCannotInterruptLocalManeuvers)
{
  Access::receive(*node, trajectory());
  EXPECT_FALSE(Access::has_external(*node));
  ASSERT_TRUE(Access::request(*node));
  const auto takeoff = Access::tick(*node, 100.0);
  Access::receive(*node, trajectory());
  EXPECT_TRUE(Access::has_external(*node));
  EXPECT_EQ(Access::tick(*node, 100.0), takeoff);
  Access::tick(*node, 110.0);
  Access::flying(*node);
  Access::receive(*node, trajectory());
  EXPECT_TRUE(Access::has_external(*node));
  ASSERT_TRUE(Access::request(*node, true));
  Access::receive(*node, trajectory());
  EXPECT_FALSE(Access::has_external(*node));
}

TEST_F(ReferenceHorizonsTest, PolynomialReceivedDuringTakeoffIsUsedAfterTakeoff)
{
  ASSERT_TRUE(Access::request(*node));
  Access::tick(*node, 100.0);
  Access::receive(*node, trajectory());
  Access::tick(*node, 110.0);
  Access::flying(*node);
  Polynomial heartbeat;
  heartbeat.type = heartbeat.HEART_BEAT;
  heartbeat.start_wt_pos = 120.0;
  Access::age(*node);
  Access::receive(*node, heartbeat);
  EXPECT_DOUBLE_EQ(Access::tick(*node, 122.0).front().pose.position.x, 2.0);
}

TEST_F(ReferenceHorizonsTest, OnlyMatchingSuperHeartbeatRefreshesTrajectory)
{
  hover();
  Access::receive(*node, trajectory());
  Polynomial heartbeat;
  heartbeat.type = heartbeat.HEART_BEAT;
  heartbeat.start_wt_pos = 120.0;
  Access::age(*node);
  Access::receive(*node, heartbeat);
  EXPECT_LT(Access::age_seconds(*node), 0.5);
  heartbeat.start_wt_pos = 121.0;
  Access::age(*node);
  Access::receive(*node, heartbeat);
  EXPECT_GT(Access::age_seconds(*node), 9.0);
}

TEST_F(ReferenceHorizonsTest, RejectsInvalidPolynomialsWithoutChangingHold)
{
  hover();
  const auto held = Access::tick(*node, 120.0);
  auto msg = trajectory();
  msg.time_pos[0] = -1.0;
  Access::receive(*node, msg);
  EXPECT_FALSE(Access::has_external(*node));
  msg = trajectory();
  msg.coef_pos_x[0] = std::numeric_limits<double>::quiet_NaN();
  Access::receive(*node, msg);
  EXPECT_FALSE(Access::has_external(*node));
  EXPECT_EQ(Access::tick(*node, 121.0), held);
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

TEST_F(ReferenceHorizonsTest, BenchmarkAppendsCsvOnceAfterGoalAndHeartbeatStop)
{
  hover();
  prepare_benchmark();
  auto msg = trajectory(2.0);
  msg.type |= msg.HEART_BEAT;
  Access::receive(*node, msg);
  Access::odometry(*node, 1.5, 1.5, 0.7);
  Access::tick(*node, 121.0);
  Access::odometry(*node, 2.0, 1.5, 0.7);
  const auto held = Access::tick(*node, 123.0);
  EXPECT_FALSE(Access::has_external(*node));
  Access::benchmark_tick(*node, 123);
  EXPECT_TRUE(csv_contents().empty());
  Access::age_benchmark_heartbeat(*node);
  Access::benchmark_tick(*node, 124);
  const std::string header = "run_id,start_time_s,end_time_s,total_time_s,distance_m,"
    "average_speed_mps,waypoint_count,start_x,start_y,start_z,end_x,end_y,end_z\n";
  const std::string first_row = "1,120.000000,124.000000,4.000000,1.000000,0.250000,1,"
    "1.000000,-2.000000,1.500000,2.000000,-2.000000,1.500000\n";
  EXPECT_EQ(csv_contents(), header + first_row);
  Access::benchmark_tick(*node, 125);
  EXPECT_EQ(csv_contents(), header + first_row);
  EXPECT_EQ(Access::tick(*node, 125.0), held);

  Access::start_benchmark(*node, csv_path, 130);
  Access::benchmark_tick(*node, 133);
  EXPECT_EQ(csv_contents(), header + first_row);
  msg.start_wt_pos = 130.0;
  Access::receive(*node, msg);
  Access::age_benchmark_heartbeat(*node);
  Access::benchmark_tick(*node, 134);
  const auto result = csv_contents();
  EXPECT_EQ(std::count(result.begin(), result.end(), '\n'), 3);
  EXPECT_EQ(result.find(header, header.size()), std::string::npos);
  EXPECT_NE(result.find("\n2,130.000000,134.000000,"), std::string::npos);
}

TEST_F(ReferenceHorizonsTest, BenchmarkRequiresMatchingHeartbeatAndCancelsOnLanding)
{
  hover();
  prepare_benchmark();
  auto msg = trajectory(2.0);
  msg.type |= msg.HEART_BEAT;
  Access::receive(*node, msg);
  Access::age_benchmark_heartbeat(*node);
  Polynomial heartbeat;
  heartbeat.type = heartbeat.HEART_BEAT;
  heartbeat.start_wt_pos = 120.0;
  Access::receive(*node, heartbeat);
  Access::odometry(*node, 2.0, 1.5, 0.7);
  Access::benchmark_tick(*node, 124);
  EXPECT_TRUE(csv_contents().empty());
  ASSERT_TRUE(Access::request(*node, true));
  Access::age_benchmark_heartbeat(*node);
  Access::benchmark_tick(*node, 130);
  EXPECT_TRUE(csv_contents().empty());
}

}  // namespace laser_uav_managers
