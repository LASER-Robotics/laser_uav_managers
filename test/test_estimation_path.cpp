#include <chrono>
#include <limits>
#include <thread>

#include <gtest/gtest.h>
#include <rcl/time.h>

#include <laser_uav_managers/estimation_manager_node.hpp>

namespace laser_uav_managers
{
struct EstimationManagerTestAccess
{
  static void configure(EstimationManager & node)
  {
    ASSERT_EQ(node.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
    ASSERT_EQ(node.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  }
  static void path(EstimationManager & node, const nav_msgs::msg::Odometry & msg)
  {
    node.publish_odometry_path(msg);
  }
  static void openvins(EstimationManager & node, const nav_msgs::msg::Odometry & msg)
  {
    node.enable_openvins_odom_ = true;
    node.odometry_open_vins_callback(std::make_shared<nav_msgs::msg::Odometry>(msg));
  }
  static void ekf(EstimationManager & node)
  {
    auto stamp = node.get_clock()->now();
    node.publish_odometry(node.odom_pub_, stamp);
  }
  static nav_msgs::msg::Path history(const EstimationManager & node) {return node.odometry_path_;}
  static bool timer_ready(EstimationManager & node) {return node.timer_->is_ready();}
  static void deactivate(EstimationManager & node) {node.on_deactivate(rclcpp_lifecycle::State());}
  static void cleanup(EstimationManager & node) {node.on_cleanup(rclcpp_lifecycle::State());}
};

class EstimationPathTest : public testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("use_sim_time", true),
      rclcpp::Parameter("path.max_poses", 3), rclcpp::Parameter("uav_name", "uav7")});
    options.arguments({"--ros-args", "--params-file", TEST_ESTIMATION_CONFIG_PATH,
      "--params-file", TEST_UAV_CONFIG_PATH, "-r", "__ns:=/estimation_path_test"});
    node = std::make_shared<EstimationManager>(options);
    set_time(120.0);
    EstimationManagerTestAccess::configure(*node);
  }
  void TearDown() override
  {
    observer.reset();
    node.reset();
    rclcpp::shutdown();
  }
  void set_time(double seconds)
  {
    ASSERT_EQ(rcl_enable_ros_time_override(node->get_clock()->get_clock_handle()), RCL_RET_OK);
    ASSERT_EQ(rcl_set_ros_time_override(node->get_clock()->get_clock_handle(),
      static_cast<int64_t>(seconds * 1e9)), RCL_RET_OK);
  }
  nav_msgs::msg::Odometry odometry(double seconds, double x)
  {
    nav_msgs::msg::Odometry msg;
    msg.header.frame_id = "uav7/odometry";
    msg.header.stamp = rclcpp::Time(static_cast<int64_t>(seconds * 1e9), RCL_ROS_TIME);
    msg.pose.pose.position.x = x;
    msg.pose.pose.position.z = 1.5;
    msg.pose.pose.orientation.w = 1.0;
    return msg;
  }
  std::shared_ptr<EstimationManager> node;
  rclcpp::Node::SharedPtr observer;
};

TEST_F(EstimationPathTest, RetainsRecentPublishedPosesWithTheirFrameAndStamps)
{
  for (int i = 0; i < 5; ++i) {
    EstimationManagerTestAccess::path(*node, odometry(120.0 + i * 0.2, i));
  }
  const auto path = EstimationManagerTestAccess::history(*node);
  ASSERT_EQ(path.poses.size(), 3U);
  EXPECT_EQ(path.header.frame_id, "uav7/odometry");
  EXPECT_DOUBLE_EQ(path.poses.front().pose.position.x, 2.0);
  EXPECT_DOUBLE_EQ(path.poses.back().pose.position.x, 4.0);
  EXPECT_EQ(path.poses.back().header, path.header);
}

TEST_F(EstimationPathTest, FiltersDuplicatesHighRateAndInvalidPoses)
{
  auto msg = odometry(120.0, 1.0);
  EstimationManagerTestAccess::path(*node, msg);
  EstimationManagerTestAccess::path(*node, msg);
  EstimationManagerTestAccess::path(*node, odometry(120.01, 2.0));
  msg = odometry(121.0, std::numeric_limits<double>::quiet_NaN());
  EstimationManagerTestAccess::path(*node, msg);
  msg = odometry(121.0, 2.0);
  msg.pose.pose.orientation.w = 0.0;
  EstimationManagerTestAccess::path(*node, msg);
  msg = odometry(121.0, 2.0);
  msg.header.frame_id.clear();
  EstimationManagerTestAccess::path(*node, msg);
  ASSERT_EQ(EstimationManagerTestAccess::history(*node).poses.size(), 1U);
}

TEST_F(EstimationPathTest, FrameChangeOrClockResetStartsANewPath)
{
  EstimationManagerTestAccess::path(*node, odometry(120.0, 1.0));
  auto msg = odometry(120.2, 2.0);
  msg.header.frame_id = "another_map";
  EstimationManagerTestAccess::path(*node, msg);
  auto path = EstimationManagerTestAccess::history(*node);
  ASSERT_EQ(path.poses.size(), 1U);
  EXPECT_EQ(path.header.frame_id, "another_map");
  msg.header.stamp.sec = 10;
  EstimationManagerTestAccess::path(*node, msg);
  path = EstimationManagerTestAccess::history(*node);
  ASSERT_EQ(path.poses.size(), 1U);
  EXPECT_EQ(path.header.stamp.sec, 10);
}

TEST_F(EstimationPathTest, OpenVinsPublishedOdometryFeedsThePath)
{
  EstimationManagerTestAccess::openvins(*node, odometry(120.0, 4.0));
  const auto path = EstimationManagerTestAccess::history(*node);
  ASSERT_EQ(path.poses.size(), 1U);
  EXPECT_EQ(path.header.frame_id, "uav7/odometry");
  EXPECT_DOUBLE_EQ(path.poses.front().pose.position.x, 4.0);
}

TEST_F(EstimationPathTest, EkfPublishedOdometryFeedsThePath)
{
  EstimationManagerTestAccess::ekf(*node);
  const auto path = EstimationManagerTestAccess::history(*node);
  ASSERT_EQ(path.poses.size(), 1U);
  EXPECT_EQ(path.header.frame_id, "uav7/odometry");
  EXPECT_EQ(path.header.stamp.sec, 120);
  EXPECT_NEAR(path.poses.front().pose.orientation.w, 1.0, 1e-12);
}

TEST_F(EstimationPathTest, LateSubscriberReceivesLastPath)
{
  EstimationManagerTestAccess::path(*node, odometry(120.0, 4.0));
  observer = std::make_shared<rclcpp::Node>("path_observer");
  nav_msgs::msg::Path::SharedPtr received;
  auto subscription = observer->create_subscription<nav_msgs::msg::Path>(
    "/estimation_path_test/odometry_path_out", rclcpp::QoS(1).reliable().transient_local(),
    [&](nav_msgs::msg::Path::SharedPtr msg) {received = msg;});
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (!received && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(observer);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_TRUE(received);
  ASSERT_EQ(received->poses.size(), 1U);
  EXPECT_DOUBLE_EQ(received->poses.front().pose.position.x, 4.0);
}

TEST_F(EstimationPathTest, LifecycleCleanupClearsHistory)
{
  EstimationManagerTestAccess::path(*node, odometry(120.0, 4.0));
  EstimationManagerTestAccess::deactivate(*node);
  EstimationManagerTestAccess::path(*node, odometry(120.2, 5.0));
  ASSERT_EQ(EstimationManagerTestAccess::history(*node).poses.size(), 1U);
  EstimationManagerTestAccess::cleanup(*node);
  EXPECT_TRUE(EstimationManagerTestAccess::history(*node).poses.empty());
}

TEST_F(EstimationPathTest, SimulationTimerWaitsForClockInsteadOfPredictingDuringPause)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  EXPECT_FALSE(EstimationManagerTestAccess::timer_ready(*node));
  set_time(120.02);
  EXPECT_TRUE(EstimationManagerTestAccess::timer_ready(*node));
}
}  // namespace laser_uav_managers
