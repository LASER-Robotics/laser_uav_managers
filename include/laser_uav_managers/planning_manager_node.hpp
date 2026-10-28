#ifndef LASER_UAV_MANAGERS__PLANNING_MANAGER_NODE_HPP_
#define LASER_UAV_MANAGERS__PLANNING_MANAGER_NODE_HPP_

#include <chrono>

#include <future>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <laser_msgs/msg/pose_with_heading.hpp>
#include <laser_msgs/msg/trajectory_path.hpp>
#include <laser_msgs/msg/uav_control_state.hpp>
#include <laser_msgs/msg/uav_planning_diagnostics.hpp>

#include <laser_msgs/msg/reference_horizon.hpp>
#include <mars_quadrotor_msgs/msg/polynomial_trajectory.hpp>
#include <laser_uav_planners/agile_planner.hpp>

namespace laser_uav_managers
{
class PlanningManagerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit PlanningManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  friend struct PlanningManagerNodeTestAccess;
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using Status = laser_msgs::msg::UavPlanningDiagnostics;
  using Goal = laser_msgs::msg::PoseWithHeading;
  using ControlState = laser_msgs::msg::UavControlState;
  using Polynomial = mars_quadrotor_msgs::msg::PolynomialTrajectory;
  using SteadyClock = std::chrono::steady_clock;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;
  void read_parameters();
  void control_state_callback(const ControlState & msg);
  void odometry_callback(const nav_msgs::msg::Odometry & msg);
  void goto_callback(const Goal & msg);
  void goto_relative_callback(const Goal & msg);
  void trajectory_path_callback(const laser_msgs::msg::TrajectoryPath & msg);
  void super_trajectory_callback(const Polynomial & msg);
  bool store_super_trajectory(const Polynomial & msg, std::string & reason);
  bool matches_super_trajectory(const Polynomial & msg) const;
  double super_end_time() const;
  bool sample_super_trajectory(
    double now, int count, double step, double fallback_yaw,
    std::vector<laser_msgs::msg::ReferenceState> & horizon, bool & terminal) const;
  bool start_mission(const std::vector<Goal> & goals, double speed, bool stop);
  void start_segment();
  void poll_generation();
  void timer_callback();
  void publish_status();
  void abort_mission(const std::string & reason, uint8_t state = Status::ABORTED);
  void reject(const std::string & reason);
  bool control_is_fresh() const;
  bool at_goal(const Goal & goal) const;

  struct Mission
  {
    uint64_t flight_id{0};
    uint64_t id{0};
    bool active{false};
    std::string frame;
    std::vector<Goal> goals;
    std::size_t index{0};
    double speed{0.0};
    bool stop{false};
    rclcpp::Time start{0, 0, RCL_ROS_TIME};
  } mission_;
  struct GeneratedPlan
  {
    std::vector<double> times;
    std::vector<laser_msgs::msg::ReferenceState> states;
    rclcpp::Time start{0, 0, RCL_ROS_TIME};
    double elapsed_ms{0.0};
  };
  static GeneratedPlan generate_minco(
    const laser_msgs::msg::ReferenceState & start, const std::vector<Goal> & goals,
    double speed, double acceleration, double sampling_step, double yaw_rate);
  std::future<GeneratedPlan> generation_;
  uint64_t generation_mission_id_{0};
  uint64_t plan_mission_id_{0};
  uint64_t plan_flight_id_{0};
  uint64_t next_mission_id_{0};
  std::optional<GeneratedPlan> plan_;
  bool plan_final_{true};
  std::optional<rclcpp::Time> dwelling_since_;
  Polynomial super_trajectory_;
  bool have_super_trajectory_{false};
  laser_msgs::msg::ReferenceState super_hold_;
  rclcpp::Time super_goal_time_{0, 0, RCL_ROS_TIME};
  SteadyClock::time_point generation_started_{};
  SteadyClock::time_point super_last_rx_{};
  std::optional<SteadyClock::time_point> last_publication_;
  std::optional<ControlState> control_;
  std::optional<nav_msgs::msg::Odometry> odometry_;
  SteadyClock::time_point control_rx_{};
  SteadyClock::time_point odometry_rx_{};
  rclcpp::Time plan_start_{0, 0, RCL_ROS_TIME};

  laser_uav_planners::multirotor_t physical_;
  laser_uav_planners::pmm_t pmm_parameters_;
  std::string planner_;
  double publish_rate_{100.0};
  double state_timeout_{0.5};
  double planning_timeout_{5.0};
  double super_timeout_{0.5};
  double super_sampling_step_{0.01};
  double position_tolerance_{0.15};
  double heading_tolerance_{0.15};
  double waypoint_hold_time_{3.0};
  double yaw_rate_limit_{1.0};
  double minco_max_velocity_{2.0};
  double minco_max_acceleration_{2.0};
  double minco_sampling_step_{0.01};
  double planning_duration_ms_{0.0};
  double reference_period_ms_{0.0};
  uint8_t mission_state_{Status::IDLE};
  std::string detail_;
  bool active_{false};

  rclcpp::Subscription<ControlState>::SharedPtr sub_control_state_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;
  rclcpp::Subscription<Goal>::SharedPtr sub_goto_;
  rclcpp::Subscription<Goal>::SharedPtr sub_goto_relative_;
  rclcpp::Subscription<laser_msgs::msg::TrajectoryPath>::SharedPtr sub_path_;
  rclcpp::Subscription<Polynomial>::SharedPtr sub_super_;
  rclcpp_lifecycle::LifecyclePublisher<laser_msgs::msg::ReferenceHorizon>::SharedPtr pub_horizon_;
  rclcpp_lifecycle::LifecyclePublisher<Status>::SharedPtr pub_status_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_super_goal_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_cancel_;
};
}  // namespace laser_uav_managers

#endif  // LASER_UAV_MANAGERS__PLANNING_MANAGER_NODE_HPP_
