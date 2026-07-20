#ifndef LASER_UAV_MANAGERS__ESTIMATION_MANAGER_NODE_HPP_
#define LASER_UAV_MANAGERS__ESTIMATION_MANAGER_NODE_HPP_

/* includes //{ */
#include <chrono>

#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/range.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <laser_msgs/msg/estimation_manager_diagnostics.hpp>
#include <laser_msgs/msg/motor_speed_stamped.hpp>
#include <laser_msgs/msg/sensor_status.hpp>
#include <laser_msgs/msg/uav_control_diagnostics.hpp>
#include <laser_msgs/srv/set_string.hpp>

#include <laser_uav_estimators/mekf_state_estimator.hpp>
#include <lifecycle_msgs/msg/state.hpp>
/*//}*/

/* define //{*/
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
/*//}*/

namespace laser_uav_managers
{
/**
 * @brief Thread-safe temporal buffer for synchronizing asynchronous sensor messages
 * with state estimation predictions.
 */
template <typename MsgT>
struct SensorDataBuffer
{
  SensorDataBuffer() : tolerance(0, 0), timeout(0, 0)
  {}

  std::map<rclcpp::Time, typename MsgT::SharedPtr> buffer;
  std::mutex mtx;
  rclcpp::Duration tolerance;
  rclcpp::Duration timeout;
  bool is_active{false};
  typename MsgT::SharedPtr last_msg{nullptr};
};

/**
 * @brief Lifecycle node responsible for multi-sensor state estimation using a
 * Multiplicative Extended Kalman Filter (MEKF). Supports online odometry source switching.
 */
class EstimationManager : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit EstimationManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~EstimationManager() override;

private:
  /* CONFIG //{ */
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  void get_parameters();
  void configure_pub_sub();
  void configure_timers();
  void configure_services();
  /*//}*/

  /* CALLBACKS //{ */
  void odometry_px4_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void odometry_openvins_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void odometry_fast_lio_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void garmin_range_callback(const sensor_msgs::msg::Range::SharedPtr msg);
  void control_callback(const laser_msgs::msg::UavControlDiagnostics::SharedPtr msg);
  void timer_callback();
  void check_subscribers_callback();
  void diagnostics_timer_callback();
  void set_odometry_callback(
    const std::shared_ptr<laser_msgs::srv::SetString::Request> request,
    std::shared_ptr<laser_msgs::srv::SetString::Response> response);
  /*//}*/

  rclcpp::Service<laser_msgs::srv::SetString>::SharedPtr set_odometry_service_;

  /* FUNCTIONS //{ */
  void setup_ekf();
  void publish_odometry(
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr pub,
    rclcpp::Time & pub_time);

  template <typename MsgT>
  bool is_buffer_valid(
    SensorDataBuffer<MsgT> & sensor_buffer, const std::string & sensor_name,
    const rclcpp::Time & reference_time, rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock);

  void set_verbosity(const std::string & verbosity);
  /*//}*/

  /* EKF //{ */
  std::unique_ptr<laser_uav_estimators::MEKFEstimator> mekf_;
  /*//}*/

  /* ROS COMMUNICATIONS //{ */
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp_lifecycle::LifecyclePublisher<laser_msgs::msg::EstimationManagerDiagnostics>::SharedPtr
    diagnostics_pub_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_px4_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_openvins_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_fast_lio_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr garmin_sub_;
  rclcpp::Subscription<laser_msgs::msg::UavControlDiagnostics>::SharedPtr control_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr check_subscribers_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  /*//}*/

  /* STATE & THREAD-SAFETY //{ */
  std::mutex mtx_;
  rclcpp::Time last_update_time_;
  rclcpp::Time last_px4_odom_time_;
  rclcpp::Time last_openvins_odom_time_;
  rclcpp::Time last_fast_lio_odom_time_;
  rclcpp::Time last_garmin_range_time_;
  rclcpp::Time last_control_input_time_;
  std::chrono::steady_clock::time_point last_cpp_time_point_;

  SensorDataBuffer<nav_msgs::msg::Odometry> px4_odom_data_;
  SensorDataBuffer<nav_msgs::msg::Odometry> openvins_odom_data_;
  SensorDataBuffer<nav_msgs::msg::Odometry> fast_lio_odom_data_;
  SensorDataBuffer<laser_msgs::msg::UavControlDiagnostics> control_data_;
  SensorDataBuffer<sensor_msgs::msg::Range> garmin_data_;

  nav_msgs::msg::Odometry::SharedPtr last_odometry_px4_msg_;
  nav_msgs::msg::Odometry::SharedPtr last_odometry_openvins_msg_;
  nav_msgs::msg::Odometry::SharedPtr last_odometry_fast_lio_msg_;
  laser_msgs::msg::UavControlDiagnostics::SharedPtr last_control_msg_;
  sensor_msgs::msg::Range::SharedPtr last_garmin_range_msg_;

  bool is_predicted_{false};
  bool is_active_{false};
  bool is_ekf_active_{false};
  bool is_control_input_{false};
  bool is_initialized_{false};
  bool is_first_control_msg_{false};
  bool enable_px4_odom_{false};
  bool enable_openvins_odom_{false};
  bool enable_fast_lio_odom_{false};

  bool garmin_offset_calibrated_{false};
  double garmin_calibrated_offset_{-1.0};
  int garmin_counter_{0};

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string current_active_odometry_name_{"NONE"};
  std::vector<std::string> odometry_source_names_;
  double odometry_switch_distance_threshold_;
  double odometry_switch_angle_threshold_;
  double odometry_switch_velocity_linear_threshold_;
  double odometry_switch_velocity_angular_threshold_;
  /*//}*/

  /* PARAMETERS //{ */
  std::string uav_name_;
  double frequency_;
  double sensor_timeout_;
  std::string estimation_verbosity_;
  std::string ekf_verbosity_;
  double mass_;
  double arm_length_;
  double thrust_coefficient_;
  double torque_coefficient_;
  std::vector<double> inertia_vec_;
  Eigen::MatrixXd allocation_matrix_;
  std::vector<double> process_noise_vec_;

  laser_uav_estimators::NoiseGains process_noise_gains_;
  laser_uav_estimators::MeasurementNoiseGains px4_measurement_noise_gains_;
  laser_uav_estimators::MeasurementNoiseGains openvins_measurement_noise_gains_;
  laser_uav_estimators::MeasurementNoiseGains fast_lio_measurement_noise_gains_;
  laser_uav_estimators::MeasurementNoiseGains garmin_measurement_noise_gains_;

  double px4_odom_covariance_;
  double openvins_odom_covariance_;
  double fast_lio_odom_covariance_;
  double garmin_covariance_;

  double imu_tolerance_;
  double imu_timeout_;
  double imu_covariance_;
  double control_tolerance_;
  double control_timeout_;
  /*//}*/
};
}  // namespace laser_uav_managers

#endif  // LASER_UAV_MANAGERS__ESTIMATION_MANAGER_NODE_HPP_
