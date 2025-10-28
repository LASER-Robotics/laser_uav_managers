#ifndef LASER_UAV_MANAGERS__ESTIMATION_MANAGER_NODE_HPP_
#define LASER_UAV_MANAGERS__ESTIMATION_MANAGER_NODE_HPP_

/* includes //{ */
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <laser_msgs/msg/uav_control_diagnostics.hpp>
#include <laser_msgs/msg/estimation_manager_diagnostics.hpp>
#include <laser_msgs/msg/sensor_status.hpp>
#include <laser_msgs/srv/set_string.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <Eigen/Dense>
#include <mutex>
#include <memory>
#include <vector>
#include <string>
#include <map>
#include <deque>
#include <chrono>
#include <laser_uav_estimators/state_estimator.hpp>
/*//}*/

/* define //{*/
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
/*//}*/

namespace laser_uav_managers
{
template <typename MsgT>
struct SensorDataBuffer
{
  SensorDataBuffer() : tolerance(0, 0), timeout(0, 0) {
  }

  std::map<rclcpp::Time, typename MsgT::SharedPtr> buffer;
  std::mutex                                       mtx;
  rclcpp::Duration                                 tolerance;
  rclcpp::Duration                                 timeout;
  bool                                             is_active{false};
};

class EstimationManager : public rclcpp_lifecycle::LifecycleNode {
public:
  explicit EstimationManager(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  ~EstimationManager() override;

private:
  /* CONFIG //{ */
  CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

  void getParameters();

  void configPubSub();

  void configTimers();

  void configServices();
  /*//}*/

  /* CALLBACKS //{ */
  void odometryPx4Callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  void odometryOpenVinsCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  void odometryFastLioCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  void controlCallback(const laser_msgs::msg::UavControlDiagnostics::SharedPtr msg);

  void timerCallback();

  void checkSubscribersCallback();

  void diagnosticsTimerCallback();
  /*//}*/

  void setOdometryCallback(const std::shared_ptr<laser_msgs::srv::SetString::Request> request, std::shared_ptr<laser_msgs::srv::SetString::Response> response);

  rclcpp::Service<laser_msgs::srv::SetString>::SharedPtr set_odometry_service_;

  /* FUNCTIONS //{ */
  void setupEKF();

  void publishOdometry(rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr pub);

  template <typename MsgT>
  std::optional<MsgT> getSynchronizedMessage(const rclcpp::Time &ref_time, SensorDataBuffer<MsgT> &sensor_data, std::string sensor_name);

  template <typename MsgT>
  void pruneSensorBuffer(const rclcpp::Time &now, SensorDataBuffer<MsgT> &sensor_data);
  /*//}*/

  /* EKF //{ */
  std::unique_ptr<laser_uav_estimators::StateEstimator> ekf_;
  /*//}*/

  /* ROS COMMUNICATIONS //{ */
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr                       odom_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr                       predict_pub_;
  rclcpp_lifecycle::LifecyclePublisher<laser_msgs::msg::EstimationManagerDiagnostics>::SharedPtr diagnostics_pub_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr                odometry_px4_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr                odometry_openvins_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr                odometry_fast_lio_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr                  imu_sub_;
  rclcpp::Subscription<laser_msgs::msg::UavControlDiagnostics>::SharedPtr control_sub_;
  rclcpp::TimerBase::SharedPtr                                            timer_;
  rclcpp::TimerBase::SharedPtr                                            check_subscribers_timer_;
  rclcpp::TimerBase::SharedPtr                                            diagnostics_timer_;
  /*//}*/

  /* STATE & THREAD-SAFETY //{ */
  std::mutex                            mtx_;
  rclcpp::Time                          last_update_time_;
  rclcpp::Time                          last_px4_odom_time_;
  rclcpp::Time                          last_openvins_odom_time_;
  rclcpp::Time                          last_fast_lio_odom_time_;
  rclcpp::Time                          last_imu_time_;
  rclcpp::Time                          last_control_input_time_;
  std::chrono::steady_clock::time_point last_cpp_time_point_;

  SensorDataBuffer<nav_msgs::msg::Odometry>                px4_odom_data_;
  SensorDataBuffer<nav_msgs::msg::Odometry>                openvins_odom_data_;
  SensorDataBuffer<nav_msgs::msg::Odometry>                fast_lio_odom_data_;
  SensorDataBuffer<sensor_msgs::msg::Imu>                  imu_data_;
  SensorDataBuffer<laser_msgs::msg::UavControlDiagnostics> control_data_;

  nav_msgs::msg::Odometry::SharedPtr                last_odometry_px4_msg_;
  nav_msgs::msg::Odometry::SharedPtr                last_odometry_openvins_msg_;
  nav_msgs::msg::Odometry::SharedPtr                last_odometry_fast_lio_msg_;
  sensor_msgs::msg::Imu::SharedPtr                  last_imu_msg_;
  laser_msgs::msg::UavControlDiagnostics::SharedPtr last_control_input_;

  bool is_active_{false};
  bool is_control_input_{false};
  bool is_initialized_{false};
  bool is_first_control_msg{false};
  bool enable_px4_odom_{false};
  bool enable_openvins_odom_{false};
  bool enable_fast_lio_odom_{false};

  std::string              current_active_odometry_name_{"NONE"};
  std::vector<std::string> odometry_source_names_;
  double                   odometry_switch_distance_threshold_;
  double                   odometry_switch_angle_threshold_;
  double                   odometry_switch_velocity_linear_threshold_;
  double                   odometry_switch_velocity_angular_threshold_;
  /*//}*/

  /* PARAMETERS //{ */
  double              frequency_;
  double              sensor_timeout_;
  std::string         ekf_verbosity_;
  double              mass_;
  double              arm_length_;
  double              thrust_coefficient_;
  double              torque_coefficient_;
  std::vector<double> inertia_vec_;
  Eigen::MatrixXd     allocation_matrix_;
  std::vector<double> process_noise_vec_;

  laser_uav_estimators::ProcessNoiseGains     process_noise_gains_;
  laser_uav_estimators::MeasurementNoiseGains measurement_noise_gains_;

  double px4_odom_tolerance_;
  double px4_odom_timeout_;
  double px4_odom_covariance_;
  double openvins_odom_tolerance_;
  double openvins_odom_timeout_;
  double openvins_odom_covariance_;
  double fast_lio_odom_tolerance_;
  double fast_lio_odom_timeout_;
  double fast_lio_odom_covariance_;
  double imu_tolerance_;
  double imu_timeout_;
  double imu_covariance_;
  double control_tolerance_;
  double control_timeout_;

  /*//}*/
};
}  // namespace laser_uav_managers

#endif  // LASER_UAV_MANAGERS__ESTIMATION_MANAGER_NODE_HPP_