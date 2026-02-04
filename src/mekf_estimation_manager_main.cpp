#include <rclcpp/rclcpp.hpp>
#include "laser_uav_managers/mekf_estimation_manager_node.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<laser_uav_managers::MekfEstimationManager>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}