#include <rclcpp/rclcpp.hpp>
#include "cartpole_control/lqr_balancer.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cartpole_control::LQRCartpoleBalancer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}