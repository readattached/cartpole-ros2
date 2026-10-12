#ifndef CARTPOLE_CONTROL__LQR_BALANCER_HPP_
#define CARTPOLE_CONTROL__LQR_BALANCER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <Eigen/Dense>
#include <string>

namespace cartpole_control {

class LQRCartpoleBalancer : public rclcpp::Node {
public:
  explicit LQRCartpoleBalancer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~LQRCartpoleBalancer() = default;

private:
  void declareParams();
  void getParams();
  void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void getLinearizedCartPoleDynamics();
  void solveCARE();
  void controlLoop();
  void publishEffort(double value);

  // ROS 2 Interfaces
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_effort_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Configuration Parameters
  std::string cart_joint_name_{"slider_to_cart"};
  std::string pendulum_joint_name_{"cart_to_pendulum"};

  // State & Gain Matrices
  Eigen::Matrix4d A_;
  Eigen::Vector4d B_;

  Eigen::Matrix4d Q_;
  double R_;

  Eigen::Matrix4d P_;
  Eigen::RowVector4d K_;
  Eigen::Vector4d state_{Eigen::Vector4d::Zero()};

  // System parameters
  double g_;
  double M_;

  double m_;
  double r_;
  double L_;

  double Icom_;
  double Ip_;
  double l_;

  // Gains
  double ke_;
  double k_x_;
  double k_dx_;

  bool joint_states_received_{false};
};

}  // namespace cartpole_control

#endif  // CARTPOLE_CONTROL__LQR_BALANCER_HPP_