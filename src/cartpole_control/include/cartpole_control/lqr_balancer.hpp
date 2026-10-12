// Copyright 2026 readattached

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


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