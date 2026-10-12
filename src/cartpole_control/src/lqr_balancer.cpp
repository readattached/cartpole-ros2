#include "cartpole_control/lqr_balancer.hpp"

#include <algorithm>
#include <cmath>


namespace cartpole_control {

LQRCartpoleBalancer::LQRCartpoleBalancer(const rclcpp::NodeOptions & options)
: Node("lqr_cartpole_balancer", options) {

  LQRCartpoleBalancer::declareParams();
  LQRCartpoleBalancer::getParams();

  LQRCartpoleBalancer::getLinearizedCartPoleDynamics();
  LQRCartpoleBalancer::solveCARE();

  sub_joint_states_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    std::bind(&LQRCartpoleBalancer::jointStatesCallback, this, std::placeholders::_1));

  pub_effort_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/cart_effort_controller/commands", 10);

  // 1000 Hz control loop timer (1ms)
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1),
    std::bind(&LQRCartpoleBalancer::controlLoop, this));

  RCLCPP_INFO(this->get_logger(), "LQR Cartpole Balancer Node initialized at 1000 Hz.");
}

void LQRCartpoleBalancer::declareParams() {
  this->declare_parameter<double>("g", 0.0);
  this->declare_parameter<double>("M", 0.0);
  this->declare_parameter<double>("m", 0.0);
  this->declare_parameter<double>("r", 0.0);
  this->declare_parameter<double>("L", 0.0);

  this->declare_parameter<std::vector<double>>("Q", {10.0, 100.0, 1.0, 10.0});
  this->declare_parameter<double>("R", 0.0);

  this->declare_parameter<double>("ke", 0.0);
  this->declare_parameter<double>("k_x", 0.0);
  this->declare_parameter<double>("k_dx", 0.0);
}

void LQRCartpoleBalancer::getParams() {
  std::vector<double> q_diag;

  this->get_parameter("g", g_);
  this->get_parameter("M", M_);
  this->get_parameter("m", m_);
  this->get_parameter("r", r_);
  this->get_parameter("L", L_);

  this->get_parameter("Q", q_diag);
  this->get_parameter("R", R_);

  this->get_parameter("ke", ke_);
  this->get_parameter("k_x", k_x_);
  this->get_parameter("k_dx", k_dx_);

  Q_ = Eigen::Map<Eigen::Vector4d>(q_diag.data()).asDiagonal();

  RCLCPP_INFO(this->get_logger(), "Loaded g: %f", g_);
  RCLCPP_INFO(this->get_logger(), "Loaded M: %f", M_);
  RCLCPP_INFO(this->get_logger(), "Loaded m: %f", m_);
  RCLCPP_INFO(this->get_logger(), "Loaded r: %f", r_);
  RCLCPP_INFO(this->get_logger(), "Loaded L: %f", L_);

  RCLCPP_INFO(this->get_logger(), "Loaded Q: [%f, %f, %f, %f]", Q_(0), Q_(5), Q_(10), Q_(15));
  RCLCPP_INFO(this->get_logger(), "Loaded R: %f", R_);

  RCLCPP_INFO(this->get_logger(), "Loaded ke: %f", ke_);
  RCLCPP_INFO(this->get_logger(), "Loaded k_x: %f", k_x_);
  RCLCPP_INFO(this->get_logger(), "Loaded k_dx: %f", k_dx_);
}

void LQRCartpoleBalancer::jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  auto cart_it = std::find(msg->name.begin(), msg->name.end(), cart_joint_name_);
  auto pend_it = std::find(msg->name.begin(), msg->name.end(), pendulum_joint_name_);

  if (cart_it != msg->name.end() && pend_it != msg->name.end()) {
    size_t cart_idx = std::distance(msg->name.begin(), cart_it);
    size_t pend_idx = std::distance(msg->name.begin(), pend_it);

    double x = msg->position[cart_idx];
    double theta = msg->position[pend_idx];
    double dx = (msg->velocity.size() > cart_idx) ? msg->velocity[cart_idx] : 0.0;
    double dtheta = (msg->velocity.size() > pend_idx) ? msg->velocity[pend_idx] : 0.0;

    theta = std::atan2(std::sin(theta), std::cos(theta));

    state_ << x, theta, dx, dtheta;
    joint_states_received_ = true;
  }
}

void LQRCartpoleBalancer::getLinearizedCartPoleDynamics() {
  l_ = L_ / 2;
  Icom_ = (1.0 / 12.0) * m_ * ((3.0 * r_ * r_) + (L_ * L_));
  Ip_ = Icom_ + (m_ * l_ *l_);
  double denominator = Ip_ * (M_ + m_) - (m_ * m_ * l_ * l_);

  A_.setZero();
  A_(0, 2) = 1.0;
  A_(1, 3) = 1.0;
  A_(2, 1) = -(m_ * m_ * g_ * l_ * l_) / denominator;
  A_(3, 1) = (m_ * g_ * l_ *(M_ + m_)) / denominator;

  B_.setZero();
  B_(2) = Ip_ / denominator;
  B_(3) = -(m_ * l_) / denominator;
}

void LQRCartpoleBalancer::solveCARE() {
  // 1. Construct the 8x8 Hamiltonian Matrix H
  Eigen::Matrix<double, 8, 8> H;
  Eigen::Matrix4d BRinvBT = (1.0 / R_) * B_ * B_.transpose();

  H.block<4, 4>(0, 0) = A_;
  H.block<4, 4>(0, 4) = -BRinvBT;
  H.block<4, 4>(4, 0) = -Q_;
  H.block<4, 4>(4, 4) = -A_.transpose();

  // 2. Compute Eigenvalues and Eigenvectors of H
  Eigen::EigenSolver<Eigen::Matrix<double, 8, 8>> solver(H);
  
  // Matrix to store eigenvectors corresponding to stable eigenvalues (real part < 0)
  Eigen::Matrix<double, 8, 4> stable_eigenvectors;
  int col_idx = 0;

  for (int i = 0; i < 8 && col_idx < 4; ++i) {
    if (solver.eigenvalues()[i].real() < -1e-6) {
      if (std::abs(solver.eigenvalues()[i].imag()) < 1e-6) {
        // Real eigenvalue: extract real eigenvector
        stable_eigenvectors.col(col_idx) = solver.eigenvectors().col(i).real();
        col_idx++;
      } else if (i < 7 && std::abs(solver.eigenvalues()[i].real() - solver.eigenvalues()[i+1].real()) < 1e-6) {
        // Complex conjugate pair: extract real part AND imaginary part into 2 columns
        stable_eigenvectors.col(col_idx) = solver.eigenvectors().col(i).real();
        col_idx++;
        if (col_idx < 4) {
          stable_eigenvectors.col(col_idx) = solver.eigenvectors().col(i).imag();
          col_idx++;
        }
        i++; // Skip conjugate pair in loop
      }
    }
  }

  if (col_idx < 4) {
    RCLCPP_ERROR(this->get_logger(), "CARE Solver failed: Could not find 4 stable eigenvectors!");
    return;
  }

  // 3. Extract U1 (top 4x4) and U2 (bottom 4x4) blocks
  Eigen::Matrix4d U1 = stable_eigenvectors.block<4, 4>(0, 0);
  Eigen::Matrix4d U2 = stable_eigenvectors.block<4, 4>(4, 0);

  // 4. Compute P = U2 * U1^-1
  P_ = U2 * U1.inverse();

  K_ = (1.0 / R_) * B_.transpose() * P_;
  RCLCPP_INFO(this->get_logger(), "K: [%f, %f, %f, %f]", K_(0), K_(1), K_(2), K_(3));
}

void LQRCartpoleBalancer::controlLoop() {
  if (!joint_states_received_) return;

  double x = state_(0);
  double theta = state_(1);
  double dx = state_(2);
  double dtheta = state_(3);

  double u = 0.0;

  if (std::abs(theta) < 0.3) {
    u = -K_.dot(state_);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10, 
      "u: %f N, theta: %f rad, x: %f m", u, theta, x
    );
  
  } else {
    // Energy Swing-Up Mode
    double E_current = (0.5 * Ip_ * dtheta * dtheta) + (m_ * g_ * l_ * std::cos(theta));
    double E_target = m_ * g_ * l_;
    double E_error = E_current - E_target;

    // Base swing-up cart acceleration
    double sgn = (dtheta * std::cos(theta) >= 0.0) ? 1.0 : -1.0;
    double ddx = ke_ * E_error * sgn;
    double ddx_cart = -k_x_ * x - k_dx_ * dx;

    // 4. Combined Command
    ddx += ddx_cart;

    // Force effort
    u = (M_ + m_) * ddx;
  }

  publishEffort(u);
}

void LQRCartpoleBalancer::publishEffort(double value) {
  auto msg = std_msgs::msg::Float64MultiArray();
  msg.data = {value};
  pub_effort_->publish(msg);
}

}  // namespace cartpole_control