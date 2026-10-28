#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

class PIDController : public rclcpp::Node
{
public:
  PIDController()
  : Node("pid_controller"),
    start_time_(std::chrono::steady_clock::now())
  {
    effort_publisher_ =
      this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/effort_controller/commands", 10);

    desired_state_publisher_ =
      this->create_publisher<sensor_msgs::msg::JointState>(
        "/desired_joint_states", 10);

    joint_state_subscriber_ =
      this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states",
        10,
        std::bind(
          &PIDController::joint_state_callback,
          this,
          std::placeholders::_1));

    timer_ =
      this->create_wall_timer(
        10ms,
        std::bind(
          &PIDController::control_loop,
          this));

    RCLCPP_INFO(
      this->get_logger(),
      "Custom PID effort controller started");
  }

private:

  void joint_state_callback(
    const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    for (std::size_t i = 0; i < msg->name.size(); ++i)
    {
      if (i < msg->position.size())
      {
        actual_position_[msg->name[i]] = msg->position[i];
      }

      if (i < msg->velocity.size())
      {
        actual_velocity_[msg->name[i]] = msg->velocity[i];
      }
    }

    received_joint_states_ = true;
  }


  void control_loop()
  {
    if (!received_joint_states_)
      return;

    if (effort_publisher_->get_subscription_count() == 0)
    {
      counter_++;

      if (counter_ >= 100)
      {
        counter_ = 0;

        RCLCPP_INFO(
          this->get_logger(),
          "Waiting for effort_controller...");
      }

      return;
    }


    const std::array<std::string, 4> joints = {
      "joint1",
      "joint2",
      "joint3",
      "joint4"
    };


    for (const auto & joint : joints)
    {
      if (actual_position_.find(joint) == actual_position_.end())
        return;

      if (actual_velocity_.find(joint) == actual_velocity_.end())
        return;
    }


    // --------------------------------------------
    // Time
    // --------------------------------------------

    auto now = std::chrono::steady_clock::now();

    double t =
      std::chrono::duration<double>(
        now - start_time_).count();

    constexpr double dt = 0.01;


    // --------------------------------------------
    // Desired sinusoidal trajectory
    // --------------------------------------------

    const double frequency = 0.08;
    const double omega = 2.0 * M_PI * frequency;


    std::array<double, 4> amplitude = {
      0.35,
      0.25,
     -0.20,
      0.15
    };


    std::array<double, 4> q_des;
    std::array<double, 4> qd_des;


    for (std::size_t i = 0; i < 4; ++i)
    {
      q_des[i] =
        amplitude[i] * std::sin(omega * t);

      qd_des[i] =
        amplitude[i] * omega * std::cos(omega * t);
    }


    // --------------------------------------------
    // Initial PID gains
    // --------------------------------------------

    const std::array<double, 4> kp = {
      6.0,
      20.0,
      14.0,
      7.0
    };

    const std::array<double, 4> ki = {
      0.10,
      0.0,
      0.30,
      0.10
    };

    const std::array<double, 4> kd = {
      1.2,
      5.0,
      1.8,
      1.0
    };


    // Torque limits from URDF
    const std::array<double, 4> torque_limit = {
      8.0,
      10.0,
      10.0,
      6.0
    };


    std::array<double, 4> torque;
    std::array<double, 4> error;


    // --------------------------------------------
    // PID calculation
    // --------------------------------------------

    for (std::size_t i = 0; i < 4; ++i)
    {
      double q =
        actual_position_[joints[i]];

      double q_dot =
        actual_velocity_[joints[i]];


      error[i] =
        q_des[i] - q;


      integral_error_[i] +=
        error[i] * dt;


      // Anti-windup
      integral_error_[i] =
        std::clamp(
          integral_error_[i],
          -2.0,
          2.0);


      // derivative of error
      double derivative_error =
        qd_des[i] - q_dot;


      torque[i] =
        kp[i] * error[i]
        + ki[i] * integral_error_[i]
        + kd[i] * derivative_error;


      // Torque saturation
      torque[i] =
        std::clamp(
          torque[i],
          -torque_limit[i],
          torque_limit[i]);
    }


    // --------------------------------------------
    // Publish torque commands
    // --------------------------------------------

    std_msgs::msg::Float64MultiArray effort_msg;

    effort_msg.data.assign(
      torque.begin(),
      torque.end());

    effort_publisher_->publish(effort_msg);


    // --------------------------------------------
    // Publish desired state for logger
    // --------------------------------------------

    sensor_msgs::msg::JointState desired_msg;

    desired_msg.header.stamp = this->now();

    desired_msg.name.assign(
      joints.begin(),
      joints.end());

    desired_msg.position.assign(
      q_des.begin(),
      q_des.end());

    desired_msg.velocity.assign(
      qd_des.begin(),
      qd_des.end());

    desired_state_publisher_->publish(desired_msg);


    // --------------------------------------------
    // Console output once per second
    // --------------------------------------------

    counter_++;

    if (counter_ >= 100)
    {
      counter_ = 0;

      RCLCPP_INFO(
        this->get_logger(),
        "t=%.2f | "
        "q1_des=%.3f q1=%.3f tau1=%.3f | "
        "q2_des=%.3f q2=%.3f tau2=%.3f",
        t,
        q_des[0],
        actual_position_["joint1"],
        torque[0],
        q_des[1],
        actual_position_["joint2"],
        torque[1]);
    }
  }


  rclcpp::Publisher<
    std_msgs::msg::Float64MultiArray>::SharedPtr
    effort_publisher_;

  rclcpp::Publisher<
    sensor_msgs::msg::JointState>::SharedPtr
    desired_state_publisher_;

  rclcpp::Subscription<
    sensor_msgs::msg::JointState>::SharedPtr
    joint_state_subscriber_;

  rclcpp::TimerBase::SharedPtr timer_;


  std::unordered_map<std::string, double>
    actual_position_;

  std::unordered_map<std::string, double>
    actual_velocity_;


  std::array<double, 4> integral_error_ = {
    0.0, 0.0, 0.0, 0.0
  };


  bool received_joint_states_ = false;

  int counter_ = 0;

  std::chrono::steady_clock::time_point start_time_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<PIDController>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
