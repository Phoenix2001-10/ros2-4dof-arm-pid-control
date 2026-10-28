#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class TrajectoryPublisher : public rclcpp::Node
{
public:
  TrajectoryPublisher()
  : Node("trajectory_publisher"),
    start_time_(std::chrono::steady_clock::now())
  {
    trajectory_publisher_ =
      this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/arm_controller/joint_trajectory", 10);

    desired_state_publisher_ =
      this->create_publisher<sensor_msgs::msg::JointState>(
        "/desired_joint_states", 10);

    joint_state_subscriber_ =
      this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states",
        10,
        std::bind(
          &TrajectoryPublisher::joint_state_callback,
          this,
          std::placeholders::_1));

    timer_ =
      this->create_wall_timer(
        100ms,
        std::bind(
          &TrajectoryPublisher::control_loop,
          this));

    RCLCPP_INFO(
      this->get_logger(),
      "Sinusoidal trajectory controller started");
  }

private:

  void joint_state_callback(
    const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    for (std::size_t i = 0;
         i < msg->name.size() && i < msg->position.size();
         ++i)
    {
      actual_positions_[msg->name[i]] = msg->position[i];
    }

    received_joint_states_ = true;
  }


  void control_loop()
  {
    if (trajectory_publisher_->get_subscription_count() == 0)
    {
      RCLCPP_INFO(
        this->get_logger(),
        "Waiting for arm_controller...");
      return;
    }

    auto now = std::chrono::steady_clock::now();

    double t =
      std::chrono::duration<double>(
        now - start_time_).count();

    const double frequency = 0.15;
    const double omega = 2.0 * M_PI * frequency;

    double q1_des =  0.50 * std::sin(omega * t);
    double q2_des =  0.40 * std::sin(omega * t);
    double q3_des = -0.30 * std::sin(omega * t);
    double q4_des =  0.20 * std::sin(omega * t);

    std::vector<double> desired_positions = {
      q1_des,
      q2_des,
      q3_des,
      q4_des
    };

    trajectory_msgs::msg::JointTrajectory trajectory;

    trajectory.joint_names = {
      "joint1",
      "joint2",
      "joint3",
      "joint4"
    };

    trajectory_msgs::msg::JointTrajectoryPoint point;

    point.positions = desired_positions;

    point.time_from_start.sec = 0;
    point.time_from_start.nanosec = 200000000;

    trajectory.points.push_back(point);

    trajectory_publisher_->publish(trajectory);


    sensor_msgs::msg::JointState desired_msg;

    desired_msg.header.stamp = this->now();

    desired_msg.name = {
      "joint1",
      "joint2",
      "joint3",
      "joint4"
    };

    desired_msg.position = desired_positions;

    desired_state_publisher_->publish(desired_msg);


    if (received_joint_states_)
    {
      std::vector<std::string> joints = {
        "joint1",
        "joint2",
        "joint3",
        "joint4"
      };

      std::vector<double> errors(4, 0.0);

      bool all_joints_found = true;

      for (std::size_t i = 0; i < joints.size(); ++i)
      {
        if (actual_positions_.find(joints[i]) !=
            actual_positions_.end())
        {
          errors[i] =
            desired_positions[i]
            - actual_positions_[joints[i]];
        }
        else
        {
          all_joints_found = false;
        }
      }

      counter_++;

      if (all_joints_found && counter_ >= 10)
      {
        counter_ = 0;

        RCLCPP_INFO(
          this->get_logger(),
          "t=%.2f | "
          "q1_des=%.3f q1=%.3f e1=%.3f | "
          "q2_des=%.3f q2=%.3f e2=%.3f",
          t,
          q1_des,
          actual_positions_["joint1"],
          errors[0],
          q2_des,
          actual_positions_["joint2"],
          errors[1]);
      }
    }
  }


  rclcpp::Publisher<
    trajectory_msgs::msg::JointTrajectory>::SharedPtr
    trajectory_publisher_;

  rclcpp::Publisher<
    sensor_msgs::msg::JointState>::SharedPtr
    desired_state_publisher_;

  rclcpp::Subscription<
    sensor_msgs::msg::JointState>::SharedPtr
    joint_state_subscriber_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::chrono::steady_clock::time_point start_time_;

  std::unordered_map<std::string, double>
    actual_positions_;

  bool received_joint_states_ = false;

  int counter_ = 0;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<TrajectoryPublisher>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
