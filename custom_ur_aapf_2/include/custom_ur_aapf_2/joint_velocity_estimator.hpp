#pragma once

#include <deque>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace custom_ur_aapf_2
{

// JointVelocityEstimator — standalone node that estimates joint velocities q̇
// from a stream of joint POSITIONS and republishes them on its own topic.
//
// Why a separate node: the velocity is derived by finite-differencing recent
// positions over a window (more representative than a single instantaneous
// sample, and independent of whether the driver fills /joint_states.velocity).
// Keeping it standalone lets any consumer (here: the AAPF supervisor's
// singularity-coast hold) subscribe to a clean q̇ instead of computing it.
//
//   input_topic  (sensor_msgs/JointState)  — positions in (e.g. /joint_states)
//   output_topic (sensor_msgs/JointState)  — name[] + velocity[] = estimated q̇
//
// q̇ ≈ (q_newest − q_oldest_in_window) / Δt, computed per configured joint.
class JointVelocityEstimator : public rclcpp::Node
{
public:
  explicit JointVelocityEstimator(const rclcpp::NodeOptions& options);

private:
  void jointStateCb(sensor_msgs::msg::JointState::SharedPtr msg);

  std::vector<std::string> joint_names_;
  double                   window_sec_   = 0.05;
  std::string              input_topic_  = "/joint_states";
  std::string              output_topic_ = "/arm_joint_velocities";

  // Recent (time[s], joint positions[rad]) samples, joint_names_ order.
  std::deque<std::pair<double, std::vector<double>>> hist_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr    pub_;
};

}  // namespace custom_ur_aapf_2
