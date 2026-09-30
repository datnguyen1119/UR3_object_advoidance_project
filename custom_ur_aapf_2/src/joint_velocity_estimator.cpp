#include "custom_ur_aapf_2/joint_velocity_estimator.hpp"

#include <rclcpp_components/register_node_macro.hpp>

namespace custom_ur_aapf_2
{

JointVelocityEstimator::JointVelocityEstimator(const rclcpp::NodeOptions& options)
  : rclcpp::Node("joint_velocity_estimator", options)
{
  joint_names_ = declare_parameter<std::vector<std::string>>(
      "joint_names",
      {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
       "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"});
  window_sec_   = declare_parameter<double>("velocity_window_sec", window_sec_);
  input_topic_  = declare_parameter<std::string>("input_topic", input_topic_);
  output_topic_ = declare_parameter<std::string>("output_topic", output_topic_);

  pub_ = create_publisher<sensor_msgs::msg::JointState>(output_topic_, rclcpp::QoS(10));
  sub_ = create_subscription<sensor_msgs::msg::JointState>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&JointVelocityEstimator::jointStateCb, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(),
              "JointVelocityEstimator: '%s' → '%s', %zu joints, window %.3f s",
              input_topic_.c_str(), output_topic_.c_str(), joint_names_.size(), window_sec_);
}

void JointVelocityEstimator::jointStateCb(sensor_msgs::msg::JointState::SharedPtr msg)
{
  // Extract this sample's positions for the configured joints, by name.
  std::vector<double> qpos(joint_names_.size(), 0.0);
  bool all_found = true;
  for (std::size_t i = 0; i < joint_names_.size(); ++i)
  {
    bool found = false;
    for (std::size_t j = 0; j < msg->name.size(); ++j)
    {
      if (msg->name[j] == joint_names_[i])
      {
        if (j < msg->position.size()) { qpos[i] = msg->position[j]; found = true; }
        break;
      }
    }
    all_found = all_found && found;
  }
  if (!all_found)
    return;  // partial message (e.g. gripper-only) — skip

  const double t = now().seconds();
  hist_.emplace_back(t, std::move(qpos));

  // Keep only samples within the window, but always at least two for a difference.
  while (hist_.size() > 2 && (t - hist_.front().first) > window_sec_)
    hist_.pop_front();

  if (hist_.size() < 2)
    return;

  const auto&  oldest = hist_.front();
  const auto&  newest = hist_.back();
  const double dt = newest.first - oldest.first;
  if (dt <= 1e-6)
    return;

  // q̇ ≈ (q_newest − q_oldest_in_window) / Δt — averaged direction over the window.
  sensor_msgs::msg::JointState out;
  out.header.stamp = now();
  out.name         = joint_names_;
  out.position     = newest.second;
  out.velocity.resize(joint_names_.size());
  for (std::size_t i = 0; i < joint_names_.size(); ++i)
    out.velocity[i] = (newest.second[i] - oldest.second[i]) / dt;

  pub_->publish(out);
}

}  // namespace custom_ur_aapf_2

RCLCPP_COMPONENTS_REGISTER_NODE(custom_ur_aapf_2::JointVelocityEstimator)
