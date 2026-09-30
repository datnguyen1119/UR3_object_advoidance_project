#pragma once

#include <memory>
#include <string>

#include <moveit_msgs/msg/planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace custom_ur_aapf_2
{

// ObstacleManager — dev-time helper to spawn/remove a single sphere obstacle
// in the global planning scene by publishing a diff PlanningScene to
// /planning_scene (avoids the service-call-inside-callback deadlock).
//
// Services
//   ~/spawn   (std_srvs/Trigger)   — spawn the configured sphere
//   ~/remove  (std_srvs/Trigger)   — remove the configured sphere
//   ~/toggle  (std_srvs/SetBool)   — data=true → spawn, data=false → remove
class ObstacleManager : public rclcpp::Node
{
public:
  explicit ObstacleManager(const rclcpp::NodeOptions& options);

private:
  struct Params
  {
    std::string id        = "test_sphere";
    std::string frame_id  = "world";
    double x              = 0.5;
    double y              = 0.0;
    double z              = 0.7;
    double radius         = 0.15;
  };

  void declareParameters();
  void loadParameters();

  std::pair<bool, std::string> applySphere(bool add);

  void spawnCb(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response>);
  void removeCb(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                std::shared_ptr<std_srvs::srv::Trigger::Response>);
  void toggleCb(const std::shared_ptr<std_srvs::srv::SetBool::Request>,
                std::shared_ptr<std_srvs::srv::SetBool::Response>);

  Params params_;

  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr scene_pub_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr spawn_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr remove_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr toggle_srv_;
};

}  // namespace custom_ur_aapf_2
