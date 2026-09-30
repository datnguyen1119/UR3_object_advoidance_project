#pragma once

#include <memory>
#include <string>

#include <geometry_msgs/msg/pose.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>

namespace custom_ur_aapf_2
{

// InteractiveObstacle — a *peer* obstacle source on the same convergence point
// as ObstacleManager: it publishes a sphere CollisionObject diff to
// /planning_scene, which the supervisor reads via enumerateSpheres().
//
// Unlike ObstacleManager (fixed pose, spawn-on-service), this one exposes an
// RViz InteractiveMarker you can drag at runtime; every drag re-publishes the
// obstacle at the new pose, so the supervisor sees a MOVING obstacle live.
//
// It uses its OWN collision-object id ("interactive_obstacle" by default), so it
// coexists with test_sphere and with any future camera-driven source — each
// source owns a distinct id and they never clash. This keeps the
// "obstacle source" axis open: add a source = add a peer node, edit nothing.
class InteractiveObstacle : public rclcpp::Node
{
public:
  explicit InteractiveObstacle(const rclcpp::NodeOptions& options);

private:
  struct Params
  {
    std::string id        = "interactive_obstacle";
    std::string frame_id  = "world";
    double x              = 0.5;
    double y              = 0.3;
    double z              = 0.6;
    double radius         = 0.1;
  };

  void declareParameters();
  void loadParameters();

  // Build the draggable marker (sphere visual + 3D move + per-axis controls).
  void buildMarker();

  // RViz drag callback → publish the obstacle at the new pose.
  void processFeedback(
      visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback);

  // Publish a REMOVE+ADD PlanningScene diff placing the sphere at `pose`.
  void publishObstacle(const geometry_msgs::msg::Pose& pose);

  Params params_;
  geometry_msgs::msg::Pose current_pose_;

  std::unique_ptr<interactive_markers::InteractiveMarkerServer> server_;
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr scene_pub_;
  rclcpp::TimerBase::SharedPtr init_timer_;  // one-shot: publish initial pose
};

}  // namespace custom_ur_aapf_2
