#include "custom_ur_aapf_2/obstacle_manager.hpp"

#include <chrono>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

namespace custom_ur_aapf_2
{

using namespace std::chrono_literals;

ObstacleManager::ObstacleManager(const rclcpp::NodeOptions& options)
  : rclcpp::Node("obstacle_manager", options)
{
  declareParameters();
  loadParameters();

  // Publish to /planning_scene with is_diff=true. PSM SceneMonitors of
  // move_group and the supervisor merge the diff into their world.
  // QoS: RELIABLE + KEEP_LAST(10), matching what move_group publishes/expects.
  scene_pub_ = create_publisher<moveit_msgs::msg::PlanningScene>(
      "/planning_scene", rclcpp::QoS(10).reliable());

  using std::placeholders::_1;
  using std::placeholders::_2;
  spawn_srv_  = create_service<std_srvs::srv::Trigger>(
      "~/spawn",  std::bind(&ObstacleManager::spawnCb,  this, _1, _2));
  remove_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/remove", std::bind(&ObstacleManager::removeCb, this, _1, _2));
  toggle_srv_ = create_service<std_srvs::srv::SetBool>(
      "~/toggle", std::bind(&ObstacleManager::toggleCb, this, _1, _2));

  RCLCPP_INFO(get_logger(),
              "ObstacleManager up. id='%s' frame='%s' pose=(%.2f, %.2f, %.2f) r=%.2f",
              params_.id.c_str(), params_.frame_id.c_str(),
              params_.x, params_.y, params_.z, params_.radius);
}

// ----------------------------------------------------------------------------
// Parameters
// ----------------------------------------------------------------------------

void ObstacleManager::declareParameters()
{
  declare_parameter<std::string>("obstacle.id",        params_.id);
  declare_parameter<std::string>("obstacle.frame_id",  params_.frame_id);
  declare_parameter<double>("obstacle.x",              params_.x);
  declare_parameter<double>("obstacle.y",              params_.y);
  declare_parameter<double>("obstacle.z",              params_.z);
  declare_parameter<double>("obstacle.radius",         params_.radius);
}

void ObstacleManager::loadParameters()
{
  get_parameter("obstacle.id",        params_.id);
  get_parameter("obstacle.frame_id",  params_.frame_id);
  get_parameter("obstacle.x",         params_.x);
  get_parameter("obstacle.y",         params_.y);
  get_parameter("obstacle.z",         params_.z);
  get_parameter("obstacle.radius",    params_.radius);
}

// ----------------------------------------------------------------------------
// Core: build CollisionObject + wrap in diff PlanningScene + publish.
// Topic publish avoids the deadlock pattern of calling /apply_planning_scene
// service from inside a service callback under a single-threaded executor.
// ----------------------------------------------------------------------------

std::pair<bool, std::string> ObstacleManager::applySphere(bool add)
{
  // Refresh parameters every call so the user can tweak xyz/radius at runtime
  // with `ros2 param set` and the next spawn picks them up.
  loadParameters();

  // Build a PlanningScene diff. CRITICAL: when spawning, send REMOVE first
  // then ADD in the same message. MoveIt's World treats CollisionObject::ADD
  // with an existing id as APPEND-shape (or no-op for the pose) depending on
  // version — using REMOVE then ADD guarantees the new pose/radius wins.
  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;
  scene.robot_state.is_diff = true;

  // 1. REMOVE (no-op if id doesn't exist — safe to always send).
  {
    moveit_msgs::msg::CollisionObject obj;
    obj.id = params_.id;
    obj.header.frame_id = params_.frame_id;
    obj.header.stamp = now();
    obj.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    scene.world.collision_objects.push_back(obj);
  }

  // 2. ADD with the latest pose/radius (only when spawning, not for plain remove).
  if (add)
  {
    moveit_msgs::msg::CollisionObject obj;
    obj.id = params_.id;
    obj.header.frame_id = params_.frame_id;
    obj.header.stamp = now();
    obj.operation = moveit_msgs::msg::CollisionObject::ADD;

    // CRITICAL: set obj.pose to identity. Default-constructed geometry_msgs::Pose
    // has orientation (0,0,0,0) which is an invalid (zero-norm) quaternion.
    // MoveIt's World combines shape_pose = obj.pose * primitive_poses[i]; if
    // obj.pose has zero rotation, the result collapses to origin → sphere
    // appears at @(0,0,0) regardless of primitive_poses values.
    obj.pose.orientation.w = 1.0;

    shape_msgs::msg::SolidPrimitive sphere;
    sphere.type = shape_msgs::msg::SolidPrimitive::SPHERE;
    sphere.dimensions.resize(1);
    sphere.dimensions[shape_msgs::msg::SolidPrimitive::SPHERE_RADIUS] = params_.radius;
    obj.primitives.push_back(sphere);

    geometry_msgs::msg::Pose pose;
    pose.position.x = params_.x;
    pose.position.y = params_.y;
    pose.position.z = params_.z;
    pose.orientation.w = 1.0;
    obj.primitive_poses.push_back(pose);

    scene.world.collision_objects.push_back(obj);
  }

  scene_pub_->publish(scene);

  // Diagnostic: dump exactly what we put on the wire (publisher-side truth).
  if (add)
  {
    RCLCPP_INFO(get_logger(),
                "Published diff: id='%s' frame='%s' obj.pose=(orient w=%.2f) "
                "primitive_pose=(%.3f, %.3f, %.3f) r=%.3f",
                params_.id.c_str(), params_.frame_id.c_str(),
                scene.world.collision_objects.back().pose.orientation.w,
                params_.x, params_.y, params_.z, params_.radius);
  }

  return {true, add ? "spawned (REMOVE+ADD diff published)"
                    : "removed (REMOVE diff published)"};
}

// ----------------------------------------------------------------------------
// Service callbacks
// ----------------------------------------------------------------------------

void ObstacleManager::spawnCb(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  std::tie(res->success, res->message) = applySphere(true);
  RCLCPP_INFO(get_logger(), "spawn: %s — %s",
              res->success ? "OK" : "FAIL", res->message.c_str());
}

void ObstacleManager::removeCb(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  std::tie(res->success, res->message) = applySphere(false);
  RCLCPP_INFO(get_logger(), "remove: %s — %s",
              res->success ? "OK" : "FAIL", res->message.c_str());
}

void ObstacleManager::toggleCb(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    std::shared_ptr<std_srvs::srv::SetBool::Response> res)
{
  std::tie(res->success, res->message) = applySphere(req->data);
  RCLCPP_INFO(get_logger(), "toggle(%s): %s — %s",
              req->data ? "true" : "false",
              res->success ? "OK" : "FAIL", res->message.c_str());
}

}  // namespace custom_ur_aapf_2

RCLCPP_COMPONENTS_REGISTER_NODE(custom_ur_aapf_2::ObstacleManager)
