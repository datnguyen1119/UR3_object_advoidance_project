#include "custom_ur_aapf_2/interactive_obstacle.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace custom_ur_aapf_2
{

using namespace std::chrono_literals;

InteractiveObstacle::InteractiveObstacle(const rclcpp::NodeOptions& options)
  : rclcpp::Node("interactive_obstacle", options)
{
  declareParameters();
  loadParameters();

  current_pose_.position.x = params_.x;
  current_pose_.position.y = params_.y;
  current_pose_.position.z = params_.z;
  current_pose_.orientation.w = 1.0;

  // Same convergence point + QoS as ObstacleManager — both are peer sources.
  scene_pub_ = create_publisher<moveit_msgs::msg::PlanningScene>(
      "/planning_scene", rclcpp::QoS(10).reliable());

  server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>(
      "interactive_obstacle",
      get_node_base_interface(), get_node_clock_interface(),
      get_node_logging_interface(), get_node_topics_interface(),
      get_node_services_interface());

  buildMarker();

  // Publish the initial pose once, slightly deferred so move_group's scene
  // monitor has time to subscribe (a diff sent before anyone listens is lost).
  init_timer_ = create_wall_timer(1s, [this]() {
    publishObstacle(current_pose_);
    init_timer_->cancel();
  });

  RCLCPP_INFO(get_logger(),
              "InteractiveObstacle up. id='%s' frame='%s' start=(%.2f, %.2f, %.2f) r=%.2f. "
              "Drag the marker in RViz (topic: /interactive_obstacle/update).",
              params_.id.c_str(), params_.frame_id.c_str(),
              params_.x, params_.y, params_.z, params_.radius);
}

// ----------------------------------------------------------------------------
// Parameters
// ----------------------------------------------------------------------------

void InteractiveObstacle::declareParameters()
{
  declare_parameter<std::string>("obstacle.id",       params_.id);
  declare_parameter<std::string>("obstacle.frame_id", params_.frame_id);
  declare_parameter<double>("obstacle.x",             params_.x);
  declare_parameter<double>("obstacle.y",             params_.y);
  declare_parameter<double>("obstacle.z",             params_.z);
  declare_parameter<double>("obstacle.radius",        params_.radius);
}

void InteractiveObstacle::loadParameters()
{
  get_parameter("obstacle.id",       params_.id);
  get_parameter("obstacle.frame_id", params_.frame_id);
  get_parameter("obstacle.x",        params_.x);
  get_parameter("obstacle.y",        params_.y);
  get_parameter("obstacle.z",        params_.z);
  get_parameter("obstacle.radius",   params_.radius);
}

// ----------------------------------------------------------------------------
// Marker construction
// ----------------------------------------------------------------------------

void InteractiveObstacle::buildMarker()
{
  using visualization_msgs::msg::InteractiveMarker;
  using visualization_msgs::msg::InteractiveMarkerControl;
  using visualization_msgs::msg::Marker;

  InteractiveMarker int_marker;
  int_marker.header.frame_id = params_.frame_id;
  int_marker.name            = params_.id;
  int_marker.description     = "obstacle (drag me)";
  int_marker.pose            = current_pose_;
  int_marker.scale           = static_cast<float>(std::max(2.0 * params_.radius, 0.2));

  // Sphere visual — matches the collision sphere it publishes (diameter = 2r).
  Marker sphere;
  sphere.type = Marker::SPHERE;
  sphere.scale.x = sphere.scale.y = sphere.scale.z = 2.0 * params_.radius;
  sphere.color.r = 1.0f;
  sphere.color.g = 0.3f;
  sphere.color.b = 0.0f;
  sphere.color.a = 0.6f;

  // Grab the sphere and move freely in 3D.
  InteractiveMarkerControl sphere_ctrl;
  sphere_ctrl.always_visible = true;
  sphere_ctrl.interaction_mode = InteractiveMarkerControl::MOVE_3D;
  sphere_ctrl.markers.push_back(sphere);
  int_marker.controls.push_back(sphere_ctrl);

  // Per-axis translation handles for precise placement.
  auto add_axis = [&int_marker](const std::string& name,
                                double qx, double qy, double qz) {
    InteractiveMarkerControl c;
    const double n = std::sqrt(1.0 + qx * qx + qy * qy + qz * qz);
    c.orientation.w = 1.0 / n;
    c.orientation.x = qx / n;
    c.orientation.y = qy / n;
    c.orientation.z = qz / n;
    c.name = name;
    c.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(c);
  };
  add_axis("move_x", 1.0, 0.0, 0.0);
  add_axis("move_y", 0.0, 0.0, 1.0);
  add_axis("move_z", 0.0, 1.0, 0.0);

  server_->insert(int_marker,
      std::bind(&InteractiveObstacle::processFeedback, this, std::placeholders::_1));
  server_->applyChanges();
}

// ----------------------------------------------------------------------------
// Drag callback
// ----------------------------------------------------------------------------

void InteractiveObstacle::processFeedback(
    visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback)
{
  // Re-publish on live drag (POSE_UPDATE) and on release (MOUSE_UP) so the
  // supervisor tracks a moving obstacle in real time.
  using FB = visualization_msgs::msg::InteractiveMarkerFeedback;
  if (feedback->event_type != FB::POSE_UPDATE &&
      feedback->event_type != FB::MOUSE_UP)
    return;

  current_pose_ = feedback->pose;
  publishObstacle(current_pose_);
}

// ----------------------------------------------------------------------------
// Publish the obstacle as a PlanningScene diff (REMOVE+ADD, like ObstacleManager)
// ----------------------------------------------------------------------------

void InteractiveObstacle::publishObstacle(const geometry_msgs::msg::Pose& pose)
{
  loadParameters();  // pick up runtime radius/id changes via `ros2 param set`

  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;
  scene.robot_state.is_diff = true;

  // REMOVE first (no-op if absent) so the new pose/radius always wins.
  {
    moveit_msgs::msg::CollisionObject obj;
    obj.id = params_.id;
    obj.header.frame_id = params_.frame_id;
    obj.header.stamp = now();
    obj.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    scene.world.collision_objects.push_back(obj);
  }

  // ADD at the dragged pose.
  {
    moveit_msgs::msg::CollisionObject obj;
    obj.id = params_.id;
    obj.header.frame_id = params_.frame_id;
    obj.header.stamp = now();
    obj.operation = moveit_msgs::msg::CollisionObject::ADD;
    obj.pose.orientation.w = 1.0;  // identity; valid quaternion (see ObstacleManager note)

    shape_msgs::msg::SolidPrimitive sphere;
    sphere.type = shape_msgs::msg::SolidPrimitive::SPHERE;
    sphere.dimensions.resize(1);
    sphere.dimensions[shape_msgs::msg::SolidPrimitive::SPHERE_RADIUS] = params_.radius;
    obj.primitives.push_back(sphere);

    geometry_msgs::msg::Pose ppose;
    ppose.position = pose.position;
    ppose.orientation.w = 1.0;
    obj.primitive_poses.push_back(ppose);

    scene.world.collision_objects.push_back(obj);
  }

  scene_pub_->publish(scene);
}

}  // namespace custom_ur_aapf_2

RCLCPP_COMPONENTS_REGISTER_NODE(custom_ur_aapf_2::InteractiveObstacle)
