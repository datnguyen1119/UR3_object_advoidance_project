#include <rclcpp/rclcpp.hpp>

#include <moveit_servo/servo_parameters.h>
#include <moveit_servo/servo.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene_world.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

class CustomMoveitServo : public rclcpp::Node
{
public:
  CustomMoveitServo()
  : Node("custom_moveit_servo",
         rclcpp::NodeOptions().use_intra_process_comms(false))
  {
    scene_pub_ = create_publisher<moveit_msgs::msg::PlanningScene>(
        "planning_scene", 10);
  }

  // Two-phase init: PSM and Servo need shared_from_this(), which is not
  // available inside a constructor. Call this after std::make_shared<>().
  void init()
  {
    setupPlanningSceneMonitor();
    // addCollisionObject();  // disabled: no hardcoded box in the scene — obstacles
                              // come from ObstacleManager / external sources instead.
    setupServo();
  }

  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr scene_pub_;

private:
  void setupPlanningSceneMonitor()
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
        shared_from_this(), "robot_description", tf_buffer_,
        "planning_scene_monitor");

    if (!psm_->getPlanningScene())
    {
      RCLCPP_ERROR(get_logger(), "Planning scene not configured");
      throw std::runtime_error("Planning scene not configured");
    }

    psm_->startStateMonitor("/joint_states");
    psm_->setPlanningScenePublishingFrequency(25);
    psm_->startPublishingPlanningScene(
        planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
        "/moveit_servo/publish_planning_scene");
    psm_->startSceneMonitor();
    psm_->providePlanningSceneService();
  }

  moveit_msgs::msg::CollisionObject makeBoxCollisionObject(
      const std::string& id,
      const std::string& frame_id,
      const std::array<double, 3>& dimensions,
      const geometry_msgs::msg::Pose& pose) const
  {
    moveit_msgs::msg::CollisionObject obj;
    obj.header.frame_id = frame_id;
    obj.id = id;

    shape_msgs::msg::SolidPrimitive box;
    box.type = box.BOX;
    box.dimensions = { dimensions[0], dimensions[1], dimensions[2] };

    obj.primitives.push_back(box);
    obj.primitive_poses.push_back(pose);
    obj.operation = obj.ADD;
    return obj;
  }

  void addCollisionObject()
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.6;
    pose.position.y = 0.0;
    pose.position.z = 0.6;

    auto box = makeBoxCollisionObject("box", "base_link", { 0.1, 0.6, 0.1 }, pose);

    moveit_msgs::msg::PlanningSceneWorld world;
    world.collision_objects.push_back(box);

    moveit_msgs::msg::PlanningScene scene;
    scene.is_diff = true;
    scene.world = world;

    scene_pub_->publish(scene);
  }

  void setupServo()
  {
    auto servo_parameters =
        moveit_servo::ServoParameters::makeServoParameters(shared_from_this());
    if (!servo_parameters)
    {
      RCLCPP_FATAL(get_logger(), "Failed to load the servo parameters");
      throw std::runtime_error("Failed to load servo parameters");
    }

    servo_ = std::make_unique<moveit_servo::Servo>(
        shared_from_this(), servo_parameters, psm_);
    servo_->start();
  }

  // Declaration order matters: tf_buffer_ -> psm_ -> servo_ so that Servo
  // is destroyed before the PSM it depends on.
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  planning_scene_monitor::PlanningSceneMonitorPtr psm_;
  std::unique_ptr<moveit_servo::Servo> servo_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CustomMoveitServo>();

  // Give RViz / the rest of the stack a moment to come up before Servo starts
  rclcpp::sleep_for(std::chrono::seconds(4));
  node->init();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
