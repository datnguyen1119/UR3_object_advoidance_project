#include "custom_ur_aapf_2/aapf_supervisor_node.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <tuple>
#include <utility>

#include <Eigen/SVD>
#include <geometric_shapes/shapes.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>

namespace custom_ur_aapf_2
{

using namespace std::chrono_literals;

// ============================================================================
// Construction
// ============================================================================

SupervisorNode::SupervisorNode(const rclcpp::NodeOptions& options)
  : rclcpp::Node("aapf_supervisor", options)
{
  declareParameters();
  loadParameters();

  supervisor_   = std::make_unique<Supervisor>(buildCoreParams());
  aapf_         = std::make_unique<AAPFGenerator>(node_params_.aapf);
  singularity_  = std::make_unique<SingularityHandler>(node_params_.singularity);
  latest_arm_qdot_.assign(supervisor_->params().arm_joints.size(), 0.0);

  mode_pub_       = create_publisher<std_msgs::msg::Int8>("~/mode", 10);
  mode_text_pub_  = create_publisher<std_msgs::msg::String>("~/mode_text", 10);
  error_pub_      = create_publisher<std_msgs::msg::Int8>("~/error", 10);
  error_text_pub_ = create_publisher<std_msgs::msg::String>("~/error_text", 10);
  distance_pub_   = create_publisher<std_msgs::msg::Float64>("~/obstacle_distance", 10);
  condition_pub_  = create_publisher<std_msgs::msg::Float64>("~/condition_number", 10);
  near_sing_pub_  = create_publisher<std_msgs::msg::Bool>("~/near_singularity", 10);
  ee_speed_pub_   = create_publisher<std_msgs::msg::Float64>("~/ee_speed", 10);
  log_debug_pub_  = create_publisher<std_msgs::msg::Float64MultiArray>("~/log_debug", 10);
  vel_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/velocity_markers", 10);
  velocity_arrow_scale_ = declare_parameter<double>("velocity_arrow_scale", velocity_arrow_scale_);
  marker_pub_     = create_publisher<visualization_msgs::msg::MarkerArray>("~/lookahead_markers", 10);
  twist_pub_      = create_publisher<geometry_msgs::msg::TwistStamped>(
                        node_params_.servo_twist_topic, rclcpp::QoS(10).reliable());
  joint_pub_      = create_publisher<control_msgs::msg::JointJog>(
                        node_params_.servo_joint_topic, rclcpp::QoS(10).reliable());

  traj_sub_ = create_subscription<moveit_msgs::msg::DisplayTrajectory>(
      node_params_.trajectory_topic, rclcpp::QoS(10),
      std::bind(&SupervisorNode::trajectoryCb, this, std::placeholders::_1));

  // Joint velocities for the singularity-coast hold, from JointVelocityEstimator.
  joint_vel_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      node_params_.joint_velocity_topic, rclcpp::QoS(10),
      std::bind(&SupervisorNode::jointVelocityCb, this, std::placeholders::_1));
  RCLCPP_INFO(get_logger(), "Subscribed to trajectory topic: '%s'",
              traj_sub_->get_topic_name());

  using std::placeholders::_1;
  using std::placeholders::_2;
  start_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/start", std::bind(&SupervisorNode::startCb, this, _1, _2));
  stop_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/stop",  std::bind(&SupervisorNode::stopCb,  this, _1, _2));
  reset_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/reset", std::bind(&SupervisorNode::resetCb, this, _1, _2));

  init_timer_ = create_wall_timer(0ms, [this] {
    init_timer_->cancel();
    deferredInit();
  });

  RCLCPP_INFO(get_logger(),
              "SupervisorNode(v2) constructed. check=%.1f Hz aapf=%.1f Hz, gripper_z=%.3f m",
              node_params_.obstacle_check_rate, node_params_.aapf_control_rate,
              node_params_.gripper_z_offset);
}

SupervisorNode::~SupervisorNode() = default;

// ============================================================================
// Parameters
// ============================================================================

void SupervisorNode::declareParameters()
{
  Supervisor::Params cd;  // core defaults
  // Obstacle distances.
  declare_parameter<double>("safe_distance",              cd.safe_distance);
  declare_parameter<double>("blocked_distance_threshold", cd.blocked_distance_threshold);
  declare_parameter<double>("emergency_distance",         cd.emergency_distance);
  declare_parameter<double>("return_clearance",           cd.return_clearance);
  // Waypoint / lookahead.
  declare_parameter<int>("lookahead_waypoint_count", cd.lookahead_waypoint_count);
  declare_parameter<int>("p1_search_horizon",        cd.p1_search_horizon);
  declare_parameter<int>("p1_index_offset",          cd.p1_index_offset);
  declare_parameter<int>("p2_index_offset",          cd.p2_index_offset);
  declare_parameter<double>("max_p1_p2_distance",    cd.max_p1_p2_distance);
  declare_parameter<double>("min_p1_p2_distance",    cd.min_p1_p2_distance);
  // Return to trajectory.
  declare_parameter<double>("return_to_path_distance", cd.return_to_path_distance);
  declare_parameter<int>("rejoin_waypoint_offset",     cd.rejoin_waypoint_offset);
  // Always-on debug lookahead points.
  declare_parameter<int>("vis_p2_offset", cd.vis_p2_offset);
  declare_parameter<int>("vis_p1_offset", cd.vis_p1_offset);
  // Subdivision + search.
  declare_parameter<double>("subdivide_step_m",   cd.subdivide_step_m);
  declare_parameter<int>("search_window_steps",   cd.search_window_steps);
  // Timing.
  declare_parameter<double>("avoid_timeout",  cd.avoid_timeout);
  declare_parameter<double>("rejoin_timeout", cd.rejoin_timeout);
  // Safety.
  declare_parameter<double>("max_path_deviation", cd.max_path_deviation);
  declare_parameter<std::vector<std::string>>("arm_joints", cd.arm_joints);

  // Node-level.
  declare_parameter<double>("obstacle_check_rate", node_params_.obstacle_check_rate);
  declare_parameter<double>("aapf_control_rate",   node_params_.aapf_control_rate);
  declare_parameter<double>("scene_stale_sec",     node_params_.scene_stale_sec);
  declare_parameter<double>("gripper_z_offset",    node_params_.gripper_z_offset);
  declare_parameter<double>("ee_speed_window_sec", node_params_.ee_speed_window_sec);
  declare_parameter<std::string>("trajectory_topic",  node_params_.trajectory_topic);
  declare_parameter<std::string>("servo_twist_topic", node_params_.servo_twist_topic);
  declare_parameter<std::string>("controller_follow_action", node_params_.controller_follow_action);
  declare_parameter<bool>("resume_trajectory_on_rejoin", node_params_.resume_trajectory_on_rejoin);
  declare_parameter<bool>("reik_resume", node_params_.reik_resume);
  declare_parameter<double>("resume_delay_sec", node_params_.resume_delay_sec);
  declare_parameter<double>("resume_velocity_scaling",     node_params_.resume_velocity_scaling);
  declare_parameter<double>("resume_acceleration_scaling", node_params_.resume_acceleration_scaling);
  declare_parameter<bool>("enable_controller_switch", node_params_.enable_controller_switch);
  declare_parameter<std::string>("motion_controller", node_params_.motion_controller);
  declare_parameter<std::string>("servo_controller",  node_params_.servo_controller);
  declare_parameter<std::string>("move_group_name", node_params_.move_group_name);
  declare_parameter<std::string>("planning_frame",  node_params_.planning_frame);
  declare_parameter<std::string>("ee_link",         node_params_.ee_link);

  // AAPF tuning.
  declare_parameter<double>("aapf.k_att",                node_params_.aapf.k_att);
  declare_parameter<double>("aapf.k_rep",                node_params_.aapf.k_rep);
  declare_parameter<double>("aapf.influence_distance",   node_params_.aapf.influence_distance);
  declare_parameter<double>("aapf.v_max",                node_params_.aapf.v_max);
  declare_parameter<double>("aapf.v_min",                node_params_.aapf.v_min);
  declare_parameter<double>("aapf.repulsive_saturation", node_params_.aapf.repulsive_saturation);
  declare_parameter<double>("aapf.v_rep_max",            node_params_.aapf.v_rep_max);
  declare_parameter<std::string>("aapf.command_frame",   node_params_.aapf.command_frame);

  // Singularity coast (trục 2).
  declare_parameter<std::string>("servo_joint_topic",        node_params_.servo_joint_topic);
  declare_parameter<std::string>("joint_velocity_topic",     node_params_.joint_velocity_topic);
  declare_parameter<bool>("enable_singularity_coast",        node_params_.enable_singularity_coast);
  declare_parameter<double>("singularity.kappa_enter",       node_params_.singularity.kappa_enter);
  declare_parameter<double>("singularity.kappa_exit",        node_params_.singularity.kappa_exit);
}

void SupervisorNode::loadParameters()
{
  get_parameter("obstacle_check_rate", node_params_.obstacle_check_rate);
  get_parameter("aapf_control_rate",   node_params_.aapf_control_rate);
  get_parameter("scene_stale_sec",     node_params_.scene_stale_sec);
  get_parameter("gripper_z_offset",    node_params_.gripper_z_offset);
  get_parameter("ee_speed_window_sec", node_params_.ee_speed_window_sec);
  get_parameter("trajectory_topic",    node_params_.trajectory_topic);
  get_parameter("servo_twist_topic",   node_params_.servo_twist_topic);
  get_parameter("controller_follow_action",     node_params_.controller_follow_action);
  get_parameter("resume_trajectory_on_rejoin",  node_params_.resume_trajectory_on_rejoin);
  get_parameter("reik_resume",                   node_params_.reik_resume);
  get_parameter("resume_velocity_scaling",        node_params_.resume_velocity_scaling);
  get_parameter("resume_acceleration_scaling",    node_params_.resume_acceleration_scaling);
  get_parameter("resume_delay_sec",              node_params_.resume_delay_sec);
  get_parameter("enable_controller_switch",      node_params_.enable_controller_switch);
  get_parameter("motion_controller",             node_params_.motion_controller);
  get_parameter("servo_controller",              node_params_.servo_controller);
  get_parameter("move_group_name",     node_params_.move_group_name);
  get_parameter("planning_frame",      node_params_.planning_frame);
  get_parameter("ee_link",             node_params_.ee_link);

  get_parameter("aapf.k_att",                node_params_.aapf.k_att);
  get_parameter("aapf.k_rep",                node_params_.aapf.k_rep);
  get_parameter("aapf.influence_distance",   node_params_.aapf.influence_distance);
  get_parameter("aapf.v_max",                node_params_.aapf.v_max);
  get_parameter("aapf.v_min",                node_params_.aapf.v_min);
  get_parameter("aapf.repulsive_saturation", node_params_.aapf.repulsive_saturation);
  get_parameter("aapf.v_rep_max",            node_params_.aapf.v_rep_max);
  get_parameter("aapf.command_frame",        node_params_.aapf.command_frame);

  get_parameter("servo_joint_topic",         node_params_.servo_joint_topic);
  get_parameter("joint_velocity_topic",      node_params_.joint_velocity_topic);
  get_parameter("enable_singularity_coast",  node_params_.enable_singularity_coast);
  get_parameter("singularity.kappa_enter",   node_params_.singularity.kappa_enter);
  get_parameter("singularity.kappa_exit",    node_params_.singularity.kappa_exit);

  if (node_params_.singularity.kappa_exit >= node_params_.singularity.kappa_enter)
    RCLCPP_WARN(get_logger(),
                "singularity.kappa_exit (%.1f) >= kappa_enter (%.1f): no hysteresis band, "
                "the joint-hold channel may chatter.",
                node_params_.singularity.kappa_exit, node_params_.singularity.kappa_enter);
}

Supervisor::Params SupervisorNode::buildCoreParams() const
{
  Supervisor::Params p;
  get_parameter("safe_distance",              p.safe_distance);
  get_parameter("blocked_distance_threshold", p.blocked_distance_threshold);
  get_parameter("emergency_distance",         p.emergency_distance);
  get_parameter("return_clearance",           p.return_clearance);
  get_parameter("lookahead_waypoint_count",   p.lookahead_waypoint_count);
  get_parameter("p1_search_horizon",          p.p1_search_horizon);
  get_parameter("p1_index_offset",            p.p1_index_offset);
  get_parameter("p2_index_offset",            p.p2_index_offset);
  get_parameter("max_p1_p2_distance",         p.max_p1_p2_distance);
  get_parameter("min_p1_p2_distance",         p.min_p1_p2_distance);
  get_parameter("return_to_path_distance",    p.return_to_path_distance);
  get_parameter("rejoin_waypoint_offset",     p.rejoin_waypoint_offset);
  get_parameter("vis_p2_offset",              p.vis_p2_offset);
  get_parameter("vis_p1_offset",              p.vis_p1_offset);
  get_parameter("subdivide_step_m",           p.subdivide_step_m);
  get_parameter("search_window_steps",        p.search_window_steps);
  get_parameter("avoid_timeout",              p.avoid_timeout);
  get_parameter("rejoin_timeout",             p.rejoin_timeout);
  get_parameter("max_path_deviation",         p.max_path_deviation);
  get_parameter("arm_joints",                 p.arm_joints);

  if (p.blocked_distance_threshold > p.safe_distance)
    RCLCPP_WARN(get_logger(),
                "blocked_distance_threshold (%.3f) > safe_distance (%.3f): a waypoint "
                "may be flagged 'blocked' yet still count as a safe P1/P2 — check tuning.",
                p.blocked_distance_threshold, p.safe_distance);
  return p;
}

// ============================================================================
// Deferred init (PSM + action client + timers)
// ============================================================================

void SupervisorNode::deferredInit()
{
  try
  {
    psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
        shared_from_this(), "robot_description");
    psm_->startStateMonitor();
    psm_->startSceneMonitor();
    psm_->startWorldGeometryMonitor();
    psm_->requestPlanningSceneState();
  }
  catch (const std::exception& e)
  {
    RCLCPP_FATAL(get_logger(), "PSM init failed: %s", e.what());
    return;
  }

  // Cache the arm JointModelGroup for Jacobian / condition-number computation.
  jmg_ = psm_->getRobotModel()->getJointModelGroup(node_params_.move_group_name);
  if (jmg_ == nullptr)
    RCLCPP_WARN(get_logger(),
                "Move group '%s' not found — singularity coast disabled (κ unavailable).",
                node_params_.move_group_name.c_str());

  // Re-IK resume prerequisite: the group needs an IK solver, which only exists if
  // robot_description_kinematics was loaded into THIS node's namespace (launch).
  if (jmg_ != nullptr)
  {
    if (jmg_->getSolverInstance())
      RCLCPP_INFO(get_logger(), "IK solver available for '%s' — re-IK resume usable.",
                  node_params_.move_group_name.c_str());
    else
      RCLCPP_WARN(get_logger(),
                  "No IK solver for '%s' — re-IK resume will be skipped. "
                  "Load kinematics.yaml into this node (see aapf_supervisor.launch.py).",
                  node_params_.move_group_name.c_str());
  }

  follow_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, node_params_.controller_follow_action);
  RCLCPP_INFO(get_logger(), "FollowJointTrajectory action client: '%s'",
              node_params_.controller_follow_action.c_str());

  // Real-robot controller switching (Stage 2b). Only wired when enabled.
  if (node_params_.enable_controller_switch)
  {
    switch_client_ = create_client<SwitchController>("/controller_manager/switch_controller");
    servo_cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        node_params_.servo_controller + "/commands", rclcpp::QoS(1));
    // Gate starts CLOSED in switch mode: only opened after a successful switch to FORWARD.
    servo_controller_active_ = false;
    current_controller_ = ActiveController::SCALED;   // bringup spawns scaled active
    desired_controller_ = ActiveController::SCALED;
    RCLCPP_INFO(get_logger(),
                "Controller switch ENABLED: motion='%s', servo='%s'.",
                node_params_.motion_controller.c_str(),
                node_params_.servo_controller.c_str());
  }
  else
  {
    // Sim / single-controller: gate permanently open, no switching.
    servo_controller_active_ = true;
  }

  supervisor_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / node_params_.obstacle_check_rate),
      [this] { supervisorTick(); });
  twist_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / node_params_.aapf_control_rate),
      [this] { twistTick(); });

  RCLCPP_INFO(get_logger(), "SupervisorNode(v2) ready (PSM + action client + timers up)");
}

// ============================================================================
// FK + scene helpers
// ============================================================================

Eigen::Vector3d SupervisorNode::computeEEPosition(
    moveit::core::RobotState& working_state, const std::vector<double>& q) const
{
  working_state.setJointGroupPositions(node_params_.move_group_name, q);
  working_state.updateLinkTransforms();
  const Eigen::Isometry3d T = working_state.getGlobalLinkTransform(node_params_.ee_link);
  return T * Eigen::Vector3d(0.0, 0.0, node_params_.gripper_z_offset);
}

std::vector<double> SupervisorNode::readArmJoints(const moveit::core::RobotState& s) const
{
  const auto& names = supervisor_->params().arm_joints;
  std::vector<double> q(names.size());
  for (std::size_t i = 0; i < names.size(); ++i)
  {
    const double* p = s.getJointPositions(names[i]);
    if (p == nullptr)
      throw std::runtime_error("Joint '" + names[i] + "' not in current state");
    q[i] = *p;
  }
  return q;
}

void SupervisorNode::jointVelocityCb(sensor_msgs::msg::JointState::SharedPtr msg)
{
  const auto& names = supervisor_->params().arm_joints;
  std::vector<double> qd(names.size(), 0.0);
  for (std::size_t i = 0; i < names.size(); ++i)
  {
    // Match each arm joint by name → its velocity entry. Missing → 0.
    for (std::size_t j = 0; j < msg->name.size(); ++j)
    {
      if (msg->name[j] == names[i])
      {
        if (j < msg->velocity.size())
          qd[i] = msg->velocity[j];
        break;
      }
    }
  }
  std::lock_guard<std::mutex> lock(qdot_mtx_);
  latest_arm_qdot_ = std::move(qd);
}

std::vector<double> SupervisorNode::latestArmVelocities() const
{
  std::lock_guard<std::mutex> lock(qdot_mtx_);
  return latest_arm_qdot_;
}

double SupervisorNode::conditionNumber(moveit::core::RobotState& s) const
{
  if (jmg_ == nullptr)
    return std::numeric_limits<double>::infinity();
  // Same computation as MoveIt Servo (servo_calcs.cpp): SVD of the full 6×N
  // Jacobian, κ = σ_max / σ_min. Keeps κ comparable to lower_singularity_threshold.
  s.updateLinkTransforms();
  const Eigen::MatrixXd jac = s.getJacobian(jmg_);
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(jac);
  const auto& sv = svd.singularValues();
  const double smin = sv(sv.size() - 1);
  if (smin <= 0.0)
    return std::numeric_limits<double>::infinity();
  return sv(0) / smin;
}

std::vector<Sphere> SupervisorNode::enumerateSpheres()
{
  std::vector<Sphere> out;
  if (!psm_) return out;

  planning_scene_monitor::LockedPlanningSceneRO scene(psm_);
  if (!scene) return out;

  std::stringstream dump;
  std::size_t total_shapes = 0, skipped = 0;

  for (const auto& obj_id : scene->getWorld()->getObjectIds())
  {
    const auto obj = scene->getWorld()->getObject(obj_id);
    for (std::size_t i = 0; i < obj->shapes_.size(); ++i)
    {
      ++total_shapes;
      if (obj->shapes_[i]->type != shapes::SPHERE)
      {
        ++skipped;
        dump << " [SKIP " << obj_id << "/" << i
             << " type=" << static_cast<int>(obj->shapes_[i]->type) << "]";
        continue;
      }
      const auto sp = std::static_pointer_cast<const shapes::Sphere>(obj->shapes_[i]);
      const Eigen::Vector3d c = (i < obj->global_shape_poses_.size())
                                    ? obj->global_shape_poses_[i].translation()
                                    : (obj->pose_ * obj->shape_poses_[i]).translation();
      out.push_back({ c, sp->radius });
      dump << " [KEEP " << obj_id << " r=" << sp->radius
           << " @(" << c.x() << "," << c.y() << "," << c.z() << ")]";
    }
  }

  auto& clk = *get_clock();
  RCLCPP_INFO_THROTTLE(get_logger(), clk, 3000,
      "[scene] shapes=%zu kept_spheres=%zu skipped=%zu :%s",
      total_shapes, out.size(), skipped, dump.str().c_str());

  return out;
}

// ============================================================================
// Trajectory subscription
// ============================================================================

void SupervisorNode::trajectoryCb(moveit_msgs::msg::DisplayTrajectory::SharedPtr msg)
{
  if (!psm_)
  {
    auto& clk = *get_clock();
    RCLCPP_WARN_THROTTLE(get_logger(), clk, 2000,
                         "Trajectory arrived before PSM was ready — dropping");
    return;
  }
  if (msg->trajectory.empty())
  {
    RCLCPP_WARN(get_logger(), "DisplayTrajectory had no RobotTrajectory entries");
    return;
  }

  const auto& jt = msg->trajectory.front().joint_trajectory;
  if (jt.points.empty())
  {
    RCLCPP_WARN(get_logger(), "JointTrajectory points empty");
    return;
  }

  moveit::core::RobotState working_state(psm_->getRobotModel());
  {
    planning_scene_monitor::LockedPlanningSceneRO scene(psm_);
    if (!scene)
    {
      RCLCPP_ERROR(get_logger(), "No planning scene during FK setup");
      return;
    }
    working_state = scene->getCurrentState();
  }

  Supervisor::FKFunction fk =
      [this, &working_state](const std::vector<double>& q) {
        return computeEEPosition(working_state, q);
      };

  auto stats = supervisor_->ingestTrajectory(jt, fk);
  if (stats)
    RCLCPP_INFO(get_logger(),
                "New trajectory: %zu raw (%zu joints) → %zu subdivided",
                stats->raw, stats->kept_joints, stats->subdivided);
  else
    RCLCPP_ERROR(get_logger(),
                 "ingestTrajectory failed — check arm_joints vs incoming joint list");
}

// ============================================================================
// Slow tick — mode decision + transitions + diagnostics
// ============================================================================

void SupervisorNode::supervisorTick()
{
  if (!supervisor_->isRunning())
    return;

  Eigen::Vector3d ee_now = Eigen::Vector3d::Zero();
  bool inputs_fresh = true;
  double kappa = std::numeric_limits<double>::infinity();

  try
  {
    planning_scene_monitor::LockedPlanningSceneRO scene(psm_);
    if (!scene)
    {
      inputs_fresh = false;
    }
    else
    {
      moveit::core::RobotState current_state = scene->getCurrentState();
      const auto q = readArmJoints(current_state);
      ee_now = computeEEPosition(current_state, q);
      kappa  = conditionNumber(current_state);   // singularity proximity (all modes)
    }
  }
  catch (const std::exception& e)
  {
    auto& clk = *get_clock();
    RCLCPP_WARN_THROTTLE(get_logger(), clk, 2000, "Tick FK failed: %s", e.what());
    inputs_fresh = false;
  }

  std::vector<Sphere> spheres;
  if (inputs_fresh)
    spheres = enumerateSpheres();

  Supervisor::TickInput in{ ee_now, std::move(spheres), inputs_fresh, now().seconds() };
  const auto out = supervisor_->tick(in);

  if (out.mode_changed)
    onModeTransition(out, ee_now);

  publishStatus(out);
  publishMarkers(out, ee_now);

  // Logging debug at the slow-tick rate: ee_xyz always fresh; v_* are the last
  // values cached by twistTick (zeroed there when not avoiding). twistTick also
  // publishes this topic at the fast rate while avoiding (higher-fidelity v_*).
  if (inputs_fresh)
    publishLogDebug(ee_now);

  // Singularity proximity (κ) + boolean flag — diagnostic siblings of
  // ~/obstacle_distance, published every tick in every mode.
  if (inputs_fresh && jmg_ != nullptr && std::isfinite(kappa))
  {
    std_msgs::msg::Float64 km; km.data = kappa;          condition_pub_->publish(km);
    std_msgs::msg::Bool    nm;
    nm.data = (kappa > node_params_.singularity.kappa_enter);
    near_sing_pub_->publish(nm);
  }

  // Measured EE Cartesian speed [m/s], FILTERED: finite-difference of the FK EE
  // position over a WINDOW (~ee_speed_window_sec) rather than a single 30 Hz step.
  // The single-step version aliased to 0.0/spikes when the PSM state didn't update
  // between two ticks; the window averages that out. Compare to |/delta_twist_cmds|.
  if (inputs_fresh)
  {
    const double t = now().seconds();
    ee_hist_.emplace_back(t, ee_now);
    // Keep only samples within the window, but always at least two for a difference.
    while (ee_hist_.size() > 2 &&
           (t - ee_hist_.front().first) > node_params_.ee_speed_window_sec)
      ee_hist_.pop_front();

    if (ee_hist_.size() >= 2)
    {
      const auto&  oldest = ee_hist_.front();
      const double dt = t - oldest.first;
      if (dt > 1e-6)
      {
        std_msgs::msg::Float64 sm;
        sm.data = (ee_now - oldest.second).norm() / dt;   // |Δee over window| / Δt
        ee_speed_pub_->publish(sm);
      }
    }
  }
  else
  {
    ee_hist_.clear();   // gap in state → restart the window cleanly
  }

  auto& clk = *get_clock();
  RCLCPP_INFO_THROTTLE(get_logger(), clk, 1000,
      "[v2] mode=%s err=%s d_ee=%.3f d_look=%.3f dev=%.3f idx=%zu/%zu",
      toString(out.mode).data(), toString(out.error).data(),
      out.ee_clearance, out.lookahead_min_clearance, out.path_deviation,
      out.current_idx, out.cache_size);
}

// ============================================================================
// Mode transitions — controller handoff
// ============================================================================

void SupervisorNode::onModeTransition(const Supervisor::TickOutput& out,
                                      const Eigen::Vector3d& ee_now)
{
  RCLCPP_INFO(get_logger(), "Mode change: %s -> %s (err=%s, d_ee=%.3f, idx=%zu/%zu)",
              toString(out.previous_mode).data(), toString(out.mode).data(),
              toString(out.error).data(), out.ee_clearance,
              out.current_idx, out.cache_size);

  // ---- Single-controller path (sim): ORIGINAL behaviour, unchanged ----
  if (!node_params_.enable_controller_switch)
  {
    switch (out.mode)
    {
      case Mode::AAPF_AVOID:
        // Took over from the global controller — cancel its active goal so the
        // streamed Servo commands aren't fought by the trajectory execution.
        cancelTrajectoryController();
        break;

      case Mode::STOP_OR_REPLAN:
        // Halt: stop Servo and cancel the controller. Replan is report-only.
        cancelTrajectoryController();
        twist_pub_->publish(aapf_->zeroTwist(now()));
        RCLCPP_ERROR(get_logger(),
                     "STOP_OR_REPLAN — error=%s. Robot halted; call ~/reset (and replan) to recover.",
                     toString(out.error).data());
        break;

      case Mode::FOLLOW_TRAJECTORY:
        // Returned to following. Stop streaming twist; optionally resume the
        // remaining global path through the controller.
        twist_pub_->publish(aapf_->zeroTwist(now()));
        if (out.previous_mode == Mode::RETURN_TO_TRAJECTORY &&
            node_params_.resume_trajectory_on_rejoin)
          resumeTrajectoryController(ee_now);
        break;

      default:
        break;
    }
    return;
  }

  // ---- Two-controller path (real): switch scaled <-> forward + twist gate ----
  // Side-effects that need the target controller ACTIVE (open gate / resume) are
  // DEFERRED into the switch-success callback. Latest desired wins; fail -> HALT.
  switch (out.mode)
  {
    case Mode::AAPF_AVOID:
      cancelTrajectoryController();        // abort scaled goal (switch also deactivates it)
      servo_controller_active_ = false;    // close gate until forward confirmed active
      requestControllerSwitch(ActiveController::FORWARD,
                              [this] { servo_controller_active_ = true; });
      break;

    case Mode::STOP_OR_REPLAN:
      cancelTrajectoryController();
      servo_controller_active_ = false;    // stop streaming immediately (halt is urgent)
      twist_pub_->publish(aapf_->zeroTwist(now()));
      requestControllerSwitch(ActiveController::SCALED, [] {});  // restore canonical state
      RCLCPP_ERROR(get_logger(),
                   "STOP_OR_REPLAN — error=%s. Robot halted; call ~/reset (and replan) to recover.",
                   toString(out.error).data());
      break;

    case Mode::FOLLOW_TRAJECTORY:
    {
      servo_controller_active_ = false;    // ngừng stream TRƯỚC khi switch về scaled
      twist_pub_->publish(aapf_->zeroTwist(now()));
      const bool do_resume = (out.previous_mode == Mode::RETURN_TO_TRAJECTORY &&
                              node_params_.resume_trajectory_on_rejoin);
      const Eigen::Vector3d ee = ee_now;
      requestControllerSwitch(ActiveController::SCALED,
                              [this, ee, do_resume] { if (do_resume) resumeTrajectoryController(ee); });
      break;
    }

    default:  // CHECK_AAPF_CONDITION, RETURN_TO_TRAJECTORY — keep current controller
      break;
  }
}

void SupervisorNode::cancelTrajectoryController()
{
  if (!follow_client_)
    return;
  if (!follow_client_->action_server_is_ready())
  {
    RCLCPP_WARN(get_logger(),
                "FollowJointTrajectory server '%s' not ready — cannot cancel global goal",
                node_params_.controller_follow_action.c_str());
    return;
  }
  follow_client_->async_cancel_all_goals();
  RCLCPP_INFO(get_logger(), "Cancelled active FollowJointTrajectory goal(s).");
}

std::optional<trajectory_msgs::msg::JointTrajectory>
SupervisorNode::reSolveResumeIK(const trajectory_msgs::msg::JointTrajectory& suffix,
                                const std::vector<double>& q_now) const
{
  if (suffix.points.size() < 2 || jmg_ == nullptr || !psm_)
    return std::nullopt;
  if (!jmg_->getSolverInstance())
  {
    RCLCPP_WARN(get_logger(), "Re-IK resume: no IK solver for group '%s'.",
                node_params_.move_group_name.c_str());
    return std::nullopt;
  }

  const auto& model = psm_->getRobotModel();
  robot_trajectory::RobotTrajectory rt(model, node_params_.move_group_name);

  // Seed state = real current joints. setFromIK seeds from the state it is called
  // on, so chaining (seed = previous solution) keeps the whole suffix on one IK
  // branch — no wrist-flip jump.
  moveit::core::RobotState seed(model);
  seed.setToDefaultValues();
  seed.setJointGroupPositions(node_params_.move_group_name, q_now);
  seed.update();

  // Prepend the REAL current config as the first waypoint so JTC's start-state
  // tolerance (current vs point[0]) passes. The orientation drifted during the
  // Cartesian avoidance, so the IK of pose[start] still sits a wrist-rotation away
  // from q_now; making q_now the first point turns that correction into a timed
  // interior segment instead of an illegal jump at t=0.
  rt.addSuffixWayPoint(seed, 0.0);

  // Scratch state for FK of the ORIGINAL joint values → full 6-DOF target pose.
  moveit::core::RobotState fk_state(model);
  fk_state.setToDefaultValues();

  std::size_t solved = 0, failed = 0;
  for (const auto& pt : suffix.points)
  {
    fk_state.setJointGroupPositions(node_params_.move_group_name, pt.positions);
    fk_state.update();
    const Eigen::Isometry3d target = fk_state.getGlobalLinkTransform(node_params_.ee_link);

    moveit::core::RobotState sol(seed);  // continuation: solve nearest to previous
    const bool ok = sol.setFromIK(jmg_, target, node_params_.ee_link, /*timeout=*/0.05);
    if (!ok) { ++failed; continue; }
    rt.addSuffixWayPoint(sol, /*dt placeholder, TOTG overwrites=*/0.1);
    seed = sol;
    ++solved;
  }

  if (rt.getWayPointCount() < 2)
  {
    RCLCPP_WARN(get_logger(), "Re-IK resume failed: only %zu/%zu waypoints solved.",
                solved, suffix.points.size());
    return std::nullopt;
  }

  // Re-time: assign velocities/accelerations within the robot's joint limits,
  // scaled to match the ORIGINAL plan's speed (TOTG defaults to 100% otherwise,
  // so the resumed segment would run far faster than the pre-avoidance motion).
  trajectory_processing::TimeOptimalTrajectoryGeneration totg;
  if (!totg.computeTimeStamps(rt,
                              node_params_.resume_velocity_scaling,
                              node_params_.resume_acceleration_scaling))
  {
    RCLCPP_WARN(get_logger(), "Re-IK resume: TOTG failed to time-parameterize.");
    return std::nullopt;
  }

  moveit_msgs::msg::RobotTrajectory msg;
  rt.getRobotTrajectoryMsg(msg);
  const double dur = rt.getWayPointDurationFromStart(rt.getWayPointCount() - 1);

  // EE-endpoint diagnostics: does the re-IK output actually reach the original
  // destination B? Compare the LAST re-IK waypoint's EE against B = FK(original
  // last point). If they differ, re-IK truncated geometrically; if they match but
  // the robot stops short, the issue is downstream (JTC timing / tolerances).
  const Eigen::Vector3d ee_first =
      rt.getWayPoint(0).getGlobalLinkTransform(node_params_.ee_link).translation();
  const Eigen::Vector3d ee_last =
      rt.getWayPoint(rt.getWayPointCount() - 1).getGlobalLinkTransform(node_params_.ee_link).translation();
  fk_state.setJointGroupPositions(node_params_.move_group_name, suffix.points.back().positions);
  fk_state.update();
  const Eigen::Vector3d ee_B = fk_state.getGlobalLinkTransform(node_params_.ee_link).translation();
  RCLCPP_INFO(get_logger(),
              "[re-IK diag] suffix=%zu solved=%zu failed=%zu out=%zu dur=%.2fs | "
              "ee_first=(%.3f,%.3f,%.3f) ee_last=(%.3f,%.3f,%.3f) B=(%.3f,%.3f,%.3f) "
              "|last-B|=%.3fm",
              suffix.points.size(), solved, failed, rt.getWayPointCount(), dur,
              ee_first.x(), ee_first.y(), ee_first.z(),
              ee_last.x(), ee_last.y(), ee_last.z(),
              ee_B.x(), ee_B.y(), ee_B.z(), (ee_last - ee_B).norm());

  // Continuity + timing audit of the OUTPUT msg (what JTC actually sees):
  //  - max single-joint step between consecutive points (detects KDL branch jumps),
  //  - whether time_from_start is strictly increasing (JTC samples by time).
  double max_step = 0.0; std::size_t step_at = 0; int step_joint = -1;
  bool t_monotonic = true; double t_prev = -1.0; std::size_t t_break = 0;
  for (std::size_t i = 0; i < msg.joint_trajectory.points.size(); ++i)
  {
    const auto& p = msg.joint_trajectory.points[i];
    const double t = p.time_from_start.sec + p.time_from_start.nanosec * 1e-9;
    if (t <= t_prev && i > 0) { t_monotonic = false; if (!t_break) t_break = i; }
    t_prev = t;
    if (i > 0)
    {
      const auto& q0 = msg.joint_trajectory.points[i - 1].positions;
      const auto& q1 = p.positions;
      for (std::size_t j = 0; j < q1.size(); ++j)
      {
        const double d = std::abs(q1[j] - q0[j]);
        if (d > max_step) { max_step = d; step_at = i; step_joint = static_cast<int>(j); }
      }
    }
  }
  RCLCPP_INFO(get_logger(),
              "[re-IK audit] max joint step=%.3frad at point %zu (joint %d); "
              "time monotonic=%s%s",
              max_step, step_at, step_joint, t_monotonic ? "yes" : "NO",
              t_monotonic ? "" : (" first break at point " + std::to_string(t_break)).c_str());
  return msg.joint_trajectory;
}

void SupervisorNode::resumeTrajectoryController(const Eigen::Vector3d& ee_now)
{
  if (!follow_client_)
    return;

  auto suffix = supervisor_->buildResumeFrom(ee_now);
  if (!suffix || suffix->points.size() < 2)
  {
    RCLCPP_INFO(get_logger(), "No remaining trajectory to resume (at/near path end).");
    return;
  }

  // Diagnostic: how much of the ORIGINAL path remains in the suffix. The last
  // point's (rebased) time_from_start == remaining original duration. If this is
  // tiny while the full path was long, nearestWaypointIndexGlobal jumped the start
  // index toward the end (resume picks a near-the-goal waypoint).
  {
    const auto& last_t = suffix->points.back().time_from_start;
    const std::size_t total = supervisor_->originalPointCount();
    const std::size_t start = (total >= suffix->points.size()) ? total - suffix->points.size() : 0;
    RCLCPP_INFO(get_logger(),
                "[resume diag] start_idx=%zu/%zu  suffix=%zu points  remaining=%.2fs / total=%.2fs",
                start, total, suffix->points.size(),
                last_t.sec + last_t.nanosec * 1e-9, supervisor_->originalDurationSec());
  }

  trajectory_msgs::msg::JointTrajectory to_send = *suffix;  // legacy: raw replay

  if (node_params_.reik_resume)
  {
    // Read the REAL current joint configuration (fresh, at resume time) as IK seed.
    std::vector<double> q_now;
    try
    {
      planning_scene_monitor::LockedPlanningSceneRO scene(psm_);
      if (scene)
        q_now = readArmJoints(scene->getCurrentState());
    }
    catch (const std::exception& e)
    {
      RCLCPP_WARN(get_logger(), "Re-IK resume: cannot read current state (%s).", e.what());
    }

    if (q_now.empty())
    {
      RCLCPP_WARN(get_logger(), "Re-IK resume: no current state — NOT resuming (avoiding a jump).");
      return;
    }

    auto reik = reSolveResumeIK(*suffix, q_now);
    if (!reik || reik->points.size() < 2)
    {
      // The raw suffix would jump joints (the original bug) — refuse to send it.
      RCLCPP_WARN(get_logger(), "Re-IK resume produced no usable trajectory — NOT resuming.");
      return;
    }
    to_send = std::move(*reik);
  }

  if (!follow_client_->action_server_is_ready())
  {
    RCLCPP_WARN(get_logger(), "Cannot resume: FollowJointTrajectory server not ready.");
    return;
  }

  // header.stamp == 0 means "execute from the beginning" (start now).
  to_send.header.stamp = rclcpp::Time(0, 0, get_clock()->get_clock_type());

  FollowJointTrajectory::Goal goal;
  goal.trajectory = to_send;
  const double dur = to_send.points.back().time_from_start.sec +
                     to_send.points.back().time_from_start.nanosec * 1e-9;

  // Delay the send so Servo's halt burst (it streams to the SAME JTC command
  // topic and would preempt this action goal) finishes first. Skip the delay when
  // controller switching is on (real robot: Servo and motion use separate
  // controllers, no contention). One-shot timer; cancels itself on first fire.
  const double delay = node_params_.enable_controller_switch ? 0.0 : node_params_.resume_delay_sec;
  resume_timer_ = create_wall_timer(
      std::chrono::duration<double>(delay),
      [this, goal, dur]() {
        resume_timer_->cancel();
        if (!follow_client_ || !follow_client_->action_server_is_ready())
        {
          RCLCPP_WARN(get_logger(), "Cannot resume: FollowJointTrajectory server not ready.");
          return;
        }
        follow_client_->async_send_goal(goal);
        RCLCPP_INFO(get_logger(), "Resumed global trajectory: %zu points (%s), duration=%.2fs.",
                    goal.trajectory.points.size(),
                    node_params_.reik_resume ? "re-IK" : "raw", dur);
      });
}

// ============================================================================
// Real-robot controller switch (scaled_jtc <-> forward_position_controller)
// Chained single-in-flight model: latest desired wins; on success run the
// pending side-effect (open gate / resume); on failure HALT.
// ============================================================================

void SupervisorNode::requestControllerSwitch(ActiveController target,
                                             std::function<void()> on_success)
{
  desired_controller_ = target;
  pending_switch_cb_  = std::move(on_success);
  if (!switch_inflight_)
    driveControllerSwitch();
}

void SupervisorNode::driveControllerSwitch()
{
  // Already in the desired state -> just run the pending side-effect.
  if (current_controller_ == desired_controller_)
  {
    auto cb = std::move(pending_switch_cb_);
    pending_switch_cb_ = nullptr;
    if (cb) cb();
    return;
  }

  if (!switch_client_ || !switch_client_->service_is_ready())
  {
    onControllerSwitchFailed("switch_controller service not ready");
    return;
  }

  const ActiveController target = desired_controller_;

  // Anti-jump: seed the forward controller's command buffer with the CURRENT
  // pose BEFORE it activates (servo is idle during FOLLOW, buffer would be stale).
  if (target == ActiveController::FORWARD)
    primeServoController();

  auto req = std::make_shared<SwitchController::Request>();
  if (target == ActiveController::FORWARD)
  {
    req->activate_controllers   = {node_params_.servo_controller};
    req->deactivate_controllers = {node_params_.motion_controller};
  }
  else
  {
    req->activate_controllers   = {node_params_.motion_controller};
    req->deactivate_controllers = {node_params_.servo_controller};
  }
  req->strictness = SwitchController::Request::STRICT;

  switch_inflight_ = true;
  switch_client_->async_send_request(
      req,
      [this, target](rclcpp::Client<SwitchController>::SharedFuture fut)
      {
        switch_inflight_ = false;
        auto resp = fut.get();
        if (!resp || !resp->ok)
        {
          onControllerSwitchFailed("controller_manager rejected switch");
          return;
        }
        current_controller_ = target;
        RCLCPP_INFO(get_logger(), "Controller active: %s",
                    target == ActiveController::FORWARD
                        ? node_params_.servo_controller.c_str()
                        : node_params_.motion_controller.c_str());
        // Chain: if desired changed mid-flight, issue next switch; else run cb.
        driveControllerSwitch();
      });
}

void SupervisorNode::primeServoController()
{
  if (!servo_cmd_pub_)
    return;
  try
  {
    planning_scene_monitor::LockedPlanningSceneRO scene(psm_);
    if (!scene)
      return;
    moveit::core::RobotState current_state = scene->getCurrentState();
    const auto q = readArmJoints(current_state);   // order = arm_joints (matches controller)
    std_msgs::msg::Float64MultiArray cmd;
    cmd.data.assign(q.begin(), q.end());
    servo_cmd_pub_->publish(cmd);
  }
  catch (const std::exception& e)
  {
    RCLCPP_WARN(get_logger(), "primeServoController failed: %s", e.what());
  }
}

void SupervisorNode::onControllerSwitchFailed(const std::string& why)
{
  servo_controller_active_ = false;                  // gate closed -> no streaming
  twist_pub_->publish(aapf_->zeroTwist(now()));      // halt Servo
  pending_switch_cb_ = nullptr;
  RCLCPP_ERROR(get_logger(),
               "Controller switch FAILED (%s) — HALT. Robot holds; fix and call ~/reset.",
               why.c_str());
}

// ============================================================================
// Status publish
// ============================================================================

void SupervisorNode::publishStatus(const Supervisor::TickOutput& out)
{
  std_msgs::msg::Int8 m;   m.data = static_cast<int8_t>(out.mode);  mode_pub_->publish(m);
  std_msgs::msg::String mt; mt.data = std::string(toString(out.mode)); mode_text_pub_->publish(mt);
  std_msgs::msg::Int8 e;   e.data = static_cast<int8_t>(out.error); error_pub_->publish(e);
  std_msgs::msg::String et; et.data = std::string(toString(out.error)); error_text_pub_->publish(et);
  std_msgs::msg::Float64 d; d.data = out.ee_clearance; distance_pub_->publish(d);
}

// ----------------------------------------------------------------------------
// RViz markers: EE (blue), P2 near/safety (yellow), P1 far/attractor (green),
// and an EE→P1 line. Markers absent this tick are deleted so RViz doesn't keep
// stale spheres. namespace "aapf" with stable ids → in-place updates.
// ----------------------------------------------------------------------------
void SupervisorNode::publishMarkers(const Supervisor::TickOutput& out,
                                    const Eigen::Vector3d& ee_now)
{
  const auto stamp = now();
  const std::string& frame = node_params_.planning_frame;

  auto sphere = [&](int id, double r, float cr, float cg, float cb, float ca,
                    const Eigen::Vector3d& p) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame;
    m.header.stamp = stamp;
    m.ns = "aapf";
    m.id = id;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = p.x();
    m.pose.position.y = p.y();
    m.pose.position.z = p.z();
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = r;
    m.color.r = cr; m.color.g = cg; m.color.b = cb; m.color.a = ca;
    return m;
  };
  auto del = [&](int id) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame;
    m.header.stamp = stamp;
    m.ns = "aapf";
    m.id = id;
    m.action = visualization_msgs::msg::Marker::DELETE;
    return m;
  };

  visualization_msgs::msg::MarkerArray arr;

  // id 0 — EE (current end-effector tip), always shown.
  arr.markers.push_back(sphere(0, 0.04, 0.1f, 0.4f, 1.0f, 1.0f, ee_now));

  // id 1 — P1 real (far, attractor), solid green. Only during avoidance.
  if (out.p1_pos) arr.markers.push_back(sphere(1, 0.05, 0.0f, 1.0f, 0.2f, 1.0f, *out.p1_pos));
  else            arr.markers.push_back(del(1));

  // id 2 — P2 real (near, safety-confirm), solid yellow. Only during avoidance.
  if (out.p2_pos) arr.markers.push_back(sphere(2, 0.05, 1.0f, 0.9f, 0.0f, 1.0f, *out.p2_pos));
  else            arr.markers.push_back(del(2));

  // id 4 — fixed debug lookahead P1 (far), translucent green. Always on.
  if (out.fixed_p1_pos) arr.markers.push_back(sphere(4, 0.035, 0.2f, 1.0f, 0.4f, 0.4f, *out.fixed_p1_pos));
  else                  arr.markers.push_back(del(4));

  // id 5 — fixed debug lookahead P2 (near), translucent yellow. Always on.
  if (out.fixed_p2_pos) arr.markers.push_back(sphere(5, 0.035, 1.0f, 0.95f, 0.4f, 0.4f, *out.fixed_p2_pos));
  else                  arr.markers.push_back(del(5));

  // id 3 — EE → goal (P1 / rejoin) attractor line.
  if (out.goal)
  {
    visualization_msgs::msg::Marker line;
    line.header.frame_id = frame;
    line.header.stamp = stamp;
    line.ns = "aapf";
    line.id = 3;
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.scale.x = 0.01;  // line width
    line.color.r = 0.0f; line.color.g = 1.0f; line.color.b = 0.2f; line.color.a = 0.8f;
    line.pose.orientation.w = 1.0;
    geometry_msgs::msg::Point a, b;
    a.x = ee_now.x(); a.y = ee_now.y(); a.z = ee_now.z();
    b.x = out.goal->x(); b.y = out.goal->y(); b.z = out.goal->z();
    line.points.push_back(a);
    line.points.push_back(b);
    arr.markers.push_back(line);
  }
  else
  {
    arr.markers.push_back(del(3));
  }

  marker_pub_->publish(arr);
}

// ============================================================================
// Fast tick — AAPF twist (AAPF_AVOID or RETURN_TO_TRAJECTORY)
// ============================================================================

void SupervisorNode::publishLogDebug(const Eigen::Vector3d& ee_now)
{
  std_msgs::msg::Float64MultiArray msg;
  msg.data = {
    last_v_att_.x(), last_v_att_.y(), last_v_att_.z(),
    last_v_rep_.x(), last_v_rep_.y(), last_v_rep_.z(),
    last_v_cmd_.x(), last_v_cmd_.y(), last_v_cmd_.z(),
    ee_now.x(),      ee_now.y(),      ee_now.z(),
  };
  log_debug_pub_->publish(msg);
  publishVelocityMarkers(ee_now);
}

void SupervisorNode::publishVelocityMarkers(const Eigen::Vector3d& ee_now)
{
  visualization_msgs::msg::MarkerArray arr;
  const double s = velocity_arrow_scale_;

  // (vector, id, r,g,b) — v_att hút (xanh lá), v_rep đẩy (đỏ), v_cmd tổng (vàng).
  const std::array<std::tuple<Eigen::Vector3d, int, float, float, float>, 3> defs = {{
    { last_v_att_, 0, 0.0f, 1.0f, 0.0f },
    { last_v_rep_, 1, 1.0f, 0.0f, 0.0f },
    { last_v_cmd_, 2, 1.0f, 1.0f, 0.0f },
  }};

  for (const auto& [vec, id, r, g, b] : defs)
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = node_params_.planning_frame;
    m.header.stamp = now();
    m.ns = "aapf_velocity";
    m.id = id;
    m.type = visualization_msgs::msg::Marker::ARROW;
    m.action = visualization_msgs::msg::Marker::ADD;
    const Eigen::Vector3d tip = ee_now + vec * s;
    geometry_msgs::msg::Point p0, p1;
    p0.x = ee_now.x(); p0.y = ee_now.y(); p0.z = ee_now.z();
    p1.x = tip.x();    p1.y = tip.y();    p1.z = tip.z();
    m.points = { p0, p1 };
    m.scale.x = 0.008;  // shaft diameter
    m.scale.y = 0.02;   // head diameter
    m.scale.z = 0.03;   // head length
    m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 1.0f;
    m.lifetime = rclcpp::Duration::from_seconds(0.2);
    arr.markers.push_back(m);
  }
  vel_marker_pub_->publish(arr);
}

void SupervisorNode::twistTick()
{
  const Mode m = supervisor_->mode();
  if (m != Mode::AAPF_AVOID && m != Mode::RETURN_TO_TRAJECTORY)
  {
    // Not avoiding → no AAPF velocity. Zero the cached decomposition so the
    // logger records 0 for v_att/v_rep/v_cmd during FOLLOW/STOP.
    last_v_att_.setZero();
    last_v_rep_.setZero();
    last_v_cmd_.setZero();
    // Avoidance ended → drop any coast state so the next maneuver starts on the
    // Cartesian channel ("trục 2" reset, see SingularityHandler).
    if (singularity_->coasting())
      singularity_->reset();
    return;
  }

  // Gate (real mode): never stream until the servo_controller is confirmed active.
  // Always open when switching is disabled (sim, single controller).
  if (node_params_.enable_controller_switch && !servo_controller_active_.load())
    return;

  // Attractor decided by the last (slow) decision tick. The fast loop only
  // re-evaluates the field at the current EE — it never changes the state.
  const auto goal = supervisor_->currentGoal();
  if (!goal)
    return;

  Eigen::Vector3d     ee_now;
  double              kappa = std::numeric_limits<double>::infinity();
  std::vector<double> qdot;
  try
  {
    planning_scene_monitor::LockedPlanningSceneRO scene(psm_);
    if (!scene)
      return;
    moveit::core::RobotState current_state = scene->getCurrentState();
    const auto q = readArmJoints(current_state);
    ee_now = computeEEPosition(current_state, q);
    if (node_params_.enable_singularity_coast && jmg_ != nullptr)
      kappa = conditionNumber(current_state);
  }
  catch (const std::exception&)
  {
    return;
  }

  const auto spheres = enumerateSpheres();
  AAPFGenerator::Decomposition dbg;
  const auto twist   = aapf_->computeTwist(ee_now, *goal, spheres, now(), &dbg);
  last_v_att_ = dbg.v_att;
  last_v_rep_ = dbg.v_rep;
  last_v_cmd_ = dbg.v_cmd;
  publishLogDebug(ee_now);   // fast-rate (200 Hz) fresh velocities while avoiding

  // Trục 2: near a singularity, switch from Cartesian twist (Servo decelerates)
  // to a held joint velocity (Servo's joint path skips singularity scaling).
  if (node_params_.enable_singularity_coast && jmg_ != nullptr)
  {
    qdot = latestArmVelocities();   // measured q̇ direct from /joint_states
    Eigen::VectorXd qd = Eigen::Map<Eigen::VectorXd>(qdot.data(), static_cast<Eigen::Index>(qdot.size()));
    const ChannelDecision decision = singularity_->update(kappa, qd);
    if (decision.channel == VelocityChannel::JOINT_HOLD)
    {
      control_msgs::msg::JointJog jog;
      jog.header.stamp  = now();
      jog.joint_names   = supervisor_->params().arm_joints;
      jog.velocities.assign(decision.joint_velocity.data(),
                            decision.joint_velocity.data() + decision.joint_velocity.size());
      joint_pub_->publish(jog);   // → servo_joint_topic (/delta_joint_cmds)
      return;
    }
  }

  twist_pub_->publish(twist);
}

// ============================================================================
// Service callbacks
// ============================================================================

void SupervisorNode::startCb(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  supervisor_->reset();          // FOLLOW_TRAJECTORY, clear any prior error
  supervisor_->setRunning(true); // setRunning(true) preserves current mode
  res->success = true;
  res->message = "supervisor running";
  RCLCPP_INFO(get_logger(), "Supervisor started");
}

void SupervisorNode::stopCb(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  supervisor_->setRunning(false);
  twist_pub_->publish(aapf_->zeroTwist(now()));
  res->success = true;
  res->message = "supervisor stopped";
  RCLCPP_INFO(get_logger(), "Supervisor stopped");
}

void SupervisorNode::resetCb(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  supervisor_->reset();
  // Restore canonical controller state (scaled active) so a subsequent replan/
  // execute via move_group works. Idempotent if already on scaled.
  if (node_params_.enable_controller_switch)
  {
    servo_controller_active_ = false;
    requestControllerSwitch(ActiveController::SCALED, [] {});
  }
  res->success = true;
  res->message = "reset to FOLLOW_TRAJECTORY";
  RCLCPP_INFO(get_logger(), "Supervisor reset");
}

}  // namespace custom_ur_aapf_2
