#pragma once

// SupervisorNode (v2) — ROS layer for the AAPF local-avoidance supervisor.
//
// Owns: PSM, AAPFGenerator, the FollowJointTrajectory action client (used to
// cancel the global trajectory when avoidance starts and to resume the
// remaining path on rejoin), ROS pubs/subs/services/timers.
// Delegates all decision logic to the Supervisor core.

#include <atomic>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <utility>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "custom_ur_aapf_2/aapf_generator.hpp"
#include "custom_ur_aapf_2/geometry_types.hpp"
#include "custom_ur_aapf_2/singularity_handler.hpp"
#include "custom_ur_aapf_2/supervisor.hpp"

namespace custom_ur_aapf_2
{

class SupervisorNode : public rclcpp::Node
{
public:
  explicit SupervisorNode(const rclcpp::NodeOptions& options);
  ~SupervisorNode() override;

private:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using SwitchController       = controller_manager_msgs::srv::SwitchController;

  // Which controller currently owns the joint command interfaces (real mode).
  enum class ActiveController { SCALED, FORWARD };

  struct NodeParams
  {
    // Timer rates (§12: time/frequency).
    double obstacle_check_rate = 30.0;   // mode-decision / lookahead scan [Hz]
    double aapf_control_rate   = 200.0;  // twist publish rate while avoiding [Hz]

    double scene_stale_sec  = 0.5;
    double gripper_z_offset = 0.163;     // tool0 → gripper tip along local Z
    double ee_speed_window_sec = 0.2;    // window for filtered ~/ee_speed [s]

    // Topics.
    std::string trajectory_topic  = "/display_planned_path";
    std::string servo_twist_topic = "/servo_node/delta_twist_cmds";
    std::string servo_joint_topic = "/delta_joint_cmds";   // JointJog → Servo joint path
    std::string joint_velocity_topic = "/arm_joint_velocities";  // q̇ in, from JointVelocityEstimator

    // Singularity coast ("trục 2"): near a singularity, switch the avoidance
    // command from Cartesian twist (Servo decelerates) to a held joint velocity
    // on servo_joint_topic (Servo's joint path skips singularity scaling), so the
    // arm coasts through at constant joint speed. Off by default → sim unchanged.
    bool                       enable_singularity_coast = false;
    SingularityHandler::Params singularity;

    // Trajectory controller (cancel on avoid, resume on rejoin).
    std::string controller_follow_action = "/joint_trajectory_controller/follow_joint_trajectory";
    bool        resume_trajectory_on_rejoin = true;

    // Re-IK resume (Cách 3): on rejoin, re-solve the remaining path's IK seeded
    // from the REAL current joint config (continuation IK), then re-time with TOTG,
    // instead of replaying the original joint values. Fixes the joint-mismatch
    // (wrist-flip) state-tolerance abort. false → legacy raw replay.
    bool        reik_resume = false;

    // Delay [s] before sending the resume goal after rejoin. Servo and JTC share
    // ONE controller in sim; on FOLLOW, Servo fires a halt burst
    // (incoming_command_timeout + num_outgoing_halt_msgs) onto the JTC command
    // TOPIC, which preempts the resume action goal. Waiting until that burst ends
    // (~0.3s) lets the resume run uninterrupted. Ignored when controller switching
    // is enabled (real robot uses separate controllers).
    double      resume_delay_sec = 0.5;

    // Re-IK resume re-times the rebuilt suffix with TOTG. TOTG defaults to 100%
    // of joint velocity/acceleration limits, so the resumed segment runs much
    // faster than the original plan (which was planned at a low scaling factor,
    // e.g. 0.1). Scale TOTG to match the original plan's speed. Applies only to
    // the re-IK path; raw replay keeps the original timing untouched.
    double      resume_velocity_scaling     = 0.1;
    double      resume_acceleration_scaling = 0.1;

    // Real-robot controller switching (Stage 2b). When false (sim, 1 controller):
    // behaviour unchanged — twist streams to the single controller, no switching.
    // When true (real): hand off motion_controller <-> servo_controller at
    // FOLLOW<->AVOID because they cannot be active simultaneously.
    bool        enable_controller_switch = false;
    std::string motion_controller = "scaled_joint_trajectory_controller";  // FOLLOW + resume (action)
    std::string servo_controller  = "forward_position_controller";          // AVOID/RETURN (Servo stream)

    // MoveIt group / frames.
    std::string move_group_name = "ur3_manipulator";
    std::string planning_frame  = "world";
    std::string ee_link         = "tool0";

    // AAPF tuning (forwarded to AAPFGenerator).
    AAPFGenerator::Params aapf;
  };

  // ----- parameter handling -----
  void declareParameters();
  void loadParameters();
  Supervisor::Params buildCoreParams() const;

  // ----- deferred (post-construct) PSM + timer + action-client creation -----
  void deferredInit();

  // ----- timer callbacks -----
  void supervisorTick();
  void twistTick();

  // ----- subscription callback -----
  void trajectoryCb(moveit_msgs::msg::DisplayTrajectory::SharedPtr msg);

  // ----- service callbacks -----
  void startCb(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response>);
  void stopCb(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
              std::shared_ptr<std_srvs::srv::Trigger::Response>);
  void resetCb(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response>);

  // ----- controller handoff -----
  void cancelTrajectoryController();                // hand control to Servo
  void resumeTrajectoryController(const Eigen::Vector3d& ee_now);  // re-send suffix

  // Re-solve the IK of a resume suffix (original joint values per point) using
  // continuation IK seeded from `q_now` (real current joints), then re-time with
  // TOTG. Returns a JTC-ready trajectory whose first point matches q_now, so the
  // controller's start-state tolerance passes. nullopt on solver/IK/TOTG failure.
  std::optional<trajectory_msgs::msg::JointTrajectory>
  reSolveResumeIK(const trajectory_msgs::msg::JointTrajectory& suffix,
                  const std::vector<double>& q_now) const;

  // ----- real-robot controller switch (only when enable_controller_switch) -----
  // Request a switch toward `target`; runs `on_success` once that controller is
  // active (latest request wins; failure -> HALT). Async, non-blocking.
  void requestControllerSwitch(ActiveController target, std::function<void()> on_success);
  void driveControllerSwitch();                     // chained executor: issue next / run pending cb
  void primeServoController();                      // publish current q to servo cmd (anti-jump)
  void onControllerSwitchFailed(const std::string& why);  // fail-safe HALT

  // ----- helpers backed by PSM -----
  Eigen::Vector3d computeEEPosition(
      moveit::core::RobotState& working_state, const std::vector<double>& q) const;
  std::vector<double> readArmJoints(const moveit::core::RobotState& s) const;
  std::vector<Sphere> enumerateSpheres();

  // Cache the latest arm joint velocities q̇ [rad/s], read by name from the
  // JointVelocityEstimator's topic (joint_velocity_topic), in arm_joints order.
  // This is the q̇ the SingularityHandler holds when coasting.
  void jointVelocityCb(sensor_msgs::msg::JointState::SharedPtr msg);
  std::vector<double> latestArmVelocities() const;

  // Condition number κ = σ_max/σ_min of the arm Jacobian, computed exactly as
  // MoveIt Servo does (full 6×N Jacobian, same SVD) so it matches Servo's
  // lower_singularity_threshold. Returns +inf if the group/Jacobian is unavailable.
  double conditionNumber(moveit::core::RobotState& s) const;

  // Publish mode / error / clearance heartbeat.
  void publishStatus(const Supervisor::TickOutput& out);

  // Publish RViz markers for EE / P2 (near) / P1 (far) + the EE→P1 attractor line.
  void publishMarkers(const Supervisor::TickOutput& out, const Eigen::Vector3d& ee_now);

  // React to a mode transition (cancel/resume controller, zero twist, log).
  void onModeTransition(const Supervisor::TickOutput& out, const Eigen::Vector3d& ee_now);

  // ----- members -----
  NodeParams                       node_params_;
  std::unique_ptr<Supervisor>      supervisor_;
  std::unique_ptr<AAPFGenerator>   aapf_;
  std::unique_ptr<SingularityHandler> singularity_;
  const moveit::core::JointModelGroup* jmg_ = nullptr;  // cached arm group (for Jacobian)
  planning_scene_monitor::PlanningSceneMonitorPtr psm_;

  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr follow_client_;

  // Controller-switch plumbing (real mode). Single in-flight switch, chained:
  // latest desired wins; pending side-effect (gate-open / resume) runs on success.
  rclcpp::Client<SwitchController>::SharedPtr switch_client_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr servo_cmd_pub_;  // prime
  std::atomic<bool> servo_controller_active_{true};   // twist gate; always true when switch disabled
  ActiveController  desired_controller_ = ActiveController::SCALED;
  ActiveController  current_controller_ = ActiveController::SCALED;  // startup: scaled active
  bool              switch_inflight_    = false;
  std::function<void()> pending_switch_cb_;

  // ROS plumbing.
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr             mode_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr           mode_text_pub_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr             error_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr           error_text_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr          distance_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr          condition_pub_;    // κ (singularity proximity)
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr            near_sing_pub_;    // κ > kappa_enter
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr          ee_speed_pub_;     // |Δee/Δt| measured EE speed [m/s]
  // Logging debug: [v_att(3), v_rep(3), v_cmd(3), ee_xyz(3)] in command_frame,
  // consumed by the CSV logger (custom_ur_logger). v_* zero outside AAPF_AVOID.
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr log_debug_pub_;
  Eigen::Vector3d last_v_att_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d last_v_rep_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d last_v_cmd_ = Eigen::Vector3d::Zero();
  void publishLogDebug(const Eigen::Vector3d& ee_now);

  // RViz arrows from the EE showing v_att (green) / v_rep (red) / v_cmd (yellow).
  // Length = |v| * velocity_arrow_scale_. Published on ~/velocity_markers.
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vel_marker_pub_;
  double velocity_arrow_scale_ = 0.5;   // [m per m/s] arrow length scale
  void publishVelocityMarkers(const Eigen::Vector3d& ee_now);

  // EE-speed filter: recent (time[s], EE position[m]) samples; speed is the
  // finite difference over a WINDOW (~ee_speed_window_sec) instead of a single
  // 30 Hz step — removes the 0.0/spike aliasing from per-tick differencing.
  std::deque<std::pair<double, Eigen::Vector3d>> ee_hist_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr      joint_pub_;
  rclcpp::Subscription<moveit_msgs::msg::DisplayTrajectory>::SharedPtr traj_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr  joint_vel_sub_;

  // Latest arm joint velocities [rad/s] (arm_joints order) from the estimator,
  // updated by jointVelocityCb, read by twistTick. Guarded — callbacks may run
  // multi-threaded.
  mutable std::mutex  qdot_mtx_;
  std::vector<double> latest_arm_qdot_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
  rclcpp::TimerBase::SharedPtr supervisor_timer_;
  rclcpp::TimerBase::SharedPtr twist_timer_;
  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::TimerBase::SharedPtr resume_timer_;  // one-shot: delayed resume send
};

}  // namespace custom_ur_aapf_2
