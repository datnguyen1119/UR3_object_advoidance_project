#pragma once

#include <Eigen/Core>

namespace custom_ur_aapf_2
{

// Channel through which the AAPF avoidance command is streamed to MoveIt Servo.
//
//   CARTESIAN_TWIST — publish the AAPF linear twist to /delta_twist_cmds. Servo
//                     inverts it (J⁺) and applies its singularity velocity
//                     scaling — i.e. it slows down near a singularity.
//   JOINT_HOLD      — publish a held joint velocity to /delta_joint_cmds. Servo's
//                     joint path SKIPS singularity scaling (servo_calcs.cpp:
//                     jointServoCalcs never calls velocityScalingFactorForSingularity),
//                     so the arm coasts through the singular region at constant
//                     joint speed instead of decelerating.
enum class VelocityChannel
{
  CARTESIAN_TWIST,
  JOINT_HOLD
};

// Decision emitted once per fast tick.
//   channel        — which topic the node should publish on this tick.
//   joint_velocity — the q̇ [rad/s] to command; only meaningful when
//                    channel == JOINT_HOLD (empty/undefined otherwise).
struct ChannelDecision
{
  VelocityChannel channel = VelocityChannel::CARTESIAN_TWIST;
  Eigen::VectorXd joint_velocity;
};

// ---------------------------------------------------------------------------
// SingularityHandler — "trục 2" of the avoidance controller (pure logic, no ROS).
//
// Lives INSIDE the AAPF_AVOID mode. It does NOT change what the robot is doing
// (still avoiding); it only chooses HOW the avoidance velocity reaches Servo,
// based on the Jacobian condition number κ = σ_max/σ_min.
//
// Theory (why this is safe):
//   v = J·q̇  (forward map) is always well-defined, even AT a singularity.
//   q̇ = J⁺·v (inverse) blows up near a singularity — this is what makes Servo
//   decelerate. By COMMITTING to a joint velocity q̇ (captured from the motion
//   just before the singular region) and letting the resulting Cartesian
//   velocity be whatever the forward map produces, we sidestep the inverse
//   entirely and keep moving at constant joint speed. This is "joint-velocity
//   hold / coasting through a singularity" — open-loop for the brief crossing,
//   which is acceptable during avoidance because path precision is not required
//   and RETURN_TO_TRAJECTORY re-converges afterward.
//
// Hysteresis (κ_enter > κ_exit):
//   Enter JOINT_HOLD when κ rises above κ_enter (≈ Servo's
//   lower_singularity_threshold — take over right as Servo would start
//   throttling). Return to CARTESIAN_TWIST only after κ falls back below the
//   lower κ_exit. The gap prevents per-tick chatter when κ hovers near κ_enter.
//
// Safety nets are OUTSIDE this module (already in the supervisor/node):
//   collision → emergency_distance EE-stop; joint over-travel → Servo
//   enforcePositionLimits; never-exits → avoid_timeout; drift → rejoin.
//
// Caveat (documented, handled by the caller): if the captured q̇ is near zero
// at the crossing, the hold would stall. The caller should only feed this a
// representative measured velocity (the arm is in steady avoidance motion at
// κ_enter), or guard the captured magnitude before commanding it.
// ---------------------------------------------------------------------------
class SingularityHandler
{
public:
  struct Params
  {
    double kappa_enter = 25.0;  // κ above this → switch to JOINT_HOLD
    double kappa_exit  = 23.0;  // κ below this → switch back to CARTESIAN_TWIST
  };

  explicit SingularityHandler(Params params);

  // Call every fast tick while in AAPF_AVOID.
  //   kappa         — current condition number, computed the SAME way Servo does
  //                   (σ_max/σ_min of the full 6×N Jacobian) so it matches the
  //                   25/23 thresholds in moveit_servo.yaml.
  //   measured_qdot — current measured joint velocities [rad/s] (e.g. from
  //                   /joint_states); captured once on the CARTESIAN→JOINT_HOLD
  //                   transition and held until the handler exits the region.
  ChannelDecision update(double kappa, const Eigen::VectorXd& measured_qdot);

  // True while coasting in JOINT_HOLD.
  bool coasting() const { return coasting_; }

  // Reset to CARTESIAN_TWIST and drop any held velocity. Call when leaving
  // AAPF_AVOID so a new maneuver starts on the Cartesian channel.
  void reset();

  const Params& params() const { return params_; }

private:
  Params          params_;
  bool            coasting_ = false;
  Eigen::VectorXd held_qdot_;  // velocity captured at the CARTESIAN→JOINT_HOLD edge
};

}  // namespace custom_ur_aapf_2
