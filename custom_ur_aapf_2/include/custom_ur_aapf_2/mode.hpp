#pragma once

#include <cstdint>
#include <string_view>

namespace custom_ur_aapf_2
{

// =============================================================================
// Supervisor state machine (per the AAPF local-avoidance instruction, §11).
//
//   FOLLOW_TRAJECTORY     — Robot tracks the global trajectory. Supervisor does
//                           NOT command Servo; the joint_trajectory_controller
//                           is the motion authority. Each tick scans the N
//                           waypoints ahead of the EE for obstacles.
//
//   CHECK_AAPF_CONDITION  — A forward waypoint is obstacle-affected. Resolve
//                           P2 (near, safety confirm) and P1 (far, attractor),
//                           and validate the five activation conditions (§4).
//
//   AAPF_AVOID            — Conditions met. The global trajectory goal is
//                           cancelled and the supervisor streams a Cartesian
//                           twist (v_att toward P1 + v_rep off the obstacle)
//                           into Servo. P1 is re-resolved every tick.
//
//   RETURN_TO_TRAJECTORY  — Path ahead is clear and the EE has clearance. Drive
//                           back onto the path toward a rejoin waypoint; once
//                           on the path, hand control back to FOLLOW.
//
//   STOP_OR_REPLAN        — No valid P1/P2, obstacle too large, EE too close,
//                           AAPF timeout, or rejoin failed. Robot is halted and
//                           an error is reported (replan = report-only here).
//                           Terminal until the operator calls ~/reset.
// =============================================================================
enum class Mode : std::int8_t
{
  FOLLOW_TRAJECTORY    = 0,
  CHECK_AAPF_CONDITION = 1,
  AAPF_AVOID           = 2,
  RETURN_TO_TRAJECTORY = 3,
  STOP_OR_REPLAN       = 4,
};

inline std::string_view toString(Mode m)
{
  switch (m)
  {
    case Mode::FOLLOW_TRAJECTORY:    return "FOLLOW_TRAJECTORY";
    case Mode::CHECK_AAPF_CONDITION: return "CHECK_AAPF_CONDITION";
    case Mode::AAPF_AVOID:           return "AAPF_AVOID";
    case Mode::RETURN_TO_TRAJECTORY: return "RETURN_TO_TRAJECTORY";
    case Mode::STOP_OR_REPLAN:       return "STOP_OR_REPLAN";
  }
  return "UNKNOWN";
}

// =============================================================================
// Error codes reported when the supervisor enters STOP_OR_REPLAN (§10).
// =============================================================================
enum class ErrorCode : std::int8_t
{
  NONE                     = 0,
  NO_VALID_P1              = 1,  // no safe waypoint found past the obstacle region
  INVALID_P2               = 2,  // near waypoint is itself inside obstacle / danger
  OBSTACLE_TOO_LARGE       = 3,  // P1–P2 span exceeds local-avoidance limit
  EE_TOO_CLOSE_TO_OBSTACLE = 4,  // EE inside emergency band — hard stop
  AAPF_TIMEOUT             = 5,  // avoidance ran longer than avoid_timeout
  P1_UNSTABLE_OR_INVALID   = 6,  // P1 jumped / path deviation exceeded limit
  REJOIN_FAILED            = 7,  // could not get back onto the trajectory
  SCENE_STALE              = 8,  // planning-scene inputs went stale
};

inline std::string_view toString(ErrorCode e)
{
  switch (e)
  {
    case ErrorCode::NONE:                     return "NONE";
    case ErrorCode::NO_VALID_P1:              return "NO_VALID_P1";
    case ErrorCode::INVALID_P2:               return "INVALID_P2";
    case ErrorCode::OBSTACLE_TOO_LARGE:       return "OBSTACLE_TOO_LARGE";
    case ErrorCode::EE_TOO_CLOSE_TO_OBSTACLE: return "EE_TOO_CLOSE_TO_OBSTACLE";
    case ErrorCode::AAPF_TIMEOUT:             return "AAPF_TIMEOUT";
    case ErrorCode::P1_UNSTABLE_OR_INVALID:   return "P1_UNSTABLE_OR_INVALID";
    case ErrorCode::REJOIN_FAILED:            return "REJOIN_FAILED";
    case ErrorCode::SCENE_STALE:              return "SCENE_STALE";
  }
  return "UNKNOWN";
}

}  // namespace custom_ur_aapf_2
