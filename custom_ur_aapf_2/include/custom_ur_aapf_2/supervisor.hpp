#pragma once

// Supervisor — pure business logic for the AAPF local-avoidance supervisor (v2).
//
// Owns:
//   - the 5-state machine (FOLLOW_TRAJECTORY / CHECK_AAPF_CONDITION /
//     AAPF_AVOID / RETURN_TO_TRAJECTORY / STOP_OR_REPLAN)
//   - the cached subdivided trajectory + per-waypoint EE positions, plus the
//     original (time-parameterized) trajectory for resume-on-rejoin
//   - the incremental nearest-waypoint search, the N-waypoint lookahead scan,
//     and the P1 / P2 resolution + activation-condition logic
//
// It does NOT depend on rclcpp / MoveIt at runtime. The ROS layer computes EE
// position, enumerates spheres, and feeds them in as plain data each tick, plus
// a monotonically increasing time stamp (seconds) used for the timeouts.

#include <cstddef>
#include <limits>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "custom_ur_aapf_2/geometry_types.hpp"
#include "custom_ur_aapf_2/mode.hpp"
#include "custom_ur_aapf_2/trajectory_utils.hpp"

namespace custom_ur_aapf_2
{

class Supervisor
{
public:
  struct Params
  {
    // ---- Obstacle-distance bands [m] (§12: obstacle distances) ----
    double safe_distance              = 0.15;  // waypoint/EE considered safe at/above this
    double blocked_distance_threshold = 0.12;  // waypoint counted as obstacle-affected below this
    double emergency_distance         = 0.05;  // EE below this → immediate STOP
    double return_clearance           = 0.20;  // EE clearance needed before leaving AAPF_AVOID

    // ---- Waypoint / lookahead (dense-cache index units; physical = idx*step) ----
    int    lookahead_waypoint_count = 40;   // DETECTION range: how far ahead to look for a block
    int    p1_search_horizon        = 200;  // P1 SEARCH range: how far ahead the safe exit (P1) may
                                            // be found. Must exceed lookahead — the exit past an
                                            // obstacle is usually farther than the detection range.
    int    p1_index_offset          = 8;    // P1 margin placed past the blocked region
    int    p2_index_offset          = 12;   // P2 margin placed before the blocked region
    double max_p1_p2_distance       = 0.60; // [m] above this → OBSTACLE_TOO_LARGE
    double min_p1_p2_distance       = 0.04; // [m] minimum P1–P2 separation to act on

    // ---- Return to trajectory ----
    double return_to_path_distance = 0.05;  // [m] EE within this of path → rejoin to FOLLOW
    int    rejoin_waypoint_offset  = 20;    // attractor waypoint offset while returning

    // ---- Always-on lookahead points (visualization/debug only) ----
    // Two waypoints a fixed number of dense steps ahead of the EE, published
    // continuously regardless of state. They do NOT drive avoidance — the real
    // P1/P2 used for AAPF are resolved relative to the obstacle region.
    int vis_p2_offset = 10;   // near debug lookahead (≈10 cm)
    int vis_p1_offset = 25;   // far  debug lookahead (≈25 cm)

    // ---- Cartesian subdivision + nearest search ----
    double subdivide_step_m    = 0.01;  // [m] target dense segment length
    int    search_window_steps = 40;    // forward window for incremental nearest search

    // ---- Timing [s] (§12: time/frequency) ----
    double avoid_timeout  = 12.0;  // max time in AAPF_AVOID before STOP (AAPF_TIMEOUT)
    double rejoin_timeout = 8.0;   // max time in RETURN_TO_TRAJECTORY before REJOIN_FAILED

    // ---- Safety (§12: safety) ----
    double max_path_deviation = 0.50;  // [m] EE-to-path beyond this → STOP (P1_UNSTABLE)

    // Arm joints, in the order the FK function expects.
    std::vector<std::string> arm_joints = {
      "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
      "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
    };
  };

  using FKFunction = traj::FKFunction;

  explicit Supervisor(Params params);

  // ----- Lifecycle -----
  void setRunning(bool running);
  bool isRunning() const;
  void reset();  // clear STOP/error → back to FOLLOW_TRAJECTORY

  // ----- Trajectory ingestion -----
  struct IngestStats {
    std::size_t raw;
    std::size_t subdivided;
    std::size_t kept_joints;
  };
  std::optional<IngestStats> ingestTrajectory(
      const trajectory_msgs::msg::JointTrajectory& jt, const FKFunction& fk);

  // ----- Per-tick decision -----
  struct TickInput
  {
    Eigen::Vector3d     ee_now;
    std::vector<Sphere> spheres;
    bool                inputs_fresh = true;
    double              now_sec      = 0.0;  // monotonic seconds for timeouts
  };

  struct TickOutput
  {
    Mode        mode;
    Mode        previous_mode;
    bool        mode_changed;
    ErrorCode   error;

    // Cartesian attractor the twist generator should pull toward (P1 while
    // avoiding, the rejoin waypoint while returning). nullopt → publish zero.
    std::optional<Eigen::Vector3d> goal;

    // Diagnostics.
    double      ee_clearance;            // signed EE→nearest-sphere distance
    double      lookahead_min_clearance; // min signed clearance over the window
    double      path_deviation;          // EE→nearest cached waypoint distance
    double      p1_p2_distance;
    std::optional<std::size_t> p1_idx;
    std::optional<std::size_t> p2_idx;
    // Cartesian positions of P1/P2 (resolved from the dense cache) for
    // visualization. Present whenever the matching index is set.
    std::optional<Eigen::Vector3d> p1_pos;
    std::optional<Eigen::Vector3d> p2_pos;
    // Always-on debug lookahead points (fixed offsets ahead of EE). Present
    // whenever a trajectory is cached and the supervisor is running.
    std::optional<Eigen::Vector3d> fixed_p1_pos;
    std::optional<Eigen::Vector3d> fixed_p2_pos;
    std::size_t current_idx;
    std::size_t cache_size;
    std::size_t sphere_count;
  };

  TickOutput tick(const TickInput& input);

  // ----- Resume support -----
  // Build a FollowJointTrajectory-ready suffix of the ORIGINAL (time-stamped)
  // trajectory, starting at the original waypoint nearest `ee`, with
  // time_from_start rebased to zero. nullopt if no original trajectory cached.
  std::optional<trajectory_msgs::msg::JointTrajectory>
  buildResumeFrom(const Eigen::Vector3d& ee) const;

  // ----- Accessors -----
  Mode          mode()    const;
  ErrorCode     error()   const;
  std::size_t   cacheSize() const;
  // Resume diagnostics: total points and total duration [s] of the ORIGINAL
  // (time-parameterized) trajectory currently cached. Lets the ROS layer tell
  // "resumed from where we left off" apart from "jumped near the goal".
  std::size_t   originalPointCount() const;
  double        originalDurationSec() const;
  // Latest Cartesian attractor decided by tick() (P1 while avoiding, the rejoin
  // waypoint while returning). nullopt outside the avoid/return states. Read by
  // the fast twist loop so it doesn't have to re-run the decision logic.
  std::optional<Eigen::Vector3d> currentGoal() const;
  const Params& params()  const { return params_; }

private:
  // Resolve the obstacle-affected region in the forward window and the P1/P2
  // pair. Requires mutex_ held. Fills out the diagnostic fields on `o`.
  // Returns the next mode and sets o.error when STOP is warranted.
  Mode runCheck(const TickInput& in, TickOutput& o);

  // Window scan helpers (require mutex_ held).
  struct WindowScan
  {
    std::size_t window_begin = 0;
    std::size_t window_end   = 0;   // exclusive
    bool        any_blocked  = false;
    std::size_t first_blocked = 0;
    std::size_t last_blocked  = 0;
    double      min_clearance = std::numeric_limits<double>::infinity();
  };
  // Scan `count` dense waypoints ahead of the EE for the obstacle-affected
  // region and the minimum clearance. Used with the short lookahead for
  // detection and with the long horizon for P1 search.
  WindowScan scanWindow(const std::vector<Sphere>& spheres, std::size_t count) const;

  // Search [lo, hi) for the index closest to `pref` whose clearance ≥ safe.
  std::optional<std::size_t> findSafeIndex(
      const std::vector<Sphere>& spheres,
      std::size_t lo, std::size_t hi, std::size_t pref) const;

  double clearanceAt(std::size_t idx, const std::vector<Sphere>& spheres) const;

  Params params_;
  mutable std::mutex mutex_;

  Mode      mode_  = Mode::FOLLOW_TRAJECTORY;
  ErrorCode error_ = ErrorCode::NONE;
  bool      running_ = false;

  // Dense (subdivided) path for avoidance reasoning.
  trajectory_msgs::msg::JointTrajectory cached_traj_;
  std::vector<Eigen::Vector3d>          cached_ee_positions_;
  // Original time-parameterized path (for resume).
  trajectory_msgs::msg::JointTrajectory original_traj_;
  std::vector<Eigen::Vector3d>          original_ee_positions_;

  std::size_t current_waypoint_idx_ = 0;
  double      avoid_start_sec_  = 0.0;
  double      return_start_sec_ = 0.0;
  std::optional<Eigen::Vector3d> last_goal_;  // mirrors the most recent o.goal
};

}  // namespace custom_ur_aapf_2
