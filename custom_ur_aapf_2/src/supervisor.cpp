#include "custom_ur_aapf_2/supervisor.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <utility>

namespace custom_ur_aapf_2
{

Supervisor::Supervisor(Params params) : params_(std::move(params)) {}

// ============================================================================
// Lifecycle
// ============================================================================

void Supervisor::setRunning(bool running)
{
  std::lock_guard<std::mutex> lock(mutex_);
  running_ = running;
  if (!running)
  {
    mode_  = Mode::STOP_OR_REPLAN;
    error_ = ErrorCode::NONE;
  }
}

bool Supervisor::isRunning() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return running_;
}

void Supervisor::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  mode_  = Mode::FOLLOW_TRAJECTORY;
  error_ = ErrorCode::NONE;
}

// ============================================================================
// Trajectory ingestion
// ============================================================================

std::optional<Supervisor::IngestStats> Supervisor::ingestTrajectory(
    const trajectory_msgs::msg::JointTrajectory& jt, const FKFunction& fk)
{
  if (jt.points.empty())
    return std::nullopt;

  trajectory_msgs::msg::JointTrajectory filtered;
  try
  {
    filtered = traj::filter(jt, params_.arm_joints);
  }
  catch (...)
  {
    return std::nullopt;
  }

  // Original EE positions (one per filtered point) for resume mapping.
  std::vector<Eigen::Vector3d> orig_ee;
  orig_ee.reserve(filtered.points.size());
  for (const auto& p : filtered.points)
    orig_ee.push_back(fk(p.positions));

  auto subdiv = traj::subdivideCartesian(filtered, fk, params_.subdivide_step_m);

  IngestStats stats{ jt.points.size(), subdiv.ee_positions.size(),
                     filtered.joint_names.size() };

  {
    std::lock_guard<std::mutex> lock(mutex_);
    cached_traj_           = std::move(subdiv.trajectory);
    cached_ee_positions_   = std::move(subdiv.ee_positions);
    original_traj_         = std::move(filtered);
    original_ee_positions_ = std::move(orig_ee);
    current_waypoint_idx_  = 0;
    // A fresh global plan supersedes any avoidance/stop state.
    if (running_)
    {
      mode_  = Mode::FOLLOW_TRAJECTORY;
      error_ = ErrorCode::NONE;
    }
  }
  return stats;
}

// ============================================================================
// Window scan + index helpers (mutex_ held by caller)
// ============================================================================

double Supervisor::clearanceAt(std::size_t idx, const std::vector<Sphere>& spheres) const
{
  return traj::signedDistanceToSpheres(cached_ee_positions_[idx], spheres);
}

Supervisor::WindowScan Supervisor::scanWindow(const std::vector<Sphere>& spheres,
                                              std::size_t count) const
{
  WindowScan w;
  if (cached_ee_positions_.empty()) return w;

  count = std::max<std::size_t>(1, count);
  w.window_begin = current_waypoint_idx_;
  w.window_end   = std::min(cached_ee_positions_.size(), current_waypoint_idx_ + count + 1);

  bool first_set = false;
  for (std::size_t i = w.window_begin; i < w.window_end; ++i)
  {
    const double c = clearanceAt(i, spheres);
    w.min_clearance = std::min(w.min_clearance, c);
    if (c < params_.blocked_distance_threshold)
    {
      if (!first_set) { w.first_blocked = i; first_set = true; }
      w.last_blocked = i;
      w.any_blocked  = true;
    }
  }
  return w;
}

std::optional<std::size_t> Supervisor::findSafeIndex(
    const std::vector<Sphere>& spheres,
    std::size_t lo, std::size_t hi, std::size_t pref) const
{
  if (lo >= hi) return std::nullopt;
  pref = std::clamp(pref, lo, hi - 1);

  // Expand symmetrically outward from `pref` so we pick the safe index nearest
  // the preferred margin position.
  const std::size_t span = std::max(pref - lo, hi - 1 - pref);
  for (std::size_t d = 0; d <= span; ++d)
  {
    if (pref + d < hi && clearanceAt(pref + d, spheres) >= params_.safe_distance)
      return pref + d;
    if (d > 0 && pref >= lo + d && clearanceAt(pref - d, spheres) >= params_.safe_distance)
      return pref - d;
  }
  return std::nullopt;
}

// ============================================================================
// CHECK_AAPF_CONDITION — resolve P1/P2 and validate the five conditions (§4)
// ============================================================================

Mode Supervisor::runCheck(const TickInput& in, TickOutput& o)
{
  // Use the LONG horizon here: the safe exit (P1) past an obstacle is typically
  // farther than the (short) detection lookahead. This also captures the full
  // extent of the blocked region so last_blocked isn't truncated to the window.
  const WindowScan w = scanWindow(
      in.spheres, static_cast<std::size_t>(std::max(1, params_.p1_search_horizon)));
  o.lookahead_min_clearance = w.min_clearance;

  // Path ahead cleared up while we were deciding → back to FOLLOW.
  if (!w.any_blocked)
    return Mode::FOLLOW_TRAJECTORY;

  // Condition 5 (emergency) is handled before runCheck() in tick().

  // --- P2: a safe waypoint just BEFORE the blocked region (near EE) ---
  const std::size_t p2_pref =
      (w.first_blocked > static_cast<std::size_t>(params_.p2_index_offset))
          ? w.first_blocked - static_cast<std::size_t>(params_.p2_index_offset)
          : current_waypoint_idx_;
  const auto p2 = findSafeIndex(in.spheres, current_waypoint_idx_, w.first_blocked, p2_pref);
  if (!p2)
  {
    error_ = ErrorCode::INVALID_P2;  // condition 3 failed
    return Mode::STOP_OR_REPLAN;
  }

  // --- P1: a safe waypoint just AFTER the blocked region (far) ---
  const std::size_t p1_lo   = w.last_blocked + 1;
  const std::size_t p1_pref = w.last_blocked + 1 + static_cast<std::size_t>(params_.p1_index_offset);
  auto p1 = findSafeIndex(in.spheres, p1_lo, w.window_end, p1_pref);
  if (!p1)
  {
    error_ = ErrorCode::NO_VALID_P1;  // condition 2 failed
    return Mode::STOP_OR_REPLAN;
  }

  // --- Condition 4: P1–P2 separation within local-avoidance limits ---
  double sep = (cached_ee_positions_[*p1] - cached_ee_positions_[*p2]).norm();

  // Too small: try pushing P1 further out so the attractor is meaningful.
  while (sep < params_.min_p1_p2_distance && *p1 + 1 < w.window_end)
  {
    auto p1_next = findSafeIndex(in.spheres, *p1 + 1, w.window_end, *p1 + 1);
    if (!p1_next) break;
    p1  = p1_next;
    sep = (cached_ee_positions_[*p1] - cached_ee_positions_[*p2]).norm();
  }

  o.p1_idx = p1;
  o.p2_idx = p2;
  o.p1_p2_distance = sep;

  if (sep > params_.max_p1_p2_distance)
  {
    error_ = ErrorCode::OBSTACLE_TOO_LARGE;  // condition 4 failed (too big)
    return Mode::STOP_OR_REPLAN;
  }
  if (sep < params_.min_p1_p2_distance)
  {
    // Couldn't separate enough — obstacle is negligible / degenerate. Keep
    // following rather than forcing a tiny avoidance maneuver.
    return Mode::FOLLOW_TRAJECTORY;
  }

  // All conditions satisfied → start avoiding, attractor is P1.
  o.goal = cached_ee_positions_[*p1];
  avoid_start_sec_ = in.now_sec;
  return Mode::AAPF_AVOID;
}

// ============================================================================
// Per-tick decision
// ============================================================================

Supervisor::TickOutput Supervisor::tick(const TickInput& input)
{
  std::lock_guard<std::mutex> lock(mutex_);

  TickOutput o;
  o.previous_mode = mode_;
  o.error         = error_;
  o.cache_size    = cached_ee_positions_.size();
  o.sphere_count  = input.spheres.size();
  o.ee_clearance  = traj::signedDistanceToSpheres(input.ee_now, input.spheres);
  o.lookahead_min_clearance = std::numeric_limits<double>::infinity();
  o.path_deviation = std::numeric_limits<double>::infinity();
  o.p1_p2_distance = 0.0;

  // Cleared unless a normal decision below produces an attractor; the early
  // returns (not-running / stale / emergency) all leave the goal empty.
  last_goal_.reset();

  // Not running, or nothing to follow → hold state.
  if (!running_ || cached_ee_positions_.empty())
  {
    o.mode = mode_;
    o.current_idx = current_waypoint_idx_;
    o.mode_changed = false;
    return o;
  }

  // Stale inputs → stop everywhere except an existing terminal stop.
  if (!input.inputs_fresh)
  {
    if (mode_ != Mode::STOP_OR_REPLAN)
    {
      mode_  = Mode::STOP_OR_REPLAN;
      error_ = ErrorCode::SCENE_STALE;
    }
    o.mode = mode_; o.error = error_;
    o.current_idx = current_waypoint_idx_;
    o.mode_changed = (o.mode != o.previous_mode);
    return o;
  }

  // Incremental nearest-waypoint update.
  const auto window = static_cast<std::size_t>(std::max(1, params_.search_window_steps));
  current_waypoint_idx_ = traj::nearestWaypointIndex(
      cached_ee_positions_, input.ee_now, current_waypoint_idx_, window);
  o.current_idx   = current_waypoint_idx_;
  o.path_deviation = (cached_ee_positions_[current_waypoint_idx_] - input.ee_now).norm();

  // Always-on debug lookahead points: fixed offsets ahead of the EE, clamped to
  // the end of the cache. Independent of the avoidance P1/P2.
  {
    const std::size_t last = cached_ee_positions_.size() - 1;
    const std::size_t i2 = std::min(last,
        current_waypoint_idx_ + static_cast<std::size_t>(std::max(0, params_.vis_p2_offset)));
    const std::size_t i1 = std::min(last,
        current_waypoint_idx_ + static_cast<std::size_t>(std::max(0, params_.vis_p1_offset)));
    o.fixed_p2_pos = cached_ee_positions_[i2];
    o.fixed_p1_pos = cached_ee_positions_[i1];
  }

  // Emergency band — applies in every active state (condition 5 / §10).
  const bool emergency = o.ee_clearance < params_.emergency_distance;
  if (emergency && mode_ != Mode::STOP_OR_REPLAN)
  {
    mode_  = Mode::STOP_OR_REPLAN;
    error_ = ErrorCode::EE_TOO_CLOSE_TO_OBSTACLE;
    o.mode = mode_; o.error = error_;
    o.mode_changed = (o.mode != o.previous_mode);
    return o;
  }

  Mode next = mode_;

  switch (mode_)
  {
    case Mode::FOLLOW_TRAJECTORY:
    {
      // DETECTION uses the short lookahead — react only when a block is near.
      const WindowScan w = scanWindow(
          input.spheres, static_cast<std::size_t>(std::max(1, params_.lookahead_waypoint_count)));
      o.lookahead_min_clearance = w.min_clearance;
      if (w.any_blocked)
        next = Mode::CHECK_AAPF_CONDITION;  // evaluate next tick
      break;
    }

    case Mode::CHECK_AAPF_CONDITION:
    {
      next = runCheck(input, o);
      break;
    }

    case Mode::AAPF_AVOID:
    {
      // Path-deviation guard.
      if (o.path_deviation > params_.max_path_deviation)
      {
        error_ = ErrorCode::P1_UNSTABLE_OR_INVALID;
        next = Mode::STOP_OR_REPLAN;
        break;
      }
      // Timeout (§10: AAPF_TIMEOUT).
      if (input.now_sec - avoid_start_sec_ > params_.avoid_timeout)
      {
        error_ = ErrorCode::AAPF_TIMEOUT;
        next = Mode::STOP_OR_REPLAN;
        break;
      }

      // Near scan (short lookahead) decides when the forward segment is clear.
      const WindowScan w_near = scanWindow(
          input.spheres, static_cast<std::size_t>(std::max(1, params_.lookahead_waypoint_count)));
      // Far scan (long horizon) locates the obstacle region + safe exit (P1).
      const WindowScan w = scanWindow(
          input.spheres, static_cast<std::size_t>(std::max(1, params_.p1_search_horizon)));
      o.lookahead_min_clearance = w_near.min_clearance;

      // Exit when the forward segment is clear AND the EE has clearance (§9).
      if (!w_near.any_blocked && o.ee_clearance >= params_.return_clearance)
      {
        return_start_sec_ = input.now_sec;
        next = Mode::RETURN_TO_TRAJECTORY;
        break;
      }

      // Still avoiding — re-resolve P1 dynamically (obstacle may move, §6).
      std::optional<std::size_t> p1;
      if (w.any_blocked)
      {
        const std::size_t p1_lo = w.last_blocked + 1;
        const std::size_t p1_pref =
            w.last_blocked + 1 + static_cast<std::size_t>(params_.p1_index_offset);
        p1 = findSafeIndex(input.spheres, p1_lo, w.window_end, p1_pref);
      }
      else
      {
        // Region cleared but EE not yet at return_clearance: aim a bit ahead.
        const std::size_t p1_pref = std::min(
            cached_ee_positions_.size() - 1,
            current_waypoint_idx_ + static_cast<std::size_t>(params_.rejoin_waypoint_offset));
        p1 = findSafeIndex(input.spheres, current_waypoint_idx_,
                           cached_ee_positions_.size(), p1_pref);
      }

      if (!p1)
      {
        error_ = ErrorCode::P1_UNSTABLE_OR_INVALID;  // P1 lost mid-avoid (§10)
        next = Mode::STOP_OR_REPLAN;
        break;
      }
      o.p1_idx = p1;
      o.goal   = cached_ee_positions_[*p1];
      break;
    }

    case Mode::RETURN_TO_TRAJECTORY:
    {
      // Rejoin timeout.
      if (input.now_sec - return_start_sec_ > params_.rejoin_timeout)
      {
        error_ = ErrorCode::REJOIN_FAILED;
        next = Mode::STOP_OR_REPLAN;
        break;
      }

      const WindowScan w = scanWindow(
          input.spheres, static_cast<std::size_t>(std::max(1, params_.lookahead_waypoint_count)));
      o.lookahead_min_clearance = w.min_clearance;

      // New obstacle appeared while returning → re-check.
      if (w.any_blocked)
      {
        next = Mode::CHECK_AAPF_CONDITION;
        break;
      }

      // Close enough to the path → rejoin (§9: back to FOLLOW).
      if (o.path_deviation <= params_.return_to_path_distance)
      {
        next = Mode::FOLLOW_TRAJECTORY;
        break;
      }

      // Drive toward a waypoint a little ahead on the path.
      const std::size_t goal_idx = std::min(
          cached_ee_positions_.size() - 1,
          current_waypoint_idx_ + static_cast<std::size_t>(params_.rejoin_waypoint_offset));
      o.goal = cached_ee_positions_[goal_idx];
      break;
    }

    case Mode::STOP_OR_REPLAN:
      break;  // terminal until reset()
  }

  o.mode_changed = (next != mode_);
  mode_ = next;
  o.mode  = mode_;
  o.error = error_;
  last_goal_ = o.goal;  // mirror for the fast twist loop

  // Resolve P1/P2 Cartesian positions (for visualization) from their indices.
  if (o.p1_idx && *o.p1_idx < cached_ee_positions_.size())
    o.p1_pos = cached_ee_positions_[*o.p1_idx];
  if (o.p2_idx && *o.p2_idx < cached_ee_positions_.size())
    o.p2_pos = cached_ee_positions_[*o.p2_idx];

  return o;
}

// ============================================================================
// Resume support
// ============================================================================

std::optional<trajectory_msgs::msg::JointTrajectory>
Supervisor::buildResumeFrom(const Eigen::Vector3d& ee) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (original_traj_.points.empty() || original_ee_positions_.empty())
    return std::nullopt;

  const std::size_t start = traj::nearestWaypointIndexGlobal(original_ee_positions_, ee);
  if (start + 1 >= original_traj_.points.size())
    return std::nullopt;  // already at/past the end

  trajectory_msgs::msg::JointTrajectory out;
  out.header      = original_traj_.header;
  out.joint_names = original_traj_.joint_names;

  // Rebase time_from_start so the suffix starts at t=0. Work directly on the
  // builtin_interfaces Duration (sec/nanosec) to keep the core rclcpp-free.
  auto to_ns = [](const builtin_interfaces::msg::Duration& d) -> std::int64_t {
    return static_cast<std::int64_t>(d.sec) * 1000000000LL + d.nanosec;
  };
  const std::int64_t t0_ns = to_ns(original_traj_.points[start].time_from_start);

  out.points.reserve(original_traj_.points.size() - start);
  for (std::size_t i = start; i < original_traj_.points.size(); ++i)
  {
    auto p = original_traj_.points[i];
    std::int64_t rebased_ns = to_ns(p.time_from_start) - t0_ns;
    if (rebased_ns < 0) rebased_ns = 0;
    p.time_from_start.sec     = static_cast<std::int32_t>(rebased_ns / 1000000000LL);
    p.time_from_start.nanosec = static_cast<std::uint32_t>(rebased_ns % 1000000000LL);
    out.points.push_back(std::move(p));
  }
  return out;
}

// ============================================================================
// Accessors
// ============================================================================

Mode Supervisor::mode() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_;
}

ErrorCode Supervisor::error() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return error_;
}

std::size_t Supervisor::cacheSize() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return cached_ee_positions_.size();
}

std::optional<Eigen::Vector3d> Supervisor::currentGoal() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return last_goal_;
}

std::size_t Supervisor::originalPointCount() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return original_traj_.points.size();
}

double Supervisor::originalDurationSec() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (original_traj_.points.empty())
    return 0.0;
  const auto& d = original_traj_.points.back().time_from_start;
  return d.sec + d.nanosec * 1e-9;
}

}  // namespace custom_ur_aapf_2
