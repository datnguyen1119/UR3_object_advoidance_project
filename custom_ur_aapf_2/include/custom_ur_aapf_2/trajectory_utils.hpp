#pragma once

// Header-only trajectory helpers used by the Supervisor.
//
// Design:
//  - Lookup-by-name (no positional indexing, no MoveIt-specific containers).
//  - FK is provided as a std::function so the caller can plug in any backend
//    (MoveIt RobotState, KDL, hand-rolled). The header itself does not depend
//    on MoveIt.
//  - Subdivide one-pass: each output segment is approximately <= max_step in
//    Cartesian space (linear-in-joint-space interpolation, so the bound holds
//    when joints don't induce large nonlinear curvature within one segment).

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "custom_ur_aapf_2/geometry_types.hpp"

namespace custom_ur_aapf_2::traj
{

// FK signature: joint positions (in some agreed order) → EE position in some
// fixed frame. Frame is the caller's responsibility — same FK must be used
// throughout one Supervisor cycle.
using FKFunction = std::function<Eigen::Vector3d(const std::vector<double>&)>;

// ---------------------------------------------------------------------------
// Find indices of desired joints in src_names. Throws if any are missing.
// ---------------------------------------------------------------------------
inline std::vector<std::size_t> findJointIndices(
    const std::vector<std::string>& src_names,
    const std::vector<std::string>& desired_names)
{
  std::vector<std::size_t> idx;
  idx.reserve(desired_names.size());
  for (const auto& name : desired_names)
  {
    const auto it = std::find(src_names.begin(), src_names.end(), name);
    if (it == src_names.end())
      throw std::runtime_error("Joint '" + name + "' not in trajectory");
    idx.push_back(std::distance(src_names.begin(), it));
  }
  return idx;
}

// ---------------------------------------------------------------------------
// Filter a JointTrajectory to contain only `desired_names`, in given order.
// Preserves empty per-point fields (don't fabricate zeros for velocity/effort
// when the source didn't provide them).
// ---------------------------------------------------------------------------
inline trajectory_msgs::msg::JointTrajectory filter(
    const trajectory_msgs::msg::JointTrajectory& src,
    const std::vector<std::string>& desired_names)
{
  const auto idx = findJointIndices(src.joint_names, desired_names);

  trajectory_msgs::msg::JointTrajectory out;
  out.header      = src.header;
  out.joint_names = desired_names;
  out.points.reserve(src.points.size());

  auto take = [&](const auto& src_arr, auto& dst_arr) {
    if (src_arr.empty()) return;
    dst_arr.resize(idx.size());
    for (std::size_t k = 0; k < idx.size(); ++k) dst_arr[k] = src_arr[idx[k]];
  };

  for (const auto& p_in : src.points)
  {
    trajectory_msgs::msg::JointTrajectoryPoint p_out;
    p_out.time_from_start = p_in.time_from_start;
    take(p_in.positions,     p_out.positions);
    take(p_in.velocities,    p_out.velocities);
    take(p_in.accelerations, p_out.accelerations);
    take(p_in.effort,        p_out.effort);
    out.points.push_back(std::move(p_out));
  }
  return out;
}

// ---------------------------------------------------------------------------
// Result of subdividing a trajectory: dense waypoints + pre-computed EE
// positions (no need to FK twice elsewhere).
// ---------------------------------------------------------------------------
struct CartesianSubdivision
{
  trajectory_msgs::msg::JointTrajectory trajectory;
  std::vector<Eigen::Vector3d> ee_positions;
};

// ---------------------------------------------------------------------------
// Subdivide `src` so that consecutive output waypoints are approximately
// `max_step` apart in Cartesian space (measured at the EE).
// (See custom_ur_aapf for the original derivation; velocities/accels dropped.)
// ---------------------------------------------------------------------------
inline CartesianSubdivision subdivideCartesian(
    const trajectory_msgs::msg::JointTrajectory& src,
    const FKFunction& fk,
    double max_step)
{
  CartesianSubdivision out;
  out.trajectory.header      = src.header;
  out.trajectory.joint_names = src.joint_names;

  if (src.points.empty() || max_step <= 0.0)
    return out;

  out.trajectory.points.push_back(src.points.front());
  out.ee_positions.push_back(fk(src.points.front().positions));

  for (std::size_t i = 1; i < src.points.size(); ++i)
  {
    const auto& p_prev = src.points[i - 1];
    const auto& p_curr = src.points[i];

    const Eigen::Vector3d ee_prev = out.ee_positions.back();
    const Eigen::Vector3d ee_curr = fk(p_curr.positions);
    const double dist = (ee_curr - ee_prev).norm();

    const std::size_t n_segments = std::max<std::size_t>(
        1, static_cast<std::size_t>(std::ceil(dist / max_step)));

    for (std::size_t k = 1; k < n_segments; ++k)
    {
      const double t = static_cast<double>(k) / static_cast<double>(n_segments);

      trajectory_msgs::msg::JointTrajectoryPoint p_mid;
      p_mid.positions.resize(p_curr.positions.size());
      for (std::size_t j = 0; j < p_curr.positions.size(); ++j)
        p_mid.positions[j] = (1.0 - t) * p_prev.positions[j] + t * p_curr.positions[j];

      const Eigen::Vector3d ee_mid = fk(p_mid.positions);
      out.trajectory.points.push_back(std::move(p_mid));
      out.ee_positions.push_back(ee_mid);
    }

    out.trajectory.points.push_back(p_curr);
    out.ee_positions.push_back(ee_curr);
  }

  return out;
}

// ---------------------------------------------------------------------------
// Incremental nearest-waypoint search in the forward window [hint, hint+window).
// O(window) per tick; robot doesn't travel backward along the path.
// ---------------------------------------------------------------------------
inline std::size_t nearestWaypointIndex(
    const std::vector<Eigen::Vector3d>& cache,
    const Eigen::Vector3d& point,
    std::size_t hint,
    std::size_t window_size)
{
  if (cache.empty()) return 0;
  const std::size_t end = std::min(cache.size(), hint + std::max<std::size_t>(1, window_size));
  std::size_t best = std::min(hint, cache.size() - 1);
  double best_d2 = (cache[best] - point).squaredNorm();
  for (std::size_t i = hint + 1; i < end; ++i)
  {
    const double d2 = (cache[i] - point).squaredNorm();
    if (d2 < best_d2)
    {
      best_d2 = d2;
      best = i;
    }
  }
  return best;
}

// ---------------------------------------------------------------------------
// Global nearest-waypoint search over the whole cache (used for trajectory
// resume, where the EE may be anywhere relative to the original path).
// ---------------------------------------------------------------------------
inline std::size_t nearestWaypointIndexGlobal(
    const std::vector<Eigen::Vector3d>& cache,
    const Eigen::Vector3d& point)
{
  if (cache.empty()) return 0;
  std::size_t best = 0;
  double best_d2 = (cache[0] - point).squaredNorm();
  for (std::size_t i = 1; i < cache.size(); ++i)
  {
    const double d2 = (cache[i] - point).squaredNorm();
    if (d2 < best_d2) { best_d2 = d2; best = i; }
  }
  return best;
}

// ---------------------------------------------------------------------------
// Minimum signed distance from a point to the surface of any sphere.
// Returns +inf when there are no spheres (treated as "infinitely clear").
// ---------------------------------------------------------------------------
inline double signedDistanceToSpheres(
    const Eigen::Vector3d& p, const std::vector<Sphere>& spheres)
{
  double best = std::numeric_limits<double>::infinity();
  for (const auto& s : spheres)
    best = std::min(best, (p - s.center).norm() - s.radius);
  return best;
}

}  // namespace custom_ur_aapf_2::traj
