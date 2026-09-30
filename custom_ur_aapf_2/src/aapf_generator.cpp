// AAPF generator — attractive + adaptive repulsive Cartesian velocity (§5–§8).

#include "custom_ur_aapf_2/aapf_generator.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace custom_ur_aapf_2
{

AAPFGenerator::AAPFGenerator(Params params) : params_(std::move(params)) {}

geometry_msgs::msg::TwistStamped AAPFGenerator::zeroTwist(const rclcpp::Time& stamp) const
{
  geometry_msgs::msg::TwistStamped out;
  out.header.stamp = stamp;
  out.header.frame_id = params_.command_frame;
  return out;  // all fields default-zero
}

geometry_msgs::msg::TwistStamped AAPFGenerator::computeTwist(
    const Eigen::Vector3d&     ee_pos,
    const Eigen::Vector3d&     goal_pos,
    const std::vector<Sphere>& spheres,
    const rclcpp::Time&        stamp,
    Decomposition*             dbg) const
{
  // ----- Attractive: pull the EE toward P1 (§6) -----
  Eigen::Vector3d v_att = params_.k_att * (goal_pos - ee_pos);

  // ----- Repulsive: push the EE off each nearby obstacle surface (§7) -----
  Eigen::Vector3d v_rep = Eigen::Vector3d::Zero();
  const double rho0 = params_.influence_distance;

  for (const auto& s : spheres)
  {
    const Eigen::Vector3d to_ee = ee_pos - s.center;
    const double center_dist = to_ee.norm();
    // Signed distance from the EE to the sphere surface.
    double d = center_dist - s.radius;

    if (d >= rho0)
      continue;  // outside influence radius → no repulsion (§7: v_rep = 0)

    // Outward surface normal (toward the EE). Degenerate at the center → skip.
    if (center_dist < 1e-9)
      continue;
    const Eigen::Vector3d n_hat = to_ee / center_dist;

    // Clamp d to a small positive floor so the 1/d² term stays finite; the
    // EE-too-close emergency stop in the Supervisor is the real safeguard.
    d = std::max(d, 1e-3);

    // Classic FIRAS-style gradient magnitude.
    double mag = params_.k_rep * (1.0 / d - 1.0 / rho0) * (1.0 / (d * d));
    mag = std::clamp(mag, 0.0, params_.repulsive_saturation);

    v_rep += mag * n_hat;
  }

  // (B2) Cap the summed repulsion on its OWN budget BEFORE adding attraction.
  // Without this, when many/close spheres drive |v_rep| past v_max, the final
  // uniform down-scaling of (v_att+v_rep) collapses the direction onto pure
  // repulsion and the attractive pull toward P1 vanishes. Limiting v_rep first
  // keeps v_att a meaningful fraction of the command.
  if (params_.v_rep_max > 0.0)
  {
    const double rep_speed = v_rep.norm();
    if (rep_speed > params_.v_rep_max)
      v_rep *= params_.v_rep_max / rep_speed;
  }

  // ----- Combine + saturate (§8) -----
  Eigen::Vector3d v = v_att + v_rep;
  double speed = v.norm();

  if (speed > params_.v_max && speed > 0.0)
  {
    v *= params_.v_max / speed;
    speed = params_.v_max;
  }
  else if (speed > 1e-6 && speed < params_.v_min)
  {
    v *= params_.v_min / speed;  // floor tiny commands so the EE keeps moving
  }

  if (dbg)
  {
    dbg->v_att = v_att;   // pre-saturation attractive
    dbg->v_rep = v_rep;   // pre-saturation repulsive
    dbg->v_cmd = v;       // post-saturation commanded (matches twist below)
  }

  geometry_msgs::msg::TwistStamped out;
  out.header.stamp = stamp;
  out.header.frame_id = params_.command_frame;
  out.twist.linear.x = v.x();
  out.twist.linear.y = v.y();
  out.twist.linear.z = v.z();
  // Angular left at zero — orientation control is delegated to Servo.
  return out;
}

}  // namespace custom_ur_aapf_2
