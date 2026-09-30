#pragma once

#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/time.hpp>

#include "custom_ur_aapf_2/geometry_types.hpp"

namespace custom_ur_aapf_2
{

// AAPF velocity generator (§5–§8).
//
//   v_att = k_att * (P1 - EE)                       (attractive, toward goal)
//   v_rep = Σ_i  k_rep * (1/d_i - 1/ρ0) * (1/d_i²) * n̂_i    (repulsive)
//           for every sphere whose surface distance d_i < ρ0 (influence_distance)
//           n̂_i = unit vector from the sphere surface toward the EE
//           each term's magnitude saturated to repulsive_saturation
//   v_cmd = v_att + v_rep,   then  v_min ≤ |v_cmd| ≤ v_max
//
// Output is a pure-linear Cartesian TwistStamped in command_frame (angular = 0;
// orientation is left to Servo). Stateless across calls.
class AAPFGenerator
{
public:
  struct Params
  {
    double k_att                = 1.5;   // attractive gain
    double k_rep                = 0.05;  // repulsive gain
    double influence_distance   = 0.30;  // ρ0 — repulsion influence radius [m]
    double v_max                = 0.20;  // [m/s] output speed cap (total v_cmd)
    double v_min                = 0.02;  // [m/s] floor (avoid vanishingly small cmds)
    double repulsive_saturation = 0.50;  // [m/s] cap on EACH repulsive term (per sphere)
    // (B2) Cap on the SUMMED repulsion BEFORE adding attraction. Keeping this
    // below v_max guarantees v_att always contributes to v_cmd (so the EE keeps
    // being pulled toward P1 instead of being shoved purely away). 0 → disabled.
    double v_rep_max            = 0.25;
    std::string command_frame   = "world";
  };

  explicit AAPFGenerator(Params params);

  // Velocity decomposition for logging/debug. All in command_frame [m/s].
  //   v_att — attractive component (pre-saturation)
  //   v_rep — repulsive component (pre-saturation)
  //   v_cmd — final commanded velocity (post v_min/v_max saturation) = twist out
  struct Decomposition
  {
    Eigen::Vector3d v_att = Eigen::Vector3d::Zero();
    Eigen::Vector3d v_rep = Eigen::Vector3d::Zero();
    Eigen::Vector3d v_cmd = Eigen::Vector3d::Zero();
  };

  // ee_pos / goal_pos / sphere centers must all be expressed in command_frame.
  // If `dbg` is non-null it is filled with the att/rep/cmd decomposition.
  geometry_msgs::msg::TwistStamped computeTwist(
      const Eigen::Vector3d&       ee_pos,
      const Eigen::Vector3d&       goal_pos,
      const std::vector<Sphere>&   spheres,
      const rclcpp::Time&          stamp,
      Decomposition*               dbg = nullptr) const;

  // Zero twist in command_frame (used to halt Servo cleanly).
  geometry_msgs::msg::TwistStamped zeroTwist(const rclcpp::Time& stamp) const;

  const Params& params() const { return params_; }

private:
  Params params_;
};

}  // namespace custom_ur_aapf_2
