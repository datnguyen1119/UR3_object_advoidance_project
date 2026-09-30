#pragma once

#include <Eigen/Geometry>

namespace custom_ur_aapf_2
{

// A single spherical obstacle in the command frame.
//   center — sphere center position
//   radius — sphere radius [m]
// Signed surface distance from a point p is: (p - center).norm() - radius.
struct Sphere
{
  Eigen::Vector3d center;
  double          radius;
};

}  // namespace custom_ur_aapf_2
