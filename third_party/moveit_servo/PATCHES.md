# Local patches on top of upstream `moveit_servo`

This is a **forked copy** of `moveit_servo` (from `moveit2`). Local edits are
listed here so they can be re-applied when syncing with upstream.

## Patch 1 — Adaptive Damped Least Squares (DLS) pseudo-inverse

**File**: `src/servo_calcs.cpp`
**Function**: `ServoCalcs::cartesianServoCalcs`
**Location**: right after the SVD of the Jacobian.

### What changed

Replaced the plain Moore–Penrose pseudo-inverse with an **adaptive Damped Least
Squares** pseudo-inverse. Damping `λ²` grows as the smallest singular value
approaches a threshold `epsilon`, preventing joint-velocity blow-up near
kinematic singularities.

```cpp
// === Damped Least Squares (DLS) — adaptive damping ===
const Eigen::VectorXd& sigmas = svd.singularValues();
const double sigma_min = sigmas(sigmas.size() - 1);

const double epsilon   = 0.05;   // threshold below which damping kicks in
const double lambda_max = 0.1;   // maximum damping
double lambda_sq = 0.0;

if (sigma_min < epsilon) {
  const double ratio = sigma_min / epsilon;
  lambda_sq = (1.0 - ratio * ratio) * lambda_max * lambda_max;
}

Eigen::VectorXd sigma_damped(sigmas.size());
for (int i = 0; i < sigmas.size(); ++i) {
  sigma_damped(i) = sigmas(i) / (sigmas(i) * sigmas(i) + lambda_sq);
}

Eigen::MatrixXd pseudo_inverse =
    svd.matrixV() * sigma_damped.asDiagonal() * svd.matrixU().transpose();
```

### Why

Upstream `moveit_servo` only scales the resulting joint velocities when close
to a singularity. That still allows the pseudo-inverse itself to produce huge
joint commands which then get scaled — numerically unstable and produces
jerky motion. DLS smooths motion through near-singular configurations.

### Long-term plan

This patch lives directly in upstream source — which means upstream
updates need manual conflict resolution. Possible cleaner approaches:

1. **Fork upstream into a proper git repo** and pin it via a `.repos` file —
   easier rebase, history preserved.
2. **Extract DLS into a plugin or wrapper node**, so the upstream source
   stays untouched. Best long-term.

Resolve before upgrading ROS distro or pulling a new `moveit2` release.

## Notes on the inner `moveit_servo/` directory

This folder contains a nested copy of the original upstream source at
`moveit_servo/moveit_servo/`. It is kept for reference / diffing against
upstream. **Both copies declare `<name>moveit_servo</name>` in their
`package.xml`**, which is a colcon naming conflict — colcon picks one to
build (currently the outer, customized copy). If you ever delete the inner
copy, no behavior changes; if you want a clean tree without the colcon
warning, delete `moveit_servo/moveit_servo/`.
