#include "custom_ur_aapf_2/singularity_handler.hpp"

namespace custom_ur_aapf_2
{

SingularityHandler::SingularityHandler(Params params) : params_(params) {}

ChannelDecision SingularityHandler::update(double kappa, const Eigen::VectorXd& measured_qdot)
{
  if (!coasting_)
  {
    // CARTESIAN_TWIST → JOINT_HOLD: κ has risen into the singular region. Capture
    // the velocity the arm is moving at RIGHT NOW (the clean avoidance velocity,
    // before Servo would throttle it) and hold it through the crossing.
    if (kappa > params_.kappa_enter)
    {
      coasting_  = true;
      held_qdot_ = measured_qdot;
      return {VelocityChannel::JOINT_HOLD, held_qdot_};
    }
    return {VelocityChannel::CARTESIAN_TWIST, {}};
  }

  // Currently coasting. Stay in JOINT_HOLD until κ drops back below the LOWER
  // exit threshold (hysteresis), then hand back to the Cartesian channel.
  if (kappa < params_.kappa_exit)
  {
    coasting_ = false;
    return {VelocityChannel::CARTESIAN_TWIST, {}};
  }
  return {VelocityChannel::JOINT_HOLD, held_qdot_};
}

void SingularityHandler::reset()
{
  coasting_ = false;
  held_qdot_ = Eigen::VectorXd();
}

}  // namespace custom_ur_aapf_2
