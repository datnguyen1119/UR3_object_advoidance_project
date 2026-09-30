#include <gtest/gtest.h>

#include <Eigen/Core>

#include "custom_ur_aapf_2/singularity_handler.hpp"

using custom_ur_aapf_2::SingularityHandler;
using custom_ur_aapf_2::VelocityChannel;

namespace
{
// Default 17/14 thresholds, matching aapf_params_real.yaml + Servo's
// lower_singularity_threshold.
SingularityHandler makeHandler()
{
  return SingularityHandler({/*kappa_enter=*/17.0, /*kappa_exit=*/14.0});
}

Eigen::VectorXd qdot(double v)
{
  Eigen::VectorXd q(6);
  q.setConstant(v);
  return q;
}
}  // namespace

// Well-conditioned: stay on the Cartesian channel, never coast.
TEST(SingularityHandler, StaysCartesianBelowEnter)
{
  auto h = makeHandler();
  auto d = h.update(/*kappa=*/5.0, qdot(0.1));
  EXPECT_EQ(d.channel, VelocityChannel::CARTESIAN_TWIST);
  EXPECT_FALSE(h.coasting());
}

// Crossing κ_enter switches to JOINT_HOLD and captures the current velocity.
TEST(SingularityHandler, EntersJointHoldAboveEnter)
{
  auto h = makeHandler();
  ASSERT_EQ(h.update(10.0, qdot(0.10)).channel, VelocityChannel::CARTESIAN_TWIST);

  auto d = h.update(/*kappa=*/18.0, qdot(0.10));
  EXPECT_EQ(d.channel, VelocityChannel::JOINT_HOLD);
  EXPECT_TRUE(h.coasting());
  ASSERT_EQ(d.joint_velocity.size(), 6);
  EXPECT_DOUBLE_EQ(d.joint_velocity[0], 0.10);
}

// The held velocity is FROZEN at the entry value, not refreshed each tick.
TEST(SingularityHandler, HoldsCapturedVelocityNotLatest)
{
  auto h = makeHandler();
  h.update(18.0, qdot(0.10));               // capture 0.10
  auto d = h.update(20.0, qdot(0.99));      // later measured velocity must be ignored
  EXPECT_EQ(d.channel, VelocityChannel::JOINT_HOLD);
  EXPECT_DOUBLE_EQ(d.joint_velocity[0], 0.10);
}

// Hysteresis: between κ_exit and κ_enter while coasting → keep coasting.
TEST(SingularityHandler, StaysCoastingInHysteresisBand)
{
  auto h = makeHandler();
  h.update(18.0, qdot(0.10));               // enter coast
  auto d = h.update(15.0, qdot(0.10));      // 14 < 15 < 17 → still coasting
  EXPECT_EQ(d.channel, VelocityChannel::JOINT_HOLD);
  EXPECT_TRUE(h.coasting());
}

// Falling below κ_exit hands control back to the Cartesian channel.
TEST(SingularityHandler, ExitsBelowExitThreshold)
{
  auto h = makeHandler();
  h.update(18.0, qdot(0.10));               // enter coast
  auto d = h.update(13.0, qdot(0.10));      // < 14 → exit
  EXPECT_EQ(d.channel, VelocityChannel::CARTESIAN_TWIST);
  EXPECT_FALSE(h.coasting());
}

// No chatter: a value inside the band (e.g. 16) does NOT trigger entry from the
// Cartesian channel — entry requires crossing the upper κ_enter.
TEST(SingularityHandler, BandValueDoesNotTriggerEntry)
{
  auto h = makeHandler();
  auto d = h.update(/*kappa=*/16.0, qdot(0.10));   // below enter (17), above exit (14)
  EXPECT_EQ(d.channel, VelocityChannel::CARTESIAN_TWIST);
  EXPECT_FALSE(h.coasting());
}

// reset() drops the coast state and any held velocity.
TEST(SingularityHandler, ResetReturnsToCartesian)
{
  auto h = makeHandler();
  h.update(18.0, qdot(0.10));
  ASSERT_TRUE(h.coasting());

  h.reset();
  EXPECT_FALSE(h.coasting());
  auto d = h.update(16.0, qdot(0.10));      // band value, fresh start → Cartesian
  EXPECT_EQ(d.channel, VelocityChannel::CARTESIAN_TWIST);
}

// Full crossing cycle: cartesian → coast → cartesian.
TEST(SingularityHandler, FullCrossingCycle)
{
  auto h = makeHandler();
  EXPECT_EQ(h.update(8.0,  qdot(0.10)).channel, VelocityChannel::CARTESIAN_TWIST);
  EXPECT_EQ(h.update(19.0, qdot(0.12)).channel, VelocityChannel::JOINT_HOLD);   // enter
  EXPECT_EQ(h.update(25.0, qdot(0.05)).channel, VelocityChannel::JOINT_HOLD);   // peak
  EXPECT_EQ(h.update(15.0, qdot(0.05)).channel, VelocityChannel::JOINT_HOLD);   // still in band
  EXPECT_EQ(h.update(12.0, qdot(0.05)).channel, VelocityChannel::CARTESIAN_TWIST);  // exit
}
