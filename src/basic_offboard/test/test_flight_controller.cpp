#include <gtest/gtest.h>

#include "basic_offboard/flight_controller.hpp"

using basic_offboard::FlightController;
using State = FlightController::State;
using SP    = FlightController::Tick::SP;

static FlightController::Tick run_settle(FlightController & fc, double t0 = 0.0)
{
  // Run SETTLE_TICKS ticks. State stays LANDED, settle counter fills.
  FlightController::Tick t{};
  for (int i = 0; i < FlightController::SETTLE_TICKS; ++i) {
    t = fc.tick(t0 + 0.1 * i);
  }
  return t;
}

TEST(FlightController, StartsLanded)
{
  FlightController fc;
  EXPECT_EQ(fc.state(), State::LANDED);
}

TEST(FlightController, SettlePhasePublishesHoverNoArm)
{
  FlightController fc;
  for (int i = 0; i < FlightController::SETTLE_TICKS; ++i) {
    auto t = fc.tick(0.1 * i);
    EXPECT_TRUE(t.publish_setpoint);
    EXPECT_EQ(t.setpoint_kind, SP::POSITION);
    EXPECT_FALSE(t.send_arm);
    EXPECT_FALSE(t.send_offboard_mode);
    EXPECT_EQ(fc.state(), State::LANDED);
  }
}

TEST(FlightController, ArmsExactlyOnceAfterSettle)
{
  FlightController fc;
  run_settle(fc);

  auto t1 = fc.tick(1.0);
  EXPECT_TRUE(t1.send_arm);
  EXPECT_TRUE(t1.send_offboard_mode);
  EXPECT_EQ(fc.state(), State::TAKEOFF);

  auto t2 = fc.tick(1.1);
  EXPECT_FALSE(t2.send_arm);
  EXPECT_FALSE(t2.send_offboard_mode);
}

TEST(FlightController, TakeoffReachedTransitionsToHover)
{
  FlightController fc;
  run_settle(fc);
  fc.tick(1.0);   // armed, TAKEOFF

  // Simulate climbing past TAKEOFF_Z_NED + tolerance
  fc.on_local_position({1.0f, 2.0f, FlightController::TAKEOFF_Z_NED - 0.1f});
  auto t = fc.tick(1.1);
  EXPECT_EQ(fc.state(), State::HOVER);
  // Hover should snapshot the current position.
  EXPECT_TRUE(t.publish_setpoint);
  EXPECT_EQ(t.setpoint_kind, SP::POSITION);
  EXPECT_FLOAT_EQ(t.position.x, 1.0f);
  EXPECT_FLOAT_EQ(t.position.y, 2.0f);
}

TEST(FlightController, PositionSetpointAccepted)
{
  FlightController fc;
  run_settle(fc);
  fc.tick(1.0);
  fc.on_local_position({0.0f, 0.0f, FlightController::TAKEOFF_Z_NED});
  fc.tick(1.1);          // -> HOVER
  ASSERT_EQ(fc.state(), State::HOVER);

  fc.on_pos_setpoint({3.0f, 4.0f, -5.0f}, 1.57f);
  auto t = fc.tick(1.2);
  EXPECT_EQ(t.setpoint_kind, SP::POSITION);
  EXPECT_FLOAT_EQ(t.position.x, 3.0f);
  EXPECT_FLOAT_EQ(t.position.y, 4.0f);
  EXPECT_FLOAT_EQ(t.position.z, -5.0f);
  EXPECT_FLOAT_EQ(t.yaw,        1.57f);
}

TEST(FlightController, VelocityOverridesPositionWhenFresh)
{
  FlightController fc;
  run_settle(fc);
  fc.tick(1.0);
  fc.on_local_position({0.0f, 0.0f, FlightController::TAKEOFF_Z_NED});
  fc.tick(1.1);
  fc.on_pos_setpoint({10.0f, 0.0f, -5.0f}, 0.0f);
  fc.on_vel_setpoint({1.0f, 0.0f, 0.0f, 0.0f}, 1.2);
  auto t = fc.tick(1.2);
  EXPECT_EQ(t.setpoint_kind, SP::VELOCITY);
  EXPECT_TRUE(t.heartbeat_use_velocity);
  EXPECT_FLOAT_EQ(t.velocity.vx, 1.0f);
}

TEST(FlightController, VelocityExpiresFallsBackToHover)
{
  FlightController fc;
  run_settle(fc);
  fc.tick(1.0);
  fc.on_local_position({0.0f, 0.0f, FlightController::TAKEOFF_Z_NED});
  fc.tick(1.1);
  fc.on_vel_setpoint({1.0f, 0.0f, 0.0f, 0.0f}, 1.2);
  auto active = fc.tick(1.2);
  EXPECT_EQ(active.setpoint_kind, SP::VELOCITY);

  // Skip ahead beyond the timeout.
  auto expired = fc.tick(1.2 + FlightController::VEL_SP_TIMEOUT_S + 0.05);
  EXPECT_EQ(expired.setpoint_kind, SP::POSITION);
  EXPECT_FALSE(expired.heartbeat_use_velocity);
}

TEST(FlightController, LandFromHoverGoesToLanding)
{
  FlightController fc;
  run_settle(fc);
  fc.tick(1.0);
  fc.on_disarmed(false);   // PX4 reports armed once the arm command takes effect
  fc.on_local_position({0.0f, 0.0f, FlightController::TAKEOFF_Z_NED});
  fc.tick(1.1);
  ASSERT_EQ(fc.state(), State::HOVER);

  fc.on_land_command();
  auto t = fc.tick(1.2);
  EXPECT_TRUE(t.send_land);
  EXPECT_FALSE(t.publish_setpoint);   // no trajectory in LANDING
  EXPECT_EQ(fc.state(), State::LANDING);
}

TEST(FlightController, DisarmedDuringLandingGoesToLanded)
{
  FlightController fc;
  run_settle(fc);
  fc.tick(1.0);
  fc.on_disarmed(false);
  fc.on_local_position({0.0f, 0.0f, FlightController::TAKEOFF_Z_NED});
  fc.tick(1.1);
  fc.on_land_command();
  fc.tick(1.2);
  ASSERT_EQ(fc.state(), State::LANDING);

  // PX4 auto-disarms after touchdown.
  fc.on_disarmed(true);
  fc.tick(1.3);
  EXPECT_EQ(fc.state(), State::LANDED);
}

TEST(FlightController, PositionSetpointInTakeoffIgnored)
{
  FlightController fc;
  run_settle(fc);
  fc.tick(1.0);
  ASSERT_EQ(fc.state(), State::TAKEOFF);

  fc.on_pos_setpoint({99.0f, 99.0f, -5.0f}, 0.0f);
  // local position still well above takeoff altitude (z=0 → still climbing)
  fc.on_local_position({0.0f, 0.0f, 0.0f});
  auto t = fc.tick(1.1);
  EXPECT_NEAR(t.position.x, 0.0f, 1e-6);
  EXPECT_NEAR(t.position.y, 0.0f, 1e-6);
  EXPECT_EQ(t.position.z, FlightController::TAKEOFF_Z_NED);
}
