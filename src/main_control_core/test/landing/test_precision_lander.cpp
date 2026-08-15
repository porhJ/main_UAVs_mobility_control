#include <gtest/gtest.h>

#include "main_control/landing/precision_lander.hpp"

using main_control::landing::PrecisionLander;
using State = PrecisionLander::State;
using Vec3  = PrecisionLander::Vec3;

// A small config with predictable, fence-friendly altitudes.
static PrecisionLander::Config test_cfg()
{
  PrecisionLander::Config c;
  c.search_alt       = -2.5f;
  c.approach_alt     = -2.0f;
  c.land_trigger_alt = -0.8f;
  c.descent_vel      = -0.4f;
  c.p_gain           =  1.2f;
  c.i_gain           =  0.0f;
  c.max_vel          =  1.0f;
  c.target_timeout   =  1.0f;
  c.delta_pos        =  0.4f;
  c.wp_accept        =  0.3f;
  c.spiral_points    =  20;
  return c;
}

TEST(PrecisionLander, StartsIdle)
{
  PrecisionLander pl(test_cfg());
  EXPECT_EQ(pl.state(), State::IDLE);
}

TEST(PrecisionLander, IdleStaysWithoutStartEvenWithTarget)
{
  PrecisionLander pl(test_cfg());
  pl.on_target_world({0.0f, 0.0f, 0.0f}, 0.0);
  auto t = pl.tick(0.05);
  EXPECT_EQ(pl.state(), State::IDLE);
  EXPECT_FALSE(t.has_setpoint);
}

TEST(PrecisionLander, StartWithFreshTargetEntersSearchThenApproach)
{
  PrecisionLander pl(test_cfg());
  pl.on_start();
  pl.on_target_world({1.0f, 1.0f, 0.0f}, 0.0);   // fresh target

  // On a fresh target, IDLE -> SEARCH, and SEARCH (target still fresh) -> APPROACH.
  auto t1 = pl.tick(0.05);
  EXPECT_TRUE(t1.state_changed);
  EXPECT_EQ(pl.state(), State::SEARCH);

  pl.tick(0.10);
  EXPECT_EQ(pl.state(), State::APPROACH);
}

TEST(PrecisionLander, SearchFliesSpiralWhenTargetLost)
{
  PrecisionLander pl(test_cfg());
  pl.on_start();
  // Provide a target far in the past so it is "lost" by t=2.0.
  pl.on_target_world({0.0f, 0.0f, 0.0f}, 0.0);
  pl.tick(0.05);                       // IDLE->SEARCH (target still fresh here)
  ASSERT_EQ(pl.state(), State::SEARCH);

  // Now the target is stale (>1 s). Search should emit a spiral waypoint at
  // the search altitude and stay in SEARCH.
  pl.on_local_position({5.0f, 5.0f, -2.5f});   // far from any spiral point
  auto t = pl.tick(2.0);
  EXPECT_EQ(pl.state(), State::SEARCH);
  ASSERT_TRUE(t.has_setpoint);
  EXPECT_FLOAT_EQ(t.position.z, -2.5f);
}

TEST(PrecisionLander, ApproachDescendsWhenCentered)
{
  PrecisionLander pl(test_cfg());
  pl.on_start();
  pl.on_target_world({0.0f, 0.0f, 0.0f}, 1.0);
  pl.tick(1.0);   // IDLE->SEARCH
  pl.tick(1.0);   // SEARCH->APPROACH
  ASSERT_EQ(pl.state(), State::APPROACH);

  // Drone still far horizontally -> stays in APPROACH, commands approach_alt.
  pl.on_local_position({3.0f, 0.0f, -2.0f});
  auto ta = pl.tick(1.0);
  EXPECT_EQ(pl.state(), State::APPROACH);
  ASSERT_TRUE(ta.has_setpoint);
  EXPECT_FLOAT_EQ(ta.position.z, -2.0f);

  // Drone within delta_pos of the tag -> DESCEND.
  pl.on_local_position({0.1f, 0.0f, -2.0f});
  pl.tick(1.0);
  EXPECT_EQ(pl.state(), State::DESCEND);
}

TEST(PrecisionLander, DescendRequestsLandNearGroundThenFinishes)
{
  PrecisionLander pl(test_cfg());
  pl.on_start();
  pl.on_target_world({0.0f, 0.0f, 0.0f}, 1.0);
  pl.tick(1.0);                                  // SEARCH
  pl.on_local_position({0.05f, 0.0f, -2.0f});
  pl.tick(1.0);                                  // APPROACH
  pl.tick(1.0);                                  // -> DESCEND
  ASSERT_EQ(pl.state(), State::DESCEND);

  // Above the trigger altitude: keep descending, no land yet.
  pl.on_target_world({0.0f, 0.0f, 0.0f}, 2.0);
  pl.on_local_position({0.0f, 0.0f, -2.0f});
  auto td = pl.tick(2.0);
  EXPECT_EQ(pl.state(), State::DESCEND);
  EXPECT_FALSE(td.request_land);
  EXPECT_TRUE(td.has_setpoint);

  // Near the ground (z > land_trigger_alt = -0.8): request land, go FINISHED.
  pl.on_target_world({0.0f, 0.0f, 0.0f}, 2.05);
  pl.on_local_position({0.0f, 0.0f, -0.5f});
  auto tl = pl.tick(2.05);
  EXPECT_TRUE(tl.request_land);
  EXPECT_EQ(pl.state(), State::FINISHED);

  // Land is a one-shot: FINISHED emits nothing further.
  auto tf = pl.tick(2.10);
  EXPECT_FALSE(tf.request_land);
  EXPECT_FALSE(tf.has_setpoint);
}

TEST(PrecisionLander, LostTargetDuringApproachAborts)
{
  PrecisionLander pl(test_cfg());
  pl.on_start();
  pl.on_target_world({0.0f, 0.0f, 0.0f}, 1.0);
  pl.tick(1.0);   // SEARCH
  pl.tick(1.0);   // APPROACH
  ASSERT_EQ(pl.state(), State::APPROACH);

  // No new target; advance time past the timeout -> abort to IDLE.
  auto t = pl.tick(3.0);
  EXPECT_EQ(pl.state(), State::IDLE);
  EXPECT_FALSE(t.has_setpoint);
}

TEST(PrecisionLander, LostTargetDuringDescendAborts)
{
  PrecisionLander pl(test_cfg());
  pl.on_start();
  pl.on_target_world({0.0f, 0.0f, 0.0f}, 1.0);
  pl.tick(1.0);                                  // SEARCH
  pl.on_local_position({0.05f, 0.0f, -2.0f});
  pl.tick(1.0);                                  // APPROACH
  pl.tick(1.0);                                  // DESCEND
  ASSERT_EQ(pl.state(), State::DESCEND);

  auto t = pl.tick(3.5);   // target stale -> abort
  EXPECT_EQ(pl.state(), State::IDLE);
  EXPECT_FALSE(t.has_setpoint);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
