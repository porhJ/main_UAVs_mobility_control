#include <gtest/gtest.h>

#include "basic_offboard/mission_planner.hpp"

using basic_offboard::MissionPlanner;
using Mission  = MissionPlanner::Mission;
using Waypoint = MissionPlanner::Waypoint;

static std::vector<Waypoint> square_wps(float z)
{
  return {
    {0.0f, 0.0f, z, 0.0f},
    {5.0f, 0.0f, z, 0.0f},
    {5.0f, 5.0f, z, 0.0f},
    {0.0f, 5.0f, z, 0.0f},
  };
}

TEST(MissionPlanner, StartsInEndurance)
{
  MissionPlanner p;
  EXPECT_EQ(p.mission(), Mission::ENDURANCE);
}

TEST(MissionPlanner, EmptyWaypointsNoSetpoint)
{
  MissionPlanner p;
  auto t = p.tick();
  EXPECT_FALSE(t.has_setpoint);
}

TEST(MissionPlanner, EndurancePublishesFirstWaypoint)
{
  MissionPlanner p;
  p.set_endurance_waypoints(square_wps(-5.0f));
  auto t = p.tick();
  EXPECT_TRUE(t.has_setpoint);
  EXPECT_FLOAT_EQ(t.setpoint.x, 0.0f);
  EXPECT_FLOAT_EQ(t.setpoint.y, 0.0f);
}

TEST(MissionPlanner, EnduranceAdvancesOnArrival)
{
  MissionPlanner p;
  p.set_endurance_waypoints(square_wps(-5.0f));
  // Arrive at wp 0
  p.on_local_position(0.0f, 0.0f, -5.0f);
  p.tick();
  // Next tick should target wp 1
  auto t = p.tick();
  EXPECT_FLOAT_EQ(t.setpoint.x, 5.0f);
  EXPECT_FLOAT_EQ(t.setpoint.y, 0.0f);
}

TEST(MissionPlanner, EnduranceTransitionsAfterDesiredLaps)
{
  MissionPlanner p;
  p.set_desired_laps(2);
  auto wps = square_wps(-5.0f);
  p.set_endurance_waypoints(wps);

  bool changed = false;
  for (int lap = 0; lap < 2; ++lap) {
    for (auto & w : wps) {
      p.on_local_position(w.x, w.y, w.z);
      auto t = p.tick();
      if (t.mission_changed) changed = true;
    }
  }
  EXPECT_TRUE(changed);
  EXPECT_EQ(p.mission(), Mission::MAPPING);
  EXPECT_EQ(p.lap_count(), 2);
}

TEST(MissionPlanner, MappingLoops)
{
  MissionPlanner p;
  p.set_desired_laps(1);
  p.set_endurance_waypoints(square_wps(-5.0f));
  p.set_mapping_waypoints(square_wps(-5.0f));

  // Burn one lap of endurance to enter MAPPING
  auto wps = square_wps(-5.0f);
  for (auto & w : wps) {
    p.on_local_position(w.x, w.y, w.z);
    p.tick();
  }
  ASSERT_EQ(p.mission(), Mission::MAPPING);

  // Verify mapping loops (size N, after N arrivals we're back at idx 0)
  for (size_t i = 0; i < wps.size(); ++i) {
    p.on_local_position(wps[i].x, wps[i].y, wps[i].z);
    p.tick();
  }
  auto t = p.tick();
  EXPECT_FLOAT_EQ(t.setpoint.x, wps[0].x);
  EXPECT_FLOAT_EQ(t.setpoint.y, wps[0].y);
}

TEST(MissionPlanner, DropAreaDuringEnduranceIgnored)
{
  MissionPlanner p;
  p.set_endurance_waypoints(square_wps(-5.0f));
  p.on_drop_area({2.0f, 2.0f, -3.0f, 0.0f});

  // Reach the first endurance waypoint — should NOT divert (still ENDURANCE).
  p.on_local_position(0.0f, 0.0f, -5.0f);
  auto t = p.tick();
  EXPECT_FALSE(t.mission_changed);
  EXPECT_EQ(p.mission(), Mission::ENDURANCE);
}

TEST(MissionPlanner, DropAreaDuringMappingDivertsToDropping)
{
  MissionPlanner p;
  p.set_desired_laps(1);
  p.set_endurance_waypoints(square_wps(-5.0f));
  p.set_mapping_waypoints(square_wps(-5.0f));

  // Complete endurance
  for (auto & w : square_wps(-5.0f)) {
    p.on_local_position(w.x, w.y, w.z);
    p.tick();
  }
  ASSERT_EQ(p.mission(), Mission::MAPPING);

  p.on_drop_area({2.0f, 2.0f, -3.0f, 0.0f});

  // Reach next mapping waypoint -> should divert
  auto wps = square_wps(-5.0f);
  p.on_local_position(wps[0].x, wps[0].y, wps[0].z);
  auto t = p.tick();
  EXPECT_TRUE(t.mission_changed);
  EXPECT_EQ(p.mission(), Mission::DROPPING);
}

TEST(MissionPlanner, DropReachedResumesMapping)
{
  MissionPlanner p;
  p.set_desired_laps(1);
  p.set_endurance_waypoints(square_wps(-5.0f));
  p.set_mapping_waypoints(square_wps(-5.0f));

  // Drive to MAPPING
  for (auto & w : square_wps(-5.0f)) {
    p.on_local_position(w.x, w.y, w.z);
    p.tick();
  }
  // Set drop, enter DROPPING
  p.on_drop_area({2.0f, 2.0f, -3.0f, 0.0f});
  auto wps = square_wps(-5.0f);
  p.on_local_position(wps[0].x, wps[0].y, wps[0].z);
  p.tick();
  ASSERT_EQ(p.mission(), Mission::DROPPING);

  // Arrive at drop -> resume MAPPING
  p.on_local_position(2.0f, 2.0f, -3.0f);
  auto t = p.tick();
  EXPECT_TRUE(t.mission_changed);
  EXPECT_EQ(p.mission(), Mission::MAPPING);
}
