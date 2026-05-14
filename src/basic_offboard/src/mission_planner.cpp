#include "basic_offboard/mission_planner.hpp"

#include <cmath>

namespace basic_offboard {

void MissionPlanner::on_local_position(float x, float y, float z)
{
  lpx_ = x; lpy_ = y; lpz_ = z;
}

void MissionPlanner::set_endurance_waypoints(std::vector<Waypoint> wps)
{
  endu_wps_ = std::move(wps);
  endu_idx_ = 0;
}

void MissionPlanner::set_mapping_waypoints(std::vector<Waypoint> wps)
{
  map_wps_ = std::move(wps);
  map_idx_ = 0;
}

void MissionPlanner::on_drop_area(Waypoint wp)
{
  drop_wp_     = wp;
  has_drop_wp_ = true;
  if (mission_ == Mission::MAPPING) {
    drop_pending_ = true;
  }
}

float MissionPlanner::dist_to(const Waypoint & w) const
{
  const float dx = lpx_ - w.x;
  const float dy = lpy_ - w.y;
  const float dz = lpz_ - w.z;
  return std::sqrt(dx*dx + dy*dy + dz*dz);
}

bool MissionPlanner::transition_to(Mission next)
{
  if (mission_ == next) return false;
  mission_ = next;
  return true;
}

MissionPlanner::Tick MissionPlanner::tick()
{
  Tick out;
  out.mission = mission_;

  switch (mission_) {
    case Mission::ENDURANCE: {
      if (endu_wps_.empty()) return out;
      out.has_setpoint = true;
      out.setpoint     = endu_wps_[endu_idx_];

      if (dist_to(endu_wps_[endu_idx_]) > ACCEPT_RADIUS_M) break;

      endu_idx_++;
      if (endu_idx_ < endu_wps_.size()) break;

      endu_idx_ = 0;
      lap_count_++;
      if (lap_count_ >= desired_laps_) {
        out.mission_changed = transition_to(Mission::MAPPING);
        out.mission         = mission_;
      }
      break;
    }

    case Mission::MAPPING: {
      if (map_wps_.empty()) return out;
      out.has_setpoint = true;
      out.setpoint     = map_wps_[map_idx_];

      if (dist_to(map_wps_[map_idx_]) > ACCEPT_RADIUS_M) break;

      map_idx_ = (map_idx_ + 1) % map_wps_.size();

      if (drop_pending_ && has_drop_wp_) {
        drop_pending_       = false;
        map_resume_idx_     = map_idx_;
        out.mission_changed = transition_to(Mission::DROPPING);
        out.mission         = mission_;
      }
      break;
    }

    case Mission::DROPPING: {
      if (!has_drop_wp_) return out;
      out.has_setpoint = true;
      out.setpoint     = drop_wp_;

      if (dist_to(drop_wp_) > ACCEPT_RADIUS_M) break;

      map_idx_            = map_resume_idx_;
      out.mission_changed = transition_to(Mission::MAPPING);
      out.mission         = mission_;
      break;
    }
  }

  return out;
}

}  // namespace basic_offboard
