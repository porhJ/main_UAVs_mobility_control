#pragma once

// Pure-logic mission state machine. No ROS, no PX4.
//
// Mission sequence (MC-only):
//   ENDURANCE -> desired_laps completed   -> MAPPING
//   MAPPING   -> drop area received       -> DROPPING
//   DROPPING  -> drop waypoint reached    -> MAPPING (resume at saved idx)

#include <cstddef>
#include <cstdint>
#include <vector>

namespace basic_offboard {

class MissionPlanner {
public:
  enum class Mission : uint8_t { ENDURANCE = 0, MAPPING = 1, DROPPING = 2 };

  struct Waypoint { float x{0.0f}, y{0.0f}, z{0.0f}, yaw{0.0f}; };

  struct Tick {
    bool     has_setpoint{false};
    Waypoint setpoint{};
    bool     mission_changed{false};
    Mission  mission{Mission::ENDURANCE};
  };

  // --- Inputs ---
  void on_local_position(float x, float y, float z);
  void set_endurance_waypoints(std::vector<Waypoint> wps);
  void set_mapping_waypoints(std::vector<Waypoint> wps);
  void on_drop_area(Waypoint wp);

  // --- Step ---
  Tick tick();

  // --- Config / queries ---
  void    set_desired_laps(int n)        { desired_laps_ = n; }
  Mission mission() const                { return mission_; }
  int     lap_count() const              { return lap_count_; }

  static constexpr float ACCEPT_RADIUS_M = 1.0f;

private:
  Mission mission_{Mission::ENDURANCE};

  float lpx_{0.0f}, lpy_{0.0f}, lpz_{0.0f};

  std::vector<Waypoint> endu_wps_;
  std::vector<Waypoint> map_wps_;
  Waypoint              drop_wp_{};
  bool                  has_drop_wp_{false};
  bool                  drop_pending_{false};

  size_t endu_idx_{0};
  size_t map_idx_{0};
  size_t map_resume_idx_{0};

  int lap_count_{0};
  int desired_laps_{3};

  bool  transition_to(Mission next);            // returns true if changed
  float dist_to(const Waypoint & w) const;
};

}  // namespace basic_offboard
