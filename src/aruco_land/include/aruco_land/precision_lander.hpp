#pragma once

// Pure-logic precision-landing state machine. No ROS, no PX4, no Eigen.
// Depends only on <cstdint>/<cmath>/<vector>, so it unit-tests in ms.
//
// All coordinates are PX4 NED (x=North, y=East, z=Down; z negative = above ground).
//
// The shell feeds it a marker pose already transformed into world NED (the
// optical->NED transform lives in the shell, which has the drone attitude), the
// drone's local position + yaw, and a one-shot start request. Each tick() the
// brain returns a plain struct describing the position setpoint (if any) and
// whether to request an autonomous land.
//
// Sequence (ported from px4_basics_control landing_node.cpp):
//   IDLE     -> start requested + target fresh        -> SEARCH
//   SEARCH   -> target fresh                          -> APPROACH   (else fly spiral)
//   APPROACH -> horizontal dist to tag < delta_pos    -> DESCEND     (lost -> IDLE)
//   DESCEND  -> drone.z > land_trigger_alt            -> FINISHED    (lost -> IDLE)
//              (requests land on the transition)
//   FINISHED -> (terminal, no setpoint)

#include <cstdint>
#include <vector>

namespace aruco_land {

class PrecisionLander {
public:
  enum class State { IDLE, SEARCH, APPROACH, DESCEND, FINISHED };

  struct Vec3 { float x{0.0f}, y{0.0f}, z{0.0f}; };

  // Runtime configuration (injected by the shell from ROS params).
  // Altitude defaults sit inside offboard_master's default fence box
  // [z_ceiling = -3.0, z_floor = -0.2]; keep them consistent with the fence.
  struct Config {
    float search_alt        = -2.5f;   // spiral search altitude (NED)
    float approach_alt      = -2.0f;   // approach altitude (NED); inside the fence
    float land_trigger_alt  = -0.8f;   // request land once drone.z > this
    float descent_vel       = -0.4f;   // descent-rate factor (original semantics)
    float p_gain            =  1.2f;    // PI centering proportional gain
    float i_gain            =  0.0f;    // PI centering integral gain
    float max_vel           =  1.0f;    // clamp on centering velocity + integrator
    float target_timeout    =  1.0f;    // s; target considered lost after this
    float delta_pos         =  0.4f;    // m; approach->descend horizontal threshold
    float wp_accept         =  0.3f;    // m; spiral waypoint advance radius
    int   spiral_points     = 20;       // number of spiral waypoints
    float spiral_r_step     =  0.1f;    // radius growth per spiral index
    float spiral_angle_step =  0.5f;    // angle growth (rad) per spiral index
  };

  struct Tick {
    bool  has_setpoint{false};   // publish a position setpoint this tick?
    Vec3  position{};            // position-setpoint payload (NED)
    float yaw{0.0f};             // setpoint yaw (held at current drone yaw)
    bool  request_land{false};   // one-shot: send /offboard/cmd/land
    bool  state_changed{false};  // true if state_ changed during this tick
    State state{State::IDLE};    // state after this tick
  };

  PrecisionLander() { build_spiral(); }
  explicit PrecisionLander(Config cfg) : cfg_(cfg) { build_spiral(); }

  // --- Inputs (store only) ---
  void on_local_position(Vec3 p)                 { drone_pos_ = p; }
  void on_yaw(float yaw)                         { drone_yaw_ = yaw; }
  void on_target_world(Vec3 tag_ned, double now_s) { tag_pos_ = tag_ned; tag_seen_ = true; last_tag_s_ = now_s; }
  void on_start()                               { start_requested_ = true; }

  // --- Step ---
  Tick tick(double now_s);

  // --- Queries ---
  State state() const { return state_; }
  static const char * state_str(State s);

private:
  Config cfg_{};

  State state_{State::IDLE};

  Vec3   drone_pos_{};
  float  drone_yaw_{0.0f};

  Vec3   tag_pos_{};
  bool   tag_seen_{false};
  double last_tag_s_{-1.0e9};

  bool   start_requested_{false};

  std::vector<Vec3> spiral_;
  std::size_t       spiral_idx_{0};

  // PI centering integrators (descend phase)
  float  int_x_{0.0f}, int_y_{0.0f};

  double prev_now_s_{-1.0};   // for dt in the descend integrator

  void  build_spiral();
  bool  target_lost(double now_s) const;
  float dt_since(double now_s) const;   // seconds since previous tick (fallback 0.05)
  bool  set_state(State s);             // returns true if state changed
  void  compute_centering_vel(float & vx, float & vy);  // PI, clamped
};

}  // namespace aruco_land
