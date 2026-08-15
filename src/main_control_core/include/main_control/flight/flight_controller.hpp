#pragma once

// Pure-logic flight state machine. No ROS, no PX4. Unit-testable.
//
// All coordinates are PX4 NED (z negative = above ground).
//
// Usage from a ROS node:
//   1. On every PX4/external message, call the matching on_*() input method.
//   2. At a fixed rate (10 Hz), call tick(now_s) and act on the returned struct:
//        - always publish OffboardControlMode using Tick.heartbeat_use_velocity
//        - if Tick.publish_setpoint, publish a TrajectorySetpoint
//        - if Tick.send_arm / send_offboard_mode / send_land, issue those one-shots

#include <cstdint>

#include "main_control/model/types.hpp"

namespace basic_offboard {

class FlightController {
public:
  enum class State { TAKEOFF, HOVER, CRUISE, LANDING, LANDED };
  enum class FlightMode : uint8_t { HOVER=0, CRUISE=1 };

  // Mirrors custom_interfaces/msg/OffboardStatus geofence constants.
  enum GeofenceStatus : uint8_t { GEOFENCE_OK = 0, GEOFENCE_WARN = 1, GEOFENCE_BREACH = 2 };

  using PosNED = main_control::model::PositionNed;
  using VelNED = main_control::model::VelocityNed;

  // Tunables (exposed for tests)
  static constexpr float  TAKEOFF_Z_NED    = -5.0f;
  static constexpr float  TAKEOFF_TOL_M    =  0.3f;
  static constexpr double VEL_SP_TIMEOUT_S =  0.5;
  static constexpr int    SETTLE_TICKS     = 10;
  static constexpr uint8_t NAV_STATE_OFFBOARD = 14;
  static constexpr int ARM_CMD_TOL = 100;

  // Runtime configuration (injected by the shell from ROS params).
  // Geofence is an axis-aligned box in PX4 local NED (z Down, negative = up).
  // Library default leaves the fence OFF so the core's default behaviour is
  // unconstrained; offboard_master enables it via the `fence.*` params.
  struct Config {
    bool  fence_enabled   = false;
    float fence_x_min     = -1.0f;
    float fence_x_max     = 12.0f;
    float fence_y_min     = -3.5f;
    float fence_y_max     =  3.5f;
    float fence_z_ceiling = -3.0f;   // most-negative z allowed (altitude cap)
    float fence_z_floor   = -0.2f;   // least-negative z allowed (never into the ground)
    float fence_margin    =  0.5f;   // hard-breach distance beyond the soft fence
    float takeoff_z       = TAKEOFF_Z_NED;  // takeoff target (should be >= fence_z_ceiling)
  };

  FlightController() = default;   // default Config: fence off, takeoff_z = TAKEOFF_Z_NED
  explicit FlightController(Config cfg) : cfg_(cfg) { hover_.z = cfg_.takeoff_z; }

  struct Tick {
    enum class SP { POSITION, VELOCITY };
    bool   heartbeat_use_velocity{false};
    bool   publish_setpoint{true};
    SP     setpoint_kind{SP::POSITION};
    PosNED position{};
    float  yaw{0.0f};
    VelNED velocity{};
    bool send_mode_transition{false}; // true when change from hover -> cruise or vise versa (for vtol)
    bool   send_arm{false};
    bool   send_offboard_mode{false};
    bool   send_land{false};
    bool   state_changed{false};   // true if state_ changed during this tick
  };

  // --- Inputs ---
  void on_local_position(PosNED p)              { local_pos_ = p; }
  void on_disarmed(bool disarmed)               { disarmed_ = disarmed; }
  void on_nav_state(uint8_t nav_state)             { nav_state_ = nav_state; }
  void on_pos_setpoint(PosNED p, float yaw);
  void on_vel_setpoint(VelNED v, double now_s);
  void on_land_command()                        { land_requested_ = true; }
  void on_mode(uint8_t m) { mode_ = m; }

  // --- Step ---
  Tick tick(double now_s);

  // --- Queries ---
  State   state() const                         { return state_; }
  // True when the current state honors external position/velocity setpoints.
  bool    accepts_setpoints() const             { return state_ == State::HOVER || state_ == State::CRUISE; }
  // Geofence verdict from the most recent tick(): OK / WARN / BREACH.
  uint8_t geofence_status() const               { return geofence_status_; }
  PosNED  local_position() const                { return local_pos_; }
  static const char * state_str(State s);

private:
  Config cfg_{};

  State  state_{State::LANDED};

  PosNED local_pos_{};
  bool   disarmed_{true};
  uint8_t   nav_state_{0}; // 0 is manual mode
  PosNED pos_sp_{};
  float  pos_sp_yaw_{0.0f};
  bool   pos_sp_valid_{false};

  VelNED vel_sp_{};
  double vel_sp_t_{-1.0e9};
  bool   vel_sp_valid_{false};

  // Hover hold (NED). Tracks current position while flying so that an
  // expired velocity setpoint or a missing position setpoint produces a
  // sensible hold target.
  PosNED hover_{0.0f, 0.0f, TAKEOFF_Z_NED};
  float  hover_yaw_{0.0f};

  // Activation sequence
  int  settle_counter_{0};
  int armed_sent_{0}; // counts ticks since arm command sent

  uint8_t mode_{0};          // requested mode: 0 hover (MC), 1 cruise (FW) — mirrors FlightMode
  uint8_t setpoint_type_{0}; // last accepted setpoint kind: 0 position, 1 velocity

  bool land_requested_{false};

  // Geofence verdict, re-derived every tick().
  uint8_t geofence_status_{GEOFENCE_OK};

  bool set_state(State s);             // returns true if state changed
  bool is_vel_sp_active(double now_s) const;
  bool is_transitioning() const;       // requested mode differs from current airborne state
  bool takeoff_reached() const;

  // Geofence (NED). clamp_* may raise geofence_status_ to WARN.
  bool   airborne() const;                 // state is TAKEOFF, HOVER or CRUISE
  bool   hard_breach(PosNED p) const;      // vehicle outside the box + margin
  PosNED clamp_to_fence(PosNED p);         // soft-clamp a position setpoint
  VelNED clamp_velocity(VelNED v);         // zero outward components near a wall
};

}  // namespace basic_offboard

namespace main_control::flight {

using FlightController = ::basic_offboard::FlightController;

}  // namespace main_control::flight
