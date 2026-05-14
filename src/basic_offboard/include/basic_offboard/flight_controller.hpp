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

namespace basic_offboard {

class FlightController {
public:
  enum class State { TAKEOFF, HOVER, LANDING, LANDED };

  struct PosNED { float x{0.0f}, y{0.0f}, z{0.0f}; };
  struct VelNED { float vx{0.0f}, vy{0.0f}, vz{0.0f}, yawspeed{0.0f}; };

  struct Tick {
    enum class SP { POSITION, VELOCITY };
    bool   heartbeat_use_velocity{false};
    bool   publish_setpoint{true};
    SP     setpoint_kind{SP::POSITION};
    PosNED position{};
    float  yaw{0.0f};
    VelNED velocity{};
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

  // --- Step ---
  Tick tick(double now_s);

  // --- Queries ---
  State state() const                           { return state_; }
  static const char * state_str(State s);

  // Tunables (exposed for tests)
  static constexpr float  TAKEOFF_Z_NED    = -5.0f;
  static constexpr float  TAKEOFF_TOL_M    =  0.3f;
  static constexpr double VEL_SP_TIMEOUT_S =  0.5;
  static constexpr int    SETTLE_TICKS     = 10;
  static constexpr uint8_t NAV_STATE_OFFBOARD = 14;
  static constexpr int ARM_CMD_TOL = 100;

private:
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

  bool land_requested_{false};

  bool set_state(State s);             // returns true if state changed
  bool is_vel_sp_active(double now_s) const;
  bool takeoff_reached() const;
};

}  // namespace basic_offboard
