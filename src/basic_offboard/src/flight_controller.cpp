#include "basic_offboard/flight_controller.hpp"

namespace basic_offboard {

void FlightController::on_pos_setpoint(PosNED p, float yaw)
{
  if (state_ != State::HOVER) return;       // ignore in non-HOVER states
  pos_sp_       = p;
  pos_sp_yaw_   = yaw;
  pos_sp_valid_ = true;
}

void FlightController::on_vel_setpoint(VelNED v, double now_s)
{
  if (state_ != State::HOVER) return;
  vel_sp_       = v;
  vel_sp_t_     = now_s;
  vel_sp_valid_ = true;
}

bool FlightController::is_vel_sp_active(double now_s) const
{
  return vel_sp_valid_ && (now_s - vel_sp_t_) < VEL_SP_TIMEOUT_S;
}

bool FlightController::takeoff_reached() const
{
  return local_pos_.z <= (TAKEOFF_Z_NED + TAKEOFF_TOL_M);
}

bool FlightController::set_state(State s)
{
  if (state_ == s) return false;
  state_ = s;
  return true;
}

FlightController::Tick FlightController::tick(double now_s)
{
  Tick out;
  out.heartbeat_use_velocity = is_vel_sp_active(now_s);

  // Phase 1: accumulate 10 heartbeats + hover setpoints before arming.
  if (settle_counter_ < SETTLE_TICKS) {
    settle_counter_++;
    out.publish_setpoint = true;
    out.setpoint_kind    = Tick::SP::POSITION;
    out.position         = hover_;
    out.yaw              = hover_yaw_;
    return out;
  }

  // Phase 2: send arm command until it's armed or send exceed tolerance to avoid spamming if something goes wrong.
  // yes, this is my design hehe
  if (armed_sent_ == 0){
    out.state_changed = set_state(State::TAKEOFF) || out.state_changed;
  }

  if ((nav_state_ != NAV_STATE_OFFBOARD || disarmed_) && armed_sent_ < ARM_CMD_TOL) {
    armed_sent_++;
    out.send_arm           = true;
    out.send_offboard_mode = true;
  }
  

  // Land request consumed here so it acts at most once per call.
  if (land_requested_) {
    land_requested_ = false;
    if (state_ == State::TAKEOFF || state_ == State::HOVER) {
      out.send_land     = true;
      out.state_changed = set_state(State::LANDING) || out.state_changed;
    }
  }

  // State transitions driven by vehicle telemetry.
  switch (state_) {
    case State::TAKEOFF:
      if (takeoff_reached()) {
        hover_ = local_pos_;
        out.state_changed = set_state(State::HOVER) || out.state_changed;
      }
      break;
    case State::LANDING:
      if (disarmed_) {
        out.state_changed = set_state(State::LANDED) || out.state_changed;
      }
      break;
    default:
      break;
  }

  // Setpoint selection per state.
  switch (state_) {
    case State::TAKEOFF:
      out.publish_setpoint = true;
      out.setpoint_kind    = Tick::SP::POSITION;
      out.position         = PosNED{hover_.x, hover_.y, TAKEOFF_Z_NED};
      out.yaw              = hover_yaw_;
      break;

    case State::HOVER:
      out.publish_setpoint = true;
      if (is_vel_sp_active(now_s)) {
        out.setpoint_kind = Tick::SP::VELOCITY;
        out.velocity      = vel_sp_;
        // Track current position so hover fallback is fresh when velocity expires.
        hover_ = local_pos_;
      } else if (pos_sp_valid_) {
        out.setpoint_kind = Tick::SP::POSITION;
        out.position      = pos_sp_;
        out.yaw           = pos_sp_yaw_;
        hover_     = pos_sp_;
        hover_yaw_ = pos_sp_yaw_;
      } else {
        out.setpoint_kind = Tick::SP::POSITION;
        out.position      = hover_;
        out.yaw           = hover_yaw_;
      }
      break;

    case State::LANDING:
    case State::LANDED:
      // PX4 lands autonomously after VEHICLE_CMD_NAV_LAND.
      // We keep publishing the heartbeat but skip the trajectory setpoint.
      out.publish_setpoint = false;
      break;
  }

  return out;
}

const char * FlightController::state_str(State s)
{
  switch (s) {
    case State::TAKEOFF: return "TAKEOFF";
    case State::HOVER:   return "HOVER";
    case State::LANDING: return "LANDING";
    case State::LANDED:  return "LANDED";
  }
  return "UNKNOWN";
}

}  // namespace basic_offboard
