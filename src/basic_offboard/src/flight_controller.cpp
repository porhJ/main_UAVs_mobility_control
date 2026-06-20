#include "basic_offboard/flight_controller.hpp"

#include <algorithm>

namespace basic_offboard {

namespace {
float clampf(float v, float lo, float hi) { return std::max(lo, std::min(v, hi)); }
}  // namespace

void FlightController::on_pos_setpoint(PosNED p, float yaw)
{
  if (state_ != State::HOVER) return;    
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
  return local_pos_.z <= (cfg_.takeoff_z + TAKEOFF_TOL_M);
}

bool FlightController::set_state(State s)
{
  if (state_ == s) return false;
  state_ = s;
  return true;
}

bool FlightController::airborne() const
{
  return state_ == State::TAKEOFF || state_ == State::HOVER;
}

bool FlightController::hard_breach(PosNED p) const
{
  if (!cfg_.fence_enabled) return false;
  const float m = cfg_.fence_margin;
  return p.x < cfg_.fence_x_min - m || p.x > cfg_.fence_x_max + m ||
         p.y < cfg_.fence_y_min - m || p.y > cfg_.fence_y_max + m ||
         p.z < cfg_.fence_z_ceiling - m || p.z > cfg_.fence_z_floor + m;
}

FlightController::PosNED FlightController::clamp_to_fence(PosNED p)
{
  PosNED c = p;
  c.x = clampf(c.x, cfg_.fence_x_min, cfg_.fence_x_max);
  c.y = clampf(c.y, cfg_.fence_y_min, cfg_.fence_y_max);
  c.z = clampf(c.z, cfg_.fence_z_ceiling, cfg_.fence_z_floor);
  if (c.x != p.x || c.y != p.y || c.z != p.z) {
    geofence_status_ = std::max<uint8_t>(geofence_status_, GEOFENCE_WARN);
  }
  return c;
}

FlightController::VelNED FlightController::clamp_velocity(VelNED v)
{
  const float m = cfg_.fence_margin;
  bool clamped = false;
  // Zero a velocity component that would push the vehicle past a wall it is near.
  if (local_pos_.x >= cfg_.fence_x_max - m && v.vx > 0.0f) { v.vx = 0.0f; clamped = true; }
  if (local_pos_.x <= cfg_.fence_x_min + m && v.vx < 0.0f) { v.vx = 0.0f; clamped = true; }
  if (local_pos_.y >= cfg_.fence_y_max - m && v.vy > 0.0f) { v.vy = 0.0f; clamped = true; }
  if (local_pos_.y <= cfg_.fence_y_min + m && v.vy < 0.0f) { v.vy = 0.0f; clamped = true; }
  // NED z: vz < 0 climbs toward the ceiling, vz > 0 descends toward the floor.
  if (local_pos_.z <= cfg_.fence_z_ceiling + m && v.vz < 0.0f) { v.vz = 0.0f; clamped = true; }
  if (local_pos_.z >= cfg_.fence_z_floor - m && v.vz > 0.0f) { v.vz = 0.0f; clamped = true; }
  if (clamped) geofence_status_ = std::max<uint8_t>(geofence_status_, GEOFENCE_WARN);
  return v;
}

FlightController::Tick FlightController::tick(double now_s)
{
  Tick out;
  out.heartbeat_use_velocity = is_vel_sp_active(now_s);
  geofence_status_ = GEOFENCE_OK;   // re-derived each tick

  // Phase 1
  // requirement before arm/takeoff is to publish a few setpoints so PX4 considers the offboard stream valid.
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

  // Hard geofence: if the vehicle leaves the box (+ margin) while airborne,
  // abort to an autonomous landing.
  if (airborne() && hard_breach(local_pos_)) {
    geofence_status_ = GEOFENCE_BREACH;
    if (state_ != State::LANDING) {
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
      out.position         = PosNED{hover_.x, hover_.y, cfg_.takeoff_z};
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

  // Soft geofence: keep the published setpoint inside the box (raises WARN).
  if (cfg_.fence_enabled && out.publish_setpoint) {
    if (out.setpoint_kind == Tick::SP::POSITION) {
      out.position = clamp_to_fence(out.position);
    } else {
      out.velocity = clamp_velocity(out.velocity);
    }
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
