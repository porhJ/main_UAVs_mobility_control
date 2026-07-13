#include "aruco_land/precision_lander.hpp"

#include <algorithm>
#include <cmath>

namespace aruco_land {

void PrecisionLander::build_spiral()
{
  // Ported from generateSearchSpiral(): radius r = r_step*i, angle = angle_step*i,
  // all at the search altitude. Centered on the local origin.
  spiral_.clear();
  spiral_.reserve(static_cast<std::size_t>(std::max(0, cfg_.spiral_points)));
  for (int i = 0; i < cfg_.spiral_points; ++i) {
    const float angle = cfg_.spiral_angle_step * static_cast<float>(i);
    const float r     = cfg_.spiral_r_step * static_cast<float>(i);
    spiral_.push_back({r * std::cos(angle), r * std::sin(angle), cfg_.search_alt});
  }
}

bool PrecisionLander::target_lost(double now_s) const
{
  if (!tag_seen_) return true;
  return (now_s - last_tag_s_) > cfg_.target_timeout;
}

float PrecisionLander::dt_since(double now_s) const
{
  if (prev_now_s_ < 0.0) return 0.05f;                 // first tick: assume 20 Hz
  const float dt = static_cast<float>(now_s - prev_now_s_);
  return std::clamp(dt, 0.0f, 0.2f);                   // guard against timer stalls
}

bool PrecisionLander::set_state(State s)
{
  if (state_ == s) return false;
  state_ = s;
  return true;
}

void PrecisionLander::compute_centering_vel(float & vx, float & vy)
{
  // Ported from computePIControlXY(): drive the horizontal error to zero.
  const float dx = drone_pos_.x - tag_pos_.x;
  const float dy = drone_pos_.y - tag_pos_.y;

  int_x_ += dx;
  int_y_ += dy;
  int_x_ = std::clamp(int_x_, -cfg_.max_vel, cfg_.max_vel);
  int_y_ = std::clamp(int_y_, -cfg_.max_vel, cfg_.max_vel);

  vx = -(cfg_.p_gain * dx + cfg_.i_gain * int_x_);
  vy = -(cfg_.p_gain * dy + cfg_.i_gain * int_y_);
  vx = std::clamp(vx, -cfg_.max_vel, cfg_.max_vel);
  vy = std::clamp(vy, -cfg_.max_vel, cfg_.max_vel);
}

PrecisionLander::Tick PrecisionLander::tick(double now_s)
{
  Tick out;
  const bool lost = target_lost(now_s);
  const float dt  = dt_since(now_s);

  switch (state_) {
    case State::IDLE: {
      // Armed by on_start(); begins as soon as a fresh marker is available.
      // start_requested_ is a latch (matches the original), not a one-shot: it
      // stays set so an abort back to IDLE can re-acquire on the next target.
      if (start_requested_ && !lost) {
        out.state_changed = set_state(State::SEARCH);
      }
      break;
    }

    case State::SEARCH: {
      if (!lost) {
        out.state_changed = set_state(State::APPROACH);
        break;
      }
      if (spiral_.empty()) break;
      const Vec3 wp = spiral_[spiral_idx_];
      out.has_setpoint = true;
      out.position     = wp;
      out.yaw          = drone_yaw_;

      const float ex = wp.x - drone_pos_.x;
      const float ey = wp.y - drone_pos_.y;
      const float ez = wp.z - drone_pos_.z;
      if (std::sqrt(ex*ex + ey*ey + ez*ez) < cfg_.wp_accept) {
        spiral_idx_ = (spiral_idx_ + 1) % spiral_.size();
      }
      break;
    }

    case State::APPROACH: {
      if (lost) {
        out.state_changed = set_state(State::IDLE);
        break;
      }
      out.has_setpoint = true;
      out.position     = {tag_pos_.x, tag_pos_.y, cfg_.approach_alt};
      out.yaw          = drone_yaw_;

      const float hx = drone_pos_.x - tag_pos_.x;
      const float hy = drone_pos_.y - tag_pos_.y;
      if (std::sqrt(hx*hx + hy*hy) < cfg_.delta_pos) {
        out.state_changed = set_state(State::DESCEND);
      }
      break;
    }

    case State::DESCEND: {
      if (lost) {
        out.state_changed = set_state(State::IDLE);
        break;
      }
      float vx = 0.0f, vy = 0.0f;
      compute_centering_vel(vx, vy);

      out.has_setpoint = true;
      out.position     = {drone_pos_.x + vx * dt,
                          drone_pos_.y + vy * dt,
                          drone_pos_.z - cfg_.descent_vel * dt};
      out.yaw          = drone_yaw_;

      if (drone_pos_.z > cfg_.land_trigger_alt) {  // close enough to the ground
        out.request_land  = true;
        out.state_changed = set_state(State::FINISHED);
      }
      break;
    }

    case State::FINISHED:
      break;
  }

  out.state   = state_;
  prev_now_s_ = now_s;
  return out;
}

const char * PrecisionLander::state_str(State s)
{
  switch (s) {
    case State::IDLE:     return "IDLE";
    case State::SEARCH:   return "SEARCH";
    case State::APPROACH: return "APPROACH";
    case State::DESCEND:  return "DESCEND";
    case State::FINISHED: return "FINISHED";
  }
  return "UNKNOWN";
}

}  // namespace aruco_land
