#pragma once

namespace main_control::model {

// Core coordinate types use PX4 local NED: x North, y East, z Down.
// They deliberately carry no frame-conversion or transport behavior.
struct PositionNed {
  float x{0.0f};
  float y{0.0f};
  float z{0.0f};
};

struct VelocityNed {
  float vx{0.0f};
  float vy{0.0f};
  float vz{0.0f};
  float yawspeed{0.0f};
};

struct PoseNed {
  float x{0.0f};
  float y{0.0f};
  float z{0.0f};
  float yaw{0.0f};
};

}  // namespace main_control::model
