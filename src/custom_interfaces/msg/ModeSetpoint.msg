# Setpoint + the flight mode it must execute in. Replaces
# /offboard/setpoint/{position,velocity}. Coordinates are PX4 NED
# (x=North, y=East, z=Down; altitude negative).

# --- Flight mode (values match basic_offboard::FlightController::FlightMode) ---
uint8 MODE_HOVER=0      # multicopter (MC)
uint8 MODE_CRUISE=1     # fixed-wing (FW)
uint8 mode

# --- Setpoint kind ---
uint8 TYPE_POSITION=0
uint8 TYPE_VELOCITY=1
uint8 type

geometry_msgs/Point   position    # used when type == TYPE_POSITION
float32 yaw
geometry_msgs/Vector3 velocity    # used when type == TYPE_VELOCITY
float32 yawspeed
