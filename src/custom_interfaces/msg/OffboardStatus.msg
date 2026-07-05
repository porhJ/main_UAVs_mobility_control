# Snapshot of offboard_master's authority state.
# Published latched (TRANSIENT_LOCAL) on /offboard/state so external
# requesters (mission_node, imav26 missions, CV nodes) can sequence safely.

builtin_interfaces/Time stamp

# --- Flight state (values match basic_offboard::FlightController::State) ---
uint8 STATE_TAKEOFF=0
uint8 STATE_HOVER=1
uint8 STATE_CRUISE=2
uint8 STATE_LANDING=3
uint8 STATE_LANDED=4
uint8 state

bool armed
bool offboard_active        # PX4 nav_state == OFFBOARD
bool accepting_setpoints    # external position/velocity setpoints honored in this state

# --- Flight mode (values match basic_offboard::FlightController::FlightMode) ---
uint8 MODE_HOVER=0          # multicopter (MC)
uint8 MODE_CRUISE=1         # fixed-wing (FW)
uint8 mode

# --- Geofence (values match basic_offboard::FlightController::GeofenceStatus) ---
uint8 GEOFENCE_OK=0
uint8 GEOFENCE_WARN=1        # a setpoint was clamped, or vehicle is within the fence margin
uint8 GEOFENCE_BREACH=2      # hard breach -> autonomous landing triggered
uint8 geofence_status

geometry_msgs/Point position   # current NED estimate (x=North, y=East, z=Down)
