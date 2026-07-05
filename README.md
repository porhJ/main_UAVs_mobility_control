# main_control — `feat/vtol-fw-modes` branch

A ROS2 + PX4 offboard control template for a **VTOL / fixed-wing-capable**
multicopter. Extends the `feat/offboard-state-geofence` branch (MC + geofence
+ state feedback) with a **fixed-wing cruise mode**: external setpoints now
carry a *flight mode*, and the controller commands the PX4 VTOL transition
between multicopter **hover** and fixed-wing **cruise**.

For the plain MC template see `minimal_control`; for MC + geofence without any
FW code see `feat/offboard-state-geofence`.

---

## What's in here

| Package | Role |
|---|---|
| `src/basic_offboard/` | Control nodes — flight state machine + mission state machine |
| `src/custom_interfaces/` | Project messages: `Waypoints`, `OffboardStatus`, **`ModeSetpoint`** |
| `src/px4_msgs/` | Upstream PX4 ROS2 message definitions (submodule, do not edit) |
| `src/px4_ros_com/` | Upstream PX4↔ROS2 bridge utilities (submodule, do not edit) |

See [`src/basic_offboard/ARCHITECTURE.md`](src/basic_offboard/ARCHITECTURE.md)
for the full design walkthrough.

---

## Architecture in one sentence

ROS2 nodes are **thin transport shells** around two pure-C++ classes
(`FlightController`, `MissionPlanner`) that own all decision logic and have
zero dependency on `rclcpp` or `px4_msgs` — so the logic is unit-testable in
milliseconds without launching PX4.

```
ROS2 (offboard_master.cpp, mission_node.cpp)      ← pub/sub, timer, logging
  ↓ on_*(...)   ↑ tick() returns a Tick struct
Pure logic (FlightController, MissionPlanner)     ← state machines, decisions
```

---

## Flight state machine

```
TAKEOFF ──► HOVER ◄────► CRUISE ──► LANDING ──► LANDED
             (MC)  mode   (FW)
```

- **TAKEOFF** — always multicopter; climbs to the takeoff altitude, then HOVER.
- **HOVER** — multicopter. Accepts position *and* velocity setpoints.
- **CRUISE** — fixed-wing. Accepts **position setpoints only** (PX4 ignores
  velocity/acceleration for FW; the vehicle loiters around the position).
- **HOVER ↔ CRUISE** — driven by the requested mode; the shell issues
  `VEHICLE_CMD_DO_VTOL_TRANSITION` (`param1=4` FW / `3` MC).
- **LANDING** — a land request in CRUISE first transitions back to MC, then lands.

State, mode, and geofence verdict are published on **`/offboard/state`**
(`custom_interfaces/OffboardStatus`, latched) so external nodes can sequence
safely.

---

## Driving the drone by topic

After takeoff the drone enters `HOVER` and accepts setpoints. All coordinates
are **NED** (x=North, y=East, z=Down, z<0 above ground).

```bash
# Mode-carrying setpoint: hover (MC) + position
ros2 topic pub --once /offboard/setpoint/mode custom_interfaces/msg/ModeSetpoint \
  '{mode: 0, type: 0, position: {x: 2.0, y: 0.0, z: -5.0}}'

# Switch to fixed-wing cruise, fly toward a position (loiters around it)
ros2 topic pub --once /offboard/setpoint/mode custom_interfaces/msg/ModeSetpoint \
  '{mode: 1, type: 0, position: {x: 60.0, y: 0.0, z: -40.0}}'

# Back to multicopter hover
ros2 topic pub --once /offboard/setpoint/mode custom_interfaces/msg/ModeSetpoint \
  '{mode: 0, type: 0, position: {x: 0.0, y: 0.0, z: -5.0}}'

# Land (from cruise this auto-transitions to MC first)
ros2 topic pub --once /offboard/cmd/land std_msgs/Bool '{data: true}'
```

`ModeSetpoint`: `mode` (0 hover / 1 cruise), `type` (0 position / 1 velocity),
`position`+`yaw`, `velocity`+`yawspeed`. The legacy MC-only
`/offboard/setpoint/position` and `/offboard/setpoint/velocity` topics still
work for hover.

---

## Geofence

An axis-aligned NED box, enforced in the pure core, configured via `fence.*`
params on `offboard_master`:
- **Soft** — setpoints are clamped inside the box → `GEOFENCE_WARN`.
- **Hard** — the vehicle leaving the box + margin (in TAKEOFF/HOVER/CRUISE)
  forces an autonomous landing → `GEOFENCE_BREACH`.

---

## Build & run

```bash
source /opt/ros/humble/setup.bash
cd /home/porh/px4_dev/main_control
colcon build --packages-select custom_interfaces basic_offboard \
  --cmake-args -DCMAKE_BUILD_TYPE=RELWITHDEBINFO --symlink-install
source install/setup.bash

# Flight controller alone (drive it with the topics above)
ros2 run basic_offboard offboard_master

# OR the mission state machine (needs PX4 SITL + uXRCE-DDS agent running)
ros2 launch basic_offboard mission_launch.py            # default 3 endurance laps
```

---

## Testing

```bash
colcon test --packages-select basic_offboard
colcon test-result --verbose
```

27 unit tests (18 flight + 9 mission) run without PX4, the DDS bridge, or a ROS
daemon. Note: the HOVER↔CRUISE and land-from-cruise paths are implemented but
not yet covered by dedicated tests — add them before relying on FW flight.

To step through the logic in a debugger without ROS:

```bash
gdb --args build/basic_offboard/test_flight_controller --gtest_filter=*Land*
```

---

## Related branches

- **`minimal_control`** — MC-only template, no geofence/state/VTOL.
- **`feat/offboard-state-geofence`** — MC + geofence + `/offboard/state` (this branch's parent).
- **`feat/vtol-logic`** — an earlier, standalone VTOL experiment (different approach).
