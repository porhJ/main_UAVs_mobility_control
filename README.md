# main_control — `minimal_control` branch

A **minimal, MC-only** ROS2 + PX4 offboard control template. Forked from
the full multi-vehicle codebase on `main` and stripped down to be easy to
read, debug, and extend.

If you want the full MC/FW/VTOL version with mode-switching, see the `main`
branch. If you want a clean starting point for a multicopter offboard
project — you're in the right place.

---

## What's in here

| Package | Role |
|---|---|
| `src/basic_offboard/` | The control nodes — flight state machine + mission state machine |
| `src/custom_interfaces/` | Project-specific ROS messages (`Waypoints.msg`) |
| `src/px4_msgs/` | Upstream PX4 ROS2 message definitions (submodule, do not edit) |
| `src/px4_ros_com/` | Upstream PX4↔ROS2 bridge utilities (submodule, do not edit) |

The interesting code is in `basic_offboard/`. See
[`src/basic_offboard/ARCHITECTURE.md`](src/basic_offboard/ARCHITECTURE.md)
for the full design walkthrough and rationale.

---

## Architecture in one sentence

ROS2 nodes are **thin transport shells** around two pure-C++ classes
(`FlightController`, `MissionPlanner`) that own all decision logic and
have zero dependency on `rclcpp` or `px4_msgs` — so logic is unit-testable
in <10 ms without launching PX4.

```
ROS2 (offboard_master.cpp, mission_node.cpp)      ← pub/sub, timer, logging
  ↓ on_*(...)   ↑ tick() returns a Tick struct
Pure logic (FlightController, MissionPlanner)     ← state machines, decisions
```

---

## Quick start

**Prerequisites:** ROS2 Humble, PX4 SITL + uXRCE-DDS agent (or real hardware
with the bridge configured).

```bash
# 1. Build
source /opt/ros/humble/setup.bash
cd /home/porh/px4_dev/main_control
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RELWITHDEBINFO --symlink-install
source install/setup.bash

# 2. Start PX4 SITL + the DDS agent (in separate terminals, per your existing setup)

# 3a. Run the flight controller alone (manual setpoint driving)
ros2 run basic_offboard offboard_master

# 3b. OR run the full launch with mission state machine
ros2 launch basic_offboard mission_launch.py            # default 3 endurance laps
ros2 launch basic_offboard mission_launch.py desired_laps:=1

# 4. (Optional) Mock waypoint provider for sim
ros2 run basic_offboard waypoint_test_node
```

---

## Driving the drone by topic

After takeoff completes automatically (climb to 5 m AGL), the drone enters
`HOVER` and accepts external setpoints. All coordinates are **NED**
(x=North, y=East, z=Down, z<0 above ground).

```bash
# Fly to a point
ros2 topic pub --once /offboard/setpoint/position geometry_msgs/PoseStamped \
  '{pose: {position: {x: 2.0, y: 0.0, z: -5.0}}}'

# Continuous velocity (republish at >= 2 Hz or it expires)
ros2 topic pub --rate 10 /offboard/setpoint/velocity geometry_msgs/TwistStamped \
  '{twist: {linear: {x: 1.0}}}'

# Land
ros2 topic pub --once /offboard/cmd/land std_msgs/Bool '{data: true}'
```

---

## Testing

```bash
colcon test --packages-select basic_offboard
colcon test-result --verbose
```

19 unit tests covering the flight + mission state machines. They do not
require PX4, the DDS bridge, or even a running ROS daemon.

To step through the logic in a debugger without ROS:

```bash
gdb --args build/basic_offboard/test_flight_controller --gtest_filter=*Land*
```

---

## Where to go next

- **Modify behavior** → `src/basic_offboard/include/basic_offboard/flight_controller.hpp`
  and the matching `.cpp`. The ROS shell shouldn't need changes for most logic edits.
- **Add a new mission** → `mission_planner.{hpp,cpp}`.
- **Add a new module** (e.g. velocity limiter, geofence) → create a new `.hpp/.cpp`
  pair in `basic_offboard/`, add it as a library in `CMakeLists.txt`, link it
  to the consuming node.
- **Understand the design choices** → [`src/basic_offboard/ARCHITECTURE.md`](src/basic_offboard/ARCHITECTURE.md)
  has a "why" paragraph for every non-obvious decision.

---

## Branch policy

- **`main`** — full MC/FW/VTOL implementation with mode-switching service.
- **`minimal_control`** (this branch) — MC-only template. Treat as a clean
  starting point; copy and rename for new projects.
