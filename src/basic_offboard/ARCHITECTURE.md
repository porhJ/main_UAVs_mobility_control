# `basic_offboard` — Architecture & Usage

This document explains how the package is organized, **why** it is organized
that way, and how to use, debug, and extend it.

If you only want to fly the drone, jump to [Running](#running).
If you want to change behavior, read [Layers](#two-layer-architecture) first.

---

## What this package does

It is the only ROS2 node group authorized to issue commands to the PX4
autopilot over the uXRCE-DDS bridge. It:

1. Maintains the PX4 offboard-mode heartbeat (`OffboardControlMode` at 10 Hz).
2. Runs a flight state machine: `TAKEOFF → HOVER → LANDING → LANDED`.
3. Accepts external setpoints (position, velocity, land command).
4. Optionally runs a mission state machine (`ENDURANCE → MAPPING ↔ DROPPING`)
   that publishes waypoints to itself.

Target vehicle: **multicopter only**. There is no FW or VTOL code path.

---

## Two-layer architecture

```
                                   ┌────────────────────────────────────┐
   /fmu/in/*       /fmu/out/*      │            ROS2 SHELL              │
       ▲                ▼          │  offboard_master.cpp               │
       │                │          │  mission_node.cpp                  │
       └────────────────┴──────────┤  • pub/sub + QoS                   │
                                   │  • timers                          │
                                   │  • ROS msg ↔ plain-struct convert  │
                                   │  • logging                         │
                                   └────────────────┬───────────────────┘
                                                    │  on_*(...)  /  tick()
                                                    ▼
                                   ┌────────────────────────────────────┐
                                   │           PURE LOGIC               │
                                   │  flight_controller.{hpp,cpp}       │
                                   │  mission_planner.{hpp,cpp}         │
                                   │  • state machines                  │
                                   │  • setpoint selection              │
                                   │  • acceptance-radius math          │
                                   │  • NO rclcpp / NO px4_msgs         │
                                   └────────────────────────────────────┘
```

### Why split it like this

The original package mixed transport (publishers, subscribers, QoS, timestamp
fiddling) with decisions (which setpoint to send, when to transition state).
That mixing has four practical costs:

1. **Untestable.** A unit test of "land command in LANDED should be ignored"
   needed to start a node, fake a ROS clock, and pump callbacks. So the test
   never got written, and the behavior drifted.
2. **Slow debug loop.** Every behavioral check required launching PX4 SITL,
   the DDS agent, and the node graph (~30 s round trip).
3. **Coupled changes.** Renaming a topic touched the same file that owned
   the takeoff threshold. A change to either risked breaking the other.
4. **One-way port.** Moving to Python (which is what we'd want if logic
   stays simple and the user wants quick iteration) would mean rewriting
   the whole thing instead of just the shell.

The split fixes all four. The pure layer is plain C++ that depends only on
`<cstdint>`, `<vector>`, `<cmath>`. You can `g++ flight_controller.cpp
test_flight_controller.cpp -lgtest` outside the ROS workspace if you want.
The full test suite runs in **under 10 ms**.

### Why C++ and not Python

The user asked whether Python with ROS-only-at-the-boundary would be a good
fit. Answer: yes, for the *shape* of the code, but C++ is fine for the
*language* because:

- We already have C++ infra (CMakeLists, ament, gtest) wired up.
- `rclcpp` callbacks have lower jitter than `rclpy` at 10 Hz under load.
- The pure-logic layer is so small (~250 LOC) that the language matters
  less than the separation.

If you ever do port to Python, only the shell files change. The pure-logic
classes map 1:1 to dataclasses + methods.

---

## File map

```
src/basic_offboard/
├── include/basic_offboard/
│   ├── flight_controller.hpp    # Pure logic — flight state machine
│   └── mission_planner.hpp      # Pure logic — mission state machine
├── src/
│   ├── flight_controller.cpp    # Impl
│   ├── mission_planner.cpp      # Impl
│   ├── offboard_master.cpp      # ROS shell over FlightController
│   ├── mission_node.cpp         # ROS shell over MissionPlanner
│   └── waypoint_test_node.cpp   # Mock waypoint publisher (sim/dev only)
├── test/
│   ├── test_flight_controller.cpp
│   └── test_mission_planner.cpp
├── launch/mission_launch.py
├── CMakeLists.txt
└── package.xml
```

---

## Pure-logic components

### `FlightController`

Owns the per-flight state machine.

```cpp
enum class State { TAKEOFF, HOVER, LANDING, LANDED };
```

**Inputs** (from the ROS shell, in any order, any rate):

| Method | When the shell calls it |
|---|---|
| `on_local_position(p)` | `/fmu/out/vehicle_odometry` callback |
| `on_disarmed(bool)` | `/fmu/out/vehicle_status` callback |
| `on_pos_setpoint(p, yaw)` | `/offboard/setpoint/position` callback |
| `on_vel_setpoint(v, now_s)` | `/offboard/setpoint/velocity` callback |
| `on_land_command()` | `/offboard/cmd/land` callback (true value) |

**Step**: `Tick tick(double now_s)` — called at 10 Hz by the shell timer.
Returns a struct describing what the shell should publish *this tick*:

| Field | Meaning |
|---|---|
| `heartbeat_use_velocity` | Whether the `OffboardControlMode` message should mark the velocity field active |
| `publish_setpoint` | Whether to publish a `TrajectorySetpoint` at all |
| `setpoint_kind` | `POSITION` or `VELOCITY` |
| `position`, `yaw` | Position-mode payload |
| `velocity` | Velocity-mode payload |
| `send_arm`, `send_offboard_mode`, `send_land` | One-shot `VehicleCommand`s |

**Why a tick-result struct instead of callbacks back into the shell?**
Because it keeps the dependency one-way (shell → controller → return value).
The controller never holds a pointer to the node, never invokes ROS, and is
trivially copyable. Tests just `EXPECT_TRUE(tick.send_arm)`.

#### Internal state machine

```
                ┌──────────┐  10 heartbeats sent
                │  LANDED  │ ──────────┐
                └──────────┘           │
                  ▲                    ▼
                  │             ┌──────────┐
                  │             │ TAKEOFF  │  publish hover @ TAKEOFF_Z_NED
                  │             └──────────┘
                  │                    │  local_pos.z ≤ -5 + 0.3
                  │                    ▼
                  │             ┌──────────┐
                  │             │  HOVER   │  pos sp or vel sp or hold
                  │             └──────────┘
                  │                    │  on_land_command()
                  │                    ▼
                  │             ┌──────────┐
                  │             │ LANDING  │  PX4 lands autonomously
                  │             └──────────┘
                  │                    │  disarmed=true
                  └────────────────────┘
```

**Why an explicit `TAKEOFF` state instead of letting the user publish a
takeoff setpoint?** Because the PX4 offboard activation sequence requires
*setpoints already streaming* before the mode-switch command arrives.
There is no first-tick where the controller can ask the user "where to?".
So the controller owns a fixed takeoff target (`TAKEOFF_Z_NED = -5.0 m`),
hits it, then transitions to a state that accepts external setpoints.

**Why a separate `LANDING` state?** Once we send `VEHICLE_CMD_NAV_LAND`,
PX4 leaves offboard mode and ignores our setpoints. We stop publishing
trajectory messages (we'd just be wasting bus bandwidth) and only keep the
heartbeat alive so the *next* armed flight can re-enter offboard cleanly.

#### Setpoint priority

In `HOVER`, the order is:

1. **Velocity setpoint** if received within the last `VEL_SP_TIMEOUT_S = 0.5 s`.
2. **Position setpoint** if one was ever received.
3. **Hover hold** — last known position.

**Why velocity wins over position?** Velocity is the more time-sensitive
input. If an operator is steering with a joystick at 20 Hz and also has a
stale position waypoint, the joystick should take over immediately. Position
setpoints are usually waypoints from a planner and tolerate a few hundred ms
of latency; velocity setpoints don't.

**Why the 0.5 s velocity timeout?** A velocity setpoint is a continuous
intention ("keep moving north"). If the publisher dies or hangs, the drone
should stop, not keep flying. 0.5 s is long enough to absorb publisher jitter
at 10–20 Hz, short enough that an operator notices the failsafe.

**Why a "hover hold" field tracked even during active setpoint?** So the
moment the velocity setpoint expires, the hold target is the *current*
position, not where the drone took off from. Otherwise the drone would
fly back to the takeoff point — surprising and dangerous.

#### The 10-heartbeat settle phase (`SETTLE_TICKS`)

PX4 requires that offboard messages already be streaming when you send the
mode-switch command, or it rejects the switch. The controller publishes
heartbeats and hover setpoints for 10 ticks (1 s at 10 Hz) before sending
`VEHICLE_CMD_DO_SET_MODE` and `VEHICLE_CMD_COMPONENT_ARM_DISARM`. After
that, it transitions to `TAKEOFF`. The arm and mode-switch are one-shots:
even if the timer fires a million more times, they are issued exactly once.

**Why a counter and not a wall-clock check?** Because the timer can drift,
miss ticks under load, or fire late after boot. A counter is invariant to
all of that — 10 successful ticks is 10 successful heartbeats, period.

---

### `MissionPlanner`

Owns the per-mission state machine.

```cpp
enum class Mission { ENDURANCE = 0, MAPPING = 1, DROPPING = 2 };
```

**Inputs**:

| Method | When |
|---|---|
| `on_local_position(x, y, z)` | Odometry callback |
| `set_endurance_waypoints(...)` | `/waypoints/endu` callback |
| `set_mapping_waypoints(...)` | `/waypoints/map` callback |
| `on_drop_area(wp)` | `/drop_area` callback |

**Step**: `Tick tick()` — returns the next waypoint to publish and whether
the mission changed.

#### Sequence

```
   ┌────────────┐  desired_laps × full circuit completed
   │ ENDURANCE  │ ──────────────────────────────────┐
   └────────────┘                                   ▼
                                            ┌────────────┐ ◄────┐
                                            │  MAPPING   │      │
                                            └─────┬──────┘      │  drop reached
                                                  │             │
                                                  │ drop_area   │
                                                  │ received +  │
                                                  │ next wp     │
                                                  │ reached     │
                                                  ▼             │
                                            ┌────────────┐      │
                                            │  DROPPING  │ ─────┘
                                            └────────────┘
```

**Why a single acceptance radius `ACCEPT_RADIUS_M = 1.0 m`?** With FW
cruise gone there's only one regime to tune. 1 m is comfortably above GPS
noise (~0.3 m horizontal) and tight enough that the drone visibly stops at
each point in sim. Tune up if your indoor mocap has tighter precision, or
down if your GPS is noisier.

**Why `on_drop_area` only sets `drop_pending_` during `MAPPING`?** The
drop-area signal models an AI detector that only runs while the drone is
flying the mapping circuit. If a stale drop point arrives during
`ENDURANCE`, it would be wrong to interrupt the lap counting. The planner
remembers the waypoint (so the *next* `MAPPING` arrival can divert) but
does not set the pending flag — exactly matching the original behavior.

**Why divert only on the *next* waypoint arrival, not immediately?** Two
reasons: (1) avoids a mid-leg yaw discontinuity, (2) gives the drone a
sane resume-point (`map_resume_idx_` = index right after the waypoint
where we diverted).

---

## ROS shell components

### `offboard_master` node

Wraps `FlightController`. Owns:

**Publishers** (PX4 inbound):
- `/fmu/in/offboard_control_mode` — heartbeat
- `/fmu/in/trajectory_setpoint` — pos/vel setpoint
- `/fmu/in/vehicle_command` — arm/mode/land

**Subscribers**:
- `/fmu/out/vehicle_status` — extracts `disarmed` bool
- `/fmu/out/vehicle_odometry` — current NED position
- `/offboard/cmd/land` — external land command (`std_msgs/Bool`)
- `/offboard/setpoint/position` — external position SP (`PoseStamped`)
- `/offboard/setpoint/velocity` — external velocity SP (`TwistStamped`)

**Timer**: 10 Hz. Calls `fc_.tick()` and publishes what the result says.

**Why a single 10 Hz timer instead of one timer per output?** Simpler to
reason about: every output decision happens at the same point in time with
the same input snapshot. Avoids "what if the heartbeat fired between two
state updates" hazards.

**Why log only on state change** (`logged_state_` field)? At 10 Hz a steady
state would produce 600 identical INFO lines per minute. Logging only the
edges keeps the console useful as a debug timeline.

### `mission_node` node

Wraps `MissionPlanner`. Owns:

**Publisher**: `/offboard/setpoint/position` — feeds into `offboard_master`.
**Publisher**: `/mission_state` (latched, `UInt8`) — for external observers.

**Subscribers** for `/fmu/out/vehicle_odometry`, `/waypoints/endu`,
`/waypoints/map`, `/drop_area`.

The shell converts `PoseStamped` to a flat `Waypoint{x,y,z,yaw}` struct on
ingress and back to `PoseStamped` on egress. This is the entire reason the
planner doesn't depend on geometry_msgs.

**Why a latched mission-state topic?** So a node that subscribes *after*
the mission has started still receives the current state. ROS QoS
`TRANSIENT_LOCAL` is the latch.

### `waypoint_test_node`

Mock waypoint provider for sim/dev. Publishes a hard-coded endurance circuit
and mapping lawnmower at startup; publishes a hard-coded drop area when it
sees `/mission_state == MAPPING`.

**Caveat**: the hard-coded altitudes are `-50 m` (FW cruise legacy). For
MC-only flight, lower these in the file before testing.

---

## QoS reminders

All `/fmu/in/*` and `/fmu/out/*` topics use PX4's required QoS:

```
BestEffort + TransientLocal + KeepLast(1)
```

`/waypoints/*`, `/drop_area`, `/mission_state` use `TRANSIENT_LOCAL` so
late subscribers (the mission node restarting mid-flight, e.g.) immediately
receive the last message.

External setpoint topics (`/offboard/setpoint/*`, `/offboard/cmd/land`) use
default reliable QoS (`depth=10`). The reasoning: these come from human-ish
sources (operator GCS, AI node) where occasional retransmission is fine and
late delivery is *not* — we want every command, even if it adds 50 ms latency.

---

## Coordinate convention

Everything inside this package is **PX4 NED**: x = North, y = East, z = Down.
Altitude is negative. There is no ENU anywhere in `basic_offboard`.

External nodes that work in ENU must convert before publishing to
`/offboard/setpoint/position`. Use `px4_ros_com/frame_transforms.h`.

**Why NED inside, not ENU?** Because the destination (PX4) is NED. Doing
the conversion at the package boundary minimizes the chance of a stray
sign-flip bug deep in the state machine.

---

## Running

### Build

```bash
source /opt/ros/humble/setup.bash
cd /home/porh/px4_dev/main_control
colcon build --packages-select custom_interfaces basic_offboard \
  --cmake-args -DCMAKE_BUILD_TYPE=RELWITHDEBINFO --symlink-install
source install/setup.bash
```

### Run with the mission state machine (sim + dev)

```bash
ros2 launch basic_offboard mission_launch.py            # default 3 laps
ros2 launch basic_offboard mission_launch.py desired_laps:=1
```

This starts `offboard_master` and `mission_node`. You'll need PX4 SITL +
the uXRCE-DDS bridge running separately. The waypoint test node is *not*
auto-launched — start it manually if you want mock waypoints:

```bash
ros2 run basic_offboard waypoint_test_node
```

### Run just the flight controller (no mission node)

```bash
ros2 run basic_offboard offboard_master
```

Then drive the drone manually with topics:

```bash
# Position setpoint (NED, drone climbs to 5 m AGL automatically first)
ros2 topic pub --once /offboard/setpoint/position geometry_msgs/PoseStamped \
  '{pose: {position: {x: 2.0, y: 0.0, z: -5.0}}}'

# Velocity setpoint (NED, must be republished at >= 2 Hz or it expires)
ros2 topic pub --rate 10 /offboard/setpoint/velocity geometry_msgs/TwistStamped \
  '{twist: {linear: {x: 1.0}}}'

# Land
ros2 topic pub --once /offboard/cmd/land std_msgs/Bool '{data: true}'
```

Watch the logs — every state transition prints `State: X -> Y`.

---

## Testing

### Unit tests (no ROS, no PX4)

```bash
colcon test --packages-select basic_offboard
colcon test-result --verbose
```

19 tests run in ~10 ms total. They cover:

- Settle phase publishes hover, no arm.
- Arm + offboard issued exactly once after settle.
- Takeoff completes when altitude is reached.
- Position SP accepted in HOVER.
- Velocity overrides position when fresh.
- Velocity expiry falls back to hover hold.
- Land from HOVER → LANDING; setpoint stops being published.
- Disarmed during LANDING → LANDED.
- Mission: endurance → mapping after lap count.
- Mission: drop area during endurance is ignored.
- Mission: drop area during mapping diverts at next waypoint.
- Mission: drop reached resumes mapping at saved index.

### Direct gdb / debug

```bash
cd build/basic_offboard
gdb --args ./test_flight_controller --gtest_filter=*LandFromHover*
```

You can step through `FlightController::tick()` without any of PX4 running.
This is the single biggest debug-speed win of the architecture.

---

## Extending

### Add a new flight state (e.g. `RETURN_TO_LAUNCH`)

1. Add the enum value in `flight_controller.hpp`.
2. Add a `case` in the transition-decision switch in `tick()`.
3. Add a `case` in the setpoint-selection switch in `tick()`.
4. Add a string in `state_str()`.
5. Write a gtest that drives the controller into the new state.

You should not need to touch `offboard_master.cpp` unless the new state
requires a new ROS publisher.

### Add a new mission

Same pattern in `mission_planner.{hpp,cpp}`. The shell stays untouched
unless you need a new subscriber.

### Replace mission_node with your own external planner

`offboard_master` accepts external position/velocity/land commands directly,
so any other node can drive the drone by publishing to
`/offboard/setpoint/*`. The mission_node is just one such consumer.

### Port to Python

Rewrite only the two shell files in `rclpy`. The pure-logic classes become
plain Python classes (the `tick()` shape maps to a `@dataclass` return).
The tests can be reused by porting them to `pytest`.

---

## Design decisions, summarized

| Choice | Why |
|---|---|
| Two layers, ROS at the boundary | Testability, debug speed, language-portability |
| C++ instead of Python | Existing infra; 10 Hz jitter; pure layer is tiny |
| Tick returns a struct | One-way dependency; trivially testable |
| Counter for settle phase (not wall-clock) | Robust to timer drift / late boot |
| 10-tick settle | PX4 requires streaming setpoints *before* mode switch |
| Velocity > position priority | Velocity is the more time-sensitive intent |
| 0.5 s velocity timeout | Absorbs publisher jitter; fails safe if publisher dies |
| Hover-hold tracks current pos | Velocity expiry doesn't fly drone back to origin |
| Explicit `LANDING` state | PX4 owns landing once `NAV_LAND` is sent; we just shut up |
| Single acceptance radius (1 m) | Only one regime to tune in MC-only |
| Drop-area only "pending" during MAPPING | Models an AI detector tied to mapping flight |
| Divert on *next* waypoint, not immediate | Avoids mid-leg yaw flip; gives sane resume point |
| Latched mission-state topic | Late subscribers see the current state |
| NED throughout the package | Destination is NED; convert at the package boundary |
| Log on state change only | At 10 Hz, steady-state logging is noise |
