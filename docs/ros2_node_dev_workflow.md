# ROS2 Node Development & Test Workflow

General workflow for writing, building, and testing ROS2 nodes in this workspace.

---

## 1. Environment Setup

Always source both ROS2 and the workspace before any build or run command:

```bash
source /opt/ros/humble/setup.bash
source /home/porh/px4_dev/main_control/install/setup.bash
```

Add both to `~/.bashrc` to avoid repeating this every session:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source /home/porh/px4_dev/main_control/install/setup.bash" >> ~/.bashrc
```

---

## 2. Build

### Full workspace
```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RELWITHDEBINFO --symlink-install
```

### Single package (faster iteration)
```bash
colcon build --packages-select <package_name> --cmake-args -DCMAKE_BUILD_TYPE=RELWITHDEBINFO --symlink-install
```

`--symlink-install` means Python node edits take effect immediately without rebuilding. C++ nodes still need a rebuild after source changes.

### After build
```bash
source install/setup.bash
```

---

## 3. Run Nodes

### Direct run
```bash
ros2 run <package> <node_executable>
```

### With parameters
```bash
ros2 run <package> <node_executable> --ros-args -p param_name:=value
```

### With a launch file
```bash
ros2 launch <package> <launch_file>.yaml
```

---

## 4. Inspect Topics at Runtime

```bash
# List all active topics
ros2 topic list

# Check publish rate
ros2 topic hz /topic/name

# Print messages (Ctrl+C to stop)
ros2 topic echo /topic/name

# Print one message only
ros2 topic echo /topic/name --once

# Check topic type
ros2 topic info /topic/name
```

---

## 5. Node Introspection

```bash
# List running nodes
ros2 node list

# Show node's pubs, subs, services
ros2 node info /node_name
```

---

## 6. Logging

### In C++ nodes
```cpp
RCLCPP_INFO(this->get_logger(), "Message: %s", value.c_str());
RCLCPP_WARN(this->get_logger(), "Warning");
RCLCPP_ERROR(this->get_logger(), "Error");
```

### In Python nodes
```python
self.get_logger().info('Message')
self.get_logger().warn('Warning')
self.get_logger().error('Error')
```

### Adjust log level at runtime
```bash
ros2 run <package> <node> --ros-args --log-level debug
```

---

## 7. PX4 / Offboard-Specific Requirements

### QoS profile (required for all PX4 pub/sub)
```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)
```

### Timestamps (microseconds)
```python
msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
```
```cpp
msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
```

### Offboard heartbeat
`/fmu/in/offboard_control_mode` must be published at **≥ 2 Hz** or PX4 exits offboard mode. Publish at 10 Hz to be safe.

### Coordinate frame
PX4 uses **NED** (North-East-Down). Altitude at 5 m = `z = -5.0`.

---

## 8. Common Python Dependency Issues

### Check which Python the entry point uses
`ros2 run` entry point scripts use `#!/usr/bin/python3`. Packages installed into a virtual environment may not be visible to it.

```bash
# Check shebang
head -1 install/<pkg>/lib/<pkg>/<node>

# Check what /usr/bin/python3 sees
/usr/bin/python3 -c "import <module>; print(<module>.__version__)"
```

### Install for the correct interpreter
```bash
/usr/bin/python3 -m pip install <package>
```

### NumPy ABI conflicts
ROS2 Humble's `cv_bridge` was compiled against NumPy 1.x. Libraries like `torch`, `easyocr`, and new `opencv-python` require NumPy ≥ 2.

**Workaround:** Avoid `cv_bridge` when using those libraries. Convert `sensor_msgs/Image` manually:

```python
# Publish: cv2 frame → ROS Image
from sensor_msgs.msg import Image
msg = Image()
msg.height = frame.shape[0]
msg.width  = frame.shape[1]
msg.encoding = 'bgr8'
msg.is_bigendian = False
msg.step = frame.shape[1] * 3
msg.data = frame.tobytes()

# Subscribe: ROS Image → numpy
import numpy as np
frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
```

---

## 9. Debugging Checklist

| Symptom | Check |
|---|---|
| Node exits immediately | Check for import errors: run the script directly with `/usr/bin/python3 src/<pkg>/<pkg>/<node>.py` |
| No messages on topic | `ros2 topic hz /topic` — confirms publisher is alive; check QoS mismatch |
| PX4 exits offboard mode | Heartbeat rate dropped below 2 Hz; increase timer frequency |
| Wrong positions | Check NED vs ENU — altitude should be negative for PX4 |
| `ModuleNotFoundError` at runtime | Package installed in wrong Python — see §8 |
| `_ARRAY_API not found` | NumPy version mismatch — see §8 |
| `No module named 'cv_bridge'` | ROS2 not sourced, or using venv python instead of `/usr/bin/python3` |

---

## 10. Typical Iteration Cycle

```
Edit source
    ↓
colcon build --packages-select <pkg> --symlink-install   (skip for Python edits)
    ↓
source install/setup.bash
    ↓
ros2 run <pkg> <node>
    ↓
ros2 topic echo /output_topic   (verify output in another terminal)
    ↓
repeat
```
