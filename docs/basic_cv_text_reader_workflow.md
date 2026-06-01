# basic_cv Text Reader — Build & Debug Workflow

## What the Package Does

Two ROS2 nodes that form a text-detection pipeline:

| Node | Role |
|---|---|
| `video_publisher` | Reads a video file and publishes frames to `/camera/image_raw` |
| `text_reader_node` | Subscribes to `/camera/image_raw`, runs EasyOCR every 5th frame, logs detected text |

---

## Issues Found and Fixed

### 1. Frame skip logic was inverted
**Original:** YOLO ran on every frame; OCR ran only on every 5th frame — but the `return` came *after* the expensive YOLO call.

**Fix:** Move the frame count check to the top of the callback before any inference.

```python
def image_callback(self, msg):
    self.frame_count += 1
    if self.frame_count % 5 != 0:  # skip early — no work done
        return
    ...
```

### 2. Video path was relative
**Original:** `VideoPublisher('../test_media/text_reader_test.mov')` — breaks depending on where `ros2 run` is invoked from.

**Fix:** Declare a ROS2 parameter with an absolute default path.

```python
self.declare_parameter('video_path', '/home/porh/px4_dev/main_control/test_media/text_reader_test.mov')
video_path = self.get_parameter('video_path').get_parameter_value().string_value
```

Override at runtime with:
```bash
ros2 run basic_cv video_publisher --ros-args -p video_path:=/path/to/other.mp4
```

### 3. YOLO was wrong model for text detection
**Original:** `yolov8n.pt` is a general 80-class object detector (COCO), not a text detector. Using it as a pre-filter before OCR served no purpose.

**Fix:** Removed YOLO entirely. Pass the full frame directly to EasyOCR.

---

## Dependency Conflict: NumPy vs cv_bridge

### Root Cause

| Library | NumPy requirement |
|---|---|
| `cv_bridge` (ROS2 Humble, compiled from apt) | `< 2.0` (compiled against NumPy 1.x ABI) |
| `easyocr` → `torch` → `torchvision` | `>= 2.0` |

Both cannot coexist in the same Python interpreter. Attempting to import `cv_bridge` with NumPy 2.x gives:

```
AttributeError: _ARRAY_API not found
```

### Resolution

Removed `cv_bridge` from both nodes entirely. ROS2 `sensor_msgs/Image` messages carry raw pixel bytes, so the conversion can be done with plain numpy — no compiled bridge needed.

**Publishing (cv2 frame → ROS Image):**
```python
msg = Image()
msg.height = frame.shape[0]
msg.width  = frame.shape[1]
msg.encoding = 'bgr8'
msg.is_bigendian = False
msg.step = frame.shape[1] * 3
msg.data = frame.tobytes()
```

**Subscribing (ROS Image → numpy frame):**
```python
frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
```

### Secondary Issue: Entry Point Uses Wrong Python

`ros2 run` entry point scripts use `#!/usr/bin/python3`. If `pip install` targets a virtual environment (e.g. `~/ros2_env`) instead of the system Python, the entry point still picks up the wrong package versions.

**Diagnosis:**
```bash
head -1 install/basic_cv/lib/basic_cv/video_publisher  # check shebang
/usr/bin/python3 -c "import numpy; print(numpy.__version__)"  # check what it sees
```

**Fix:** Run pip install via the exact interpreter the entry point uses:
```bash
/usr/bin/python3 -m pip install 'package'
```

---

## Build

```bash
source /opt/ros/humble/setup.bash
cd /home/porh/px4_dev/main_control
colcon build --packages-select basic_cv --cmake-args -DCMAKE_BUILD_TYPE=RELWITHDEBINFO --symlink-install
source install/setup.bash
```

## Run

Terminal 1 — publish video:
```bash
ros2 run basic_cv video_publisher
```

Terminal 2 — read text:
```bash
ros2 run basic_cv text_reader_node
```

Expected output in Terminal 2 (after EasyOCR model download on first run):
```
[INFO] [text_reader_node]: Detected: ['Sat 16 May', 'Note', ...]
```

**Note:** EasyOCR downloads its detection and recognition models (~100 MB) on first run. Subsequent runs use the cached models at `~/.EasyOCR/`.

---

## Verify the Topic

```bash
# Confirm frames are being published
ros2 topic hz /camera/image_raw

# Inspect a single frame message
ros2 topic echo /camera/image_raw --once
```
