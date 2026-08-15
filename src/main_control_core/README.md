# main_control_core

Standalone C++17 libraries for flight control, mission planning, and precision
landing. This package intentionally has no ROS 2, PX4 message, or transport
dependency.

Exported CMake targets:

- `main_control::model`
- `main_control::flight`
- `main_control::mission`
- `main_control::landing`

Build and test without ROS:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build
ctest --test-dir build --output-on-failure
cmake --install build --prefix /path/to/prefix
```
