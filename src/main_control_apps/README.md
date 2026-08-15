# main_control_apps

This package is the home for standalone C++ executables that use
`main_control_core` without ROS 2. Keep reusable behavior in a core library and
keep each executable limited to process setup, I/O, and orchestration.

New executables should link only the targets they use:

```cmake
add_executable(mission_replay src/mission_replay.cpp)
target_link_libraries(mission_replay PRIVATE main_control::mission)
install(TARGETS mission_replay RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
```
