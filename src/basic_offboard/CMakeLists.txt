cmake_minimum_required(VERSION 3.8)
project(basic_offboard)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)

include_directories(include)

# --- Pure-logic libraries (no rclcpp, unit-testable) ---

add_library(flight_controller STATIC src/flight_controller.cpp)
target_include_directories(flight_controller PUBLIC include)

add_library(mission_planner STATIC src/mission_planner.cpp)
target_include_directories(mission_planner PUBLIC include)

# --- ROS nodes ---

add_executable(offboard_master src/offboard_master.cpp)
ament_target_dependencies(offboard_master rclcpp px4_msgs geometry_msgs std_msgs)
target_link_libraries(offboard_master flight_controller)
install(TARGETS offboard_master DESTINATION lib/${PROJECT_NAME})

add_executable(mission_node src/mission_node.cpp)
ament_target_dependencies(mission_node rclcpp px4_msgs geometry_msgs std_msgs custom_interfaces)
target_link_libraries(mission_node mission_planner)
install(TARGETS mission_node DESTINATION lib/${PROJECT_NAME})

add_executable(waypoint_test_node src/waypoint_test_node.cpp)
ament_target_dependencies(waypoint_test_node rclcpp geometry_msgs std_msgs custom_interfaces)
install(TARGETS waypoint_test_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# --- Tests ---

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip stylistic linters — they fight against the project's chosen formatting.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_lint_cmake_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_flight_controller test/test_flight_controller.cpp)
  target_link_libraries(test_flight_controller flight_controller)

  ament_add_gtest(test_mission_planner test/test_mission_planner.cpp)
  target_link_libraries(test_mission_planner mission_planner)
endif()

ament_package()
