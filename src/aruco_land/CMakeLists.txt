cmake_minimum_required(VERSION 3.8)
project(aruco_land)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

# --- Pure-logic library (no rclcpp/px4_msgs/Eigen, unit-testable) ---

add_library(precision_lander STATIC src/precision_lander.cpp)
target_include_directories(precision_lander PUBLIC include)

# --- ROS node (shell) ---

add_executable(aruco_land_node src/aruco_land_node.cpp)
ament_target_dependencies(aruco_land_node
  rclcpp px4_msgs geometry_msgs std_msgs custom_interfaces)
target_link_libraries(aruco_land_node precision_lander Eigen3::Eigen)
install(TARGETS aruco_land_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

# --- Tests ---

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip stylistic linters — match basic_offboard's chosen formatting policy.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_lint_cmake_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_precision_lander test/test_precision_lander.cpp)
  target_link_libraries(test_precision_lander precision_lander)
endif()

ament_package()
