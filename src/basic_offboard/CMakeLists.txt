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

add_executable(offboard_master src/offboard_master.cpp)
ament_target_dependencies(offboard_master
  rclcpp
  px4_msgs
  geometry_msgs
  std_msgs
  custom_interfaces
)
install(TARGETS offboard_master DESTINATION lib/${PROJECT_NAME})

add_executable(mission_node src/mission_node.cpp)
ament_target_dependencies(mission_node
  rclcpp
  px4_msgs
  geometry_msgs
  custom_interfaces
)
install(TARGETS mission_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
