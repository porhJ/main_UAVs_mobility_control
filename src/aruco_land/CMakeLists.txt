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
find_package(main_control_core CONFIG REQUIRED)

# --- ROS node (shell) ---

add_executable(aruco_land_node src/aruco_land_node.cpp)
ament_target_dependencies(aruco_land_node
  rclcpp px4_msgs geometry_msgs std_msgs custom_interfaces)
target_link_libraries(aruco_land_node main_control::landing Eigen3::Eigen)
install(TARGETS aruco_land_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
