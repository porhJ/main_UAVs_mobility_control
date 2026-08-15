cmake_minimum_required(VERSION 3.16)
project(main_control_core VERSION 0.1.0 LANGUAGES CXX)

include(CTest)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

add_library(main_control_model INTERFACE)
add_library(main_control::model ALIAS main_control_model)
set_target_properties(main_control_model PROPERTIES EXPORT_NAME model)
target_compile_features(main_control_model INTERFACE cxx_std_17)
target_include_directories(main_control_model INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

function(main_control_add_core_library target component source)
  add_library(${target} STATIC ${source})
  add_library(main_control::${component} ALIAS ${target})
  set_target_properties(${target} PROPERTIES EXPORT_NAME ${component})
  target_compile_features(${target} PUBLIC cxx_std_17)
  target_compile_options(${target} PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>)
  target_include_directories(${target} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
  target_link_libraries(${target} PUBLIC main_control::model)
endfunction()

main_control_add_core_library(
  main_control_flight flight src/flight/flight_controller.cpp)
main_control_add_core_library(
  main_control_mission mission src/mission/mission_planner.cpp)
main_control_add_core_library(
  main_control_landing landing src/landing/precision_lander.cpp)

install(
  TARGETS
    main_control_model
    main_control_flight
    main_control_mission
    main_control_landing
  EXPORT main_control_coreTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

configure_package_config_file(
  cmake/main_control_coreConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/main_control_coreConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/main_control_core)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/main_control_coreConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion)

install(
  EXPORT main_control_coreTargets
  FILE main_control_coreTargets.cmake
  NAMESPACE main_control::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/main_control_core)

install(
  FILES
    ${CMAKE_CURRENT_BINARY_DIR}/main_control_coreConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/main_control_coreConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/main_control_core)

if(BUILD_TESTING)
  find_package(GTest REQUIRED)

  function(main_control_add_core_test target source library)
    add_executable(${target} ${source})
    target_compile_features(${target} PRIVATE cxx_std_17)
    target_link_libraries(${target} PRIVATE ${library} GTest::gtest_main)
    add_test(NAME ${target} COMMAND ${target})
  endfunction()

  main_control_add_core_test(
    test_flight_controller
    test/flight/test_flight_controller.cpp
    main_control::flight)
  main_control_add_core_test(
    test_mission_planner
    test/mission/test_mission_planner.cpp
    main_control::mission)
  main_control_add_core_test(
    test_precision_lander
    test/landing/test_precision_lander.cpp
    main_control::landing)

  add_test(
    NAME core_no_ros_dependencies
    COMMAND ${CMAKE_COMMAND}
      -DCORE_ROOT=${CMAKE_CURRENT_SOURCE_DIR}
      -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/check_no_ros_includes.cmake)
endif()
