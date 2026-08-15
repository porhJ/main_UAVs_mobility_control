cmake_minimum_required(VERSION 3.16)
project(main_control_apps VERSION 0.1.0 LANGUAGES CXX)

include(GNUInstallDirs)

find_package(main_control_core CONFIG REQUIRED)

# Add standalone executables here. Each executable should link only the core
# targets it needs, for example:
#
# add_executable(mission_replay src/mission_replay.cpp)
# target_link_libraries(mission_replay PRIVATE main_control::mission)
# install(TARGETS mission_replay RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

install(FILES README.md DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})
