if(NOT DEFINED CORE_ROOT)
  message(FATAL_ERROR "CORE_ROOT is required")
endif()

file(GLOB_RECURSE CORE_FILES
  "${CORE_ROOT}/include/*.hpp"
  "${CORE_ROOT}/include/*.h"
  "${CORE_ROOT}/src/*.cpp"
  "${CORE_ROOT}/src/*.cc")

set(FORBIDDEN_INCLUDES
  "#include <rclcpp/"
  "#include <px4_msgs/"
  "#include <geometry_msgs/"
  "#include <std_msgs/"
  "#include <sensor_msgs/"
  "#include <custom_interfaces/")

foreach(FILE_PATH IN LISTS CORE_FILES)
  file(READ "${FILE_PATH}" CONTENTS)
  foreach(FORBIDDEN IN LISTS FORBIDDEN_INCLUDES)
    string(FIND "${CONTENTS}" "${FORBIDDEN}" POSITION)
    if(NOT POSITION EQUAL -1)
      message(FATAL_ERROR
        "ROS transport dependency '${FORBIDDEN}' found in ${FILE_PATH}")
    endif()
  endforeach()
endforeach()
