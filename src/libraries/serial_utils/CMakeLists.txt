cmake_minimum_required(VERSION 2.4.6)
project(serial_utils)

find_package(catkin REQUIRED)

option(SERIAL_UTILS_BUILD_WIHOUT_ROS "Build without ROS?" OFF)

catkin_package(
        LIBRARIES ${PROJECT_NAME}
        INCLUDE_DIRS include
    )

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )


if(${catkin_FOUND} AND NOT SERIAL_UTILS_BUILD_WIHOUT_ROS)
  # Build with ROS
  include(serial_utils_ros.cmake)
  # message("Building with ROS")
  build_serial_utils()
else()
  # Build normally
  include(serial_utils.cmake)
  message("Building stand alone")
  build_serial_utils()
endif()
