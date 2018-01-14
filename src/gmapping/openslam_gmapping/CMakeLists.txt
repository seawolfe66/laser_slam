cmake_minimum_required(VERSION 2.8)
project(openslam_gmapping)

find_package(catkin)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gridfastslam scanmatcher sensor_base sensor_range sensor_odometry utils
)

#find_package(OpenMP)
#if(OPENMP_FOUND)
#  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}" )
#  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}" )
#endif()

include_directories(include)

add_subdirectory(gridfastslam)
add_subdirectory(scanmatcher)
add_subdirectory(sensor)
add_subdirectory(utils)

if(0)
  add_subdirectory(gui)
endif()

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
                 PATTERN "*.hxx"
  PATTERN ".svn" EXCLUDE
)
