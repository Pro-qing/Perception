cmake_minimum_required(VERSION 3.0.2)
project(lidar_map_filter)

## Compile as C++14
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
  lidar_pipeline_monitor
)

## Find PCL
find_package(PCL REQUIRED COMPONENTS common io kdtree)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lidar_map_filter
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros pcl_conversions lidar_pipeline_monitor
  DEPENDS PCL
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/lidar_map_filter.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

## Declare a C++ executable
add_executable(${PROJECT_NAME}_node src/lidar_map_filter_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

## Install
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch params
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)