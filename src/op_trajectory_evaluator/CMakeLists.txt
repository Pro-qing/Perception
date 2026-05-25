cmake_minimum_required(VERSION 2.8.3)
project(op_trajectory_evaluator)

find_package(autoware_build_flags REQUIRED)

find_package(PCL REQUIRED COMPONENTS common)

find_package(
  catkin REQUIRED COMPONENTS
    autoware_can_msgs
    autoware_msgs
    geometry_msgs
    nav_msgs
    op_planner
    op_ros_helpers
    op_utility
    pcl_conversions
    pcl_ros
    roscpp
    std_msgs
    tf
    visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(
  op_trajectory_evaluator
  nodes/op_trajectory_evaluator.cpp
  nodes/op_trajectory_evaluator_core.cpp
)

target_link_libraries(op_trajectory_evaluator ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_dependencies(
  op_trajectory_evaluator
  ${catkin_EXPORTED_TARGETS}
)

install(
  TARGETS
    op_trajectory_evaluator
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)