cmake_minimum_required(VERSION 3.0.2)
project(lidar_no_charge)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
  autoware_msgs
  message_generation
  lidar_pipeline_monitor
  pcl_conversions
)

## System dependencies are found with CMake's conventions
find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS pcl_ros roscpp std_msgs sensor_msgs geometry_msgs visualization_msgs tf2_ros tf2_geometry_msgs autoware_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(${PROJECT_NAME}_node src/lidar_no_charge.cpp)

## Add cmake target dependencies of the executable
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} lidar_pipeline_monitor_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装 Launch 文件文件夹
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)