cmake_minimum_required(VERSION 3.0.2)
project(lidar_euclidean_cluster_detector)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  geometry_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  lidar_pipeline_monitor
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(${PROJECT_NAME}_node src/lidar_euclidean_cluster_detector.cpp)

## Add cmake target dependencies of the executable
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} lidar_pipeline_monitor_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# 安装可执行文件
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装 Launch 文件文件夹
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)