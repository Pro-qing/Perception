cmake_minimum_required(VERSION 3.0.2)
project(lidar_input_downsample_noground)

## Compile as C++14
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  message_filters
  pcl_conversions
  pcl_ros
  laser_geometry
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  visualization_msgs
  autoware_msgs
  lidar_pipeline_monitor
)

## 指定 PCL 依赖
find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_lib
  CATKIN_DEPENDS
    roscpp sensor_msgs message_filters pcl_conversions pcl_ros
    laser_geometry tf2_ros tf2_geometry_msgs geometry_msgs
    visualization_msgs autoware_msgs lidar_pipeline_monitor
)

## 包含头文件目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## ---- 编译为库 (解耦的三个模块) ----
add_library(${PROJECT_NAME}_lib
  src/sensor_input.cpp
  src/downsample.cpp
  src/no_ground.cpp
)

add_dependencies(${PROJECT_NAME}_lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  lidar_pipeline_monitor_generate_messages_cpp
)

target_link_libraries(${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

## ---- 编译可执行文件 (整合节点) ----
add_executable(lidar_pipeline_node src/lidar_pipeline_node.cpp)

add_dependencies(lidar_pipeline_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  lidar_pipeline_monitor_generate_messages_cpp
)

target_link_libraries(lidar_pipeline_node
  ${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

## ---- 安装 ----
install(TARGETS lidar_pipeline_node ${PROJECT_NAME}_lib
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)