cmake_minimum_required(VERSION 3.0.2)
project(lidar_input_downsample_noground)

# 1. 设置 C++ 标准为 14 (PCL 1.8 建议 C++11 或 C++14)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 2. 【关键修改】开启 PCL_NO_PRECOMPILE 宏，防止 PointXYZI 报链接错误
add_definitions(-DPCL_NO_PRECOMPILE)

# 3. 指定寻找 PCL 1.8
find_package(PCL EXACT REQUIRED COMPONENTS common io filters segmentation sample_consensus kdtree search)

# 包含 PCL 的头文件和库
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  message_filters
  pcl_conversions
  laser_geometry
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  visualization_msgs
  autoware_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_lib
  CATKIN_DEPENDS
    roscpp sensor_msgs message_filters pcl_conversions
    laser_geometry tf2_ros tf2_geometry_msgs geometry_msgs
    visualization_msgs autoware_msgs
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

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
  scripts/config_editor_gui.py
  scripts/param_path.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
