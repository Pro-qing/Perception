cmake_minimum_required(VERSION 3.0.2)
project(lidar_sensor_input)

## 寻找依赖包
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
  autoware_msgs
  lidar_pipeline_monitor
)

## 指定 PCL 依赖
find_package(PCL REQUIRED)

catkin_package()

## 包含头文件目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## ---- 1. 编译可执行文件 ----
# 确保文件路径是 src/lidar_sensor_input.cpp
add_executable(sensor_input_node src/lidar_sensor_input.cpp)

# 第一个参数写 sensor_input_node
add_dependencies(sensor_input_node ${catkin_EXPORTED_TARGETS} lidar_pipeline_monitor_generate_messages_cpp)

target_link_libraries(sensor_input_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
## ---- 2. 注册并安装 Launch 和 Config 文件 (Colcon 关键步骤) ----
# 安装可执行文件
install(TARGETS sensor_input_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装 Launch 文件文件夹
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

