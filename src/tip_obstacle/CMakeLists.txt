cmake_minimum_required(VERSION 3.0.2)
project(tip_obstacle)

## 必须使用 c++14，否则 ROS Noetic 的基础头文件会报错
add_compile_options(-std=c++14)

# ==========================================
# 1. 强制寻找并配置 PCL 1.8
# ==========================================
find_package(PCL 1.8 REQUIRED)
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

# ==========================================
# 2. 寻找 ROS 包
# ==========================================
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  message_filters
  pcl_conversions
  pcl_msgs
  # pcl_ros
  laser_geometry
  autoware_remove_msgs
  tf
  visualization_msgs
)

# 寻找 yaml-cpp 
find_package(yaml-cpp REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
)

###########
## Build ##
###########

include_directories(
  include 
  ${PCL_INCLUDE_DIRS}     # 优先包含 PCL 1.8 的头文件
  ${catkin_INCLUDE_DIRS}
)

add_executable(tip_obstacle_node src/tip_obstacle.cpp)

add_dependencies(tip_obstacle_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 将 PCL_LIBRARIES 链接到节点
target_link_libraries(tip_obstacle_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  yaml-cpp
  pthread
)

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY 
    launch                  
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS 
  tip_obstacle_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)