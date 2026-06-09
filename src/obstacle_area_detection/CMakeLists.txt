cmake_minimum_required(VERSION 3.0.2)
project(obstacle_area_detection VERSION 1.0.0)

set(PCL_DIR "/usr/lib/x86_64-linux-gnu/cmake/pcl")
# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  autoware_msgs
  pcl_ros
  pcl_conversions
  tf
  visualization_msgs
  geometry_msgs
)
find_package(PCL 1.8 REQUIRED)
message(STATUS "PCL_VERSION = ${PCL_VERSION}")

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


# 查找yaml-cpp
find_package(yaml-cpp REQUIRED)
set(yaml-cpp_INCLUDE_DIRS ${YAML_CPP_INCLUDE_DIR})
set(yaml-cpp_LIBRARIES ${YAML_CPP_LIBRARIES})

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs pcl_ros tf visualization_msgs autoware_msgs geometry_msgs
  DEPENDS yaml-cpp
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${yaml-cpp_INCLUDE_DIRS}
)

add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)  # 保持旧行为


# ========== 辅助库: 不规则多边形滤波器 ==========
add_library(IrregularPolygonFilter
  src/IrregularPolygonFilter.cpp
)
target_link_libraries(IrregularPolygonFilter
  ${catkin_LIBRARIES}
)

# ========== 可执行文件: 库区障碍物检测节点(拆分为多个模块源文件) ==========
add_executable(obstacle_area_detection
  src/obstacle_area_detection.cpp        # 主入口 + 构造函数 + 核心回调
  src/pointcloud_preprocessing.cpp       # 点云预处理模块
  src/area_detection.cpp                 # 区域检测模块
  src/state_callbacks.cpp                # 状态回调模块
  src/visualization.cpp                  # 可视化模块
)
target_link_libraries(obstacle_area_detection
  ${catkin_LIBRARIES}
  IrregularPolygonFilter
)

install(TARGETS
  obstacle_area_detection IrregularPolygonFilter
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


# 安装启动文件
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# 安装配置文件
install(DIRECTORY
  params/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/params
)