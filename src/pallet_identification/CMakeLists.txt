cmake_minimum_required(VERSION 3.0.2)
project(pallet_identification)

# 设置C++标准 (C++17 支持 std::atomic 等现代特性)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖项 (补充了 geometry_msgs 和 tf2)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  tf2
  autoware_can_msgs
  autoware_msgs
  autoware_remove_msgs
)

# 查找 yaml-cpp
find_package(yaml-cpp REQUIRED)
set(yaml-cpp_INCLUDE_DIRS ${YAML_CPP_INCLUDE_DIR})
set(yaml-cpp_LIBRARIES ${YAML_CPP_LIBRARIES})

# 查找多线程支持库 (用于 std::thread 和 inotify 后台监听)
find_package(Threads REQUIRED)

# 配置catkin项目
catkin_package(
   CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs tf2 autoware_can_msgs autoware_msgs autoware_remove_msgs
   DEPENDS yaml-cpp
)

# 头文件包含
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${yaml-cpp_INCLUDE_DIRS}
)

add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)  # 保持旧行为

# ================= 节点 1：safe_tip_lift =================
add_executable(safe_tip_lift
  src/safe_tip_lift.cpp
)
target_link_libraries(safe_tip_lift
  ${yaml-cpp_LIBRARIES}  
  ${catkin_LIBRARIES}
  Threads::Threads       # 链接多线程库
)

# ================= 节点 2：pallet_identification =================
add_executable(pallet_identification
  src/pallet_identification.cpp
)
target_link_libraries(pallet_identification
  ${yaml-cpp_LIBRARIES}  # 新增：现在这个节点也需要 yaml-cpp 了
  ${catkin_LIBRARIES}
  Threads::Threads       # 链接多线程库
)

# 安装头文件
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  # FILES_MATCHING PATTERN "*.h"
  # PATTERN ".svn" EXCLUDE
)

# 安装可执行文件
install(TARGETS
  pallet_identification
  safe_tip_lift
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
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