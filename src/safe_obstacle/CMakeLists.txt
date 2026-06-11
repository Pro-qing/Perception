cmake_minimum_required(VERSION 3.0.2)
project(safe_obstacle)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  sensor_msgs
  # tf
  # tf2
  pcl_ros
  visualization_msgs
  autoware_can_msgs
  autoware_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pallet_tcp
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)


add_executable(safe_obstacle_node src/safe_obstacle.cpp)


add_dependencies(safe_obstacle_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(safe_obstacle_node
  ${catkin_LIBRARIES}
)


#############
## Install ##
#############

#3 头文件,                   (PATH: install/include/slam_karto/)
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

#3 拷贝 launch目录           (PATH: install/share/slam_karto/launch/)
install(DIRECTORY 
    launch params                  #注意,不能使用 launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS 
  safe_obstacle_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

