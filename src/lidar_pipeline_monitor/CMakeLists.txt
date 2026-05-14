cmake_minimum_required(VERSION 3.0.2)
project(lidar_pipeline_monitor)

# 注意：这里去掉了 message_runtime
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
)

add_message_files(
  FILES
  PipelineMetrics.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# 安装 Python 脚本到系统中，这样 rosrun 才能在 install 空间找到它
catkin_install_python(PROGRAMS
  scripts/monitor_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)