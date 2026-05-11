cmake_minimum_required(VERSION 2.8.3)
project(autoware_remove_msgs)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  rosgraph_msgs
  std_msgs
)

add_message_files(
  FILES
  Header.msg
  CoorDinate.msg
  TriggerType.msg
  Sites.msg
  SiteStatus.msg
  VehicleCmd.msg
  TaskCmd.msg
  TriggerCmd.msg
  VehicleModeCmd.msg
  BaseInfo.msg
  TaskInfo.msg
  VehicleInfo.msg
  HardwareInfo.msg

  fatigue_detect.msg
  traffic_icon_data.msg
  vehicle_pedestrian_detect.msg
  milli_data.msg
  traffic_icon_detect.msg
  milli_detect.msg
  vehicle_pedestrian_data.msg
  Gps.msg
  BatteryInfo.msg
  State.msg
)

generate_messages(DEPENDENCIES
  geometry_msgs
  rosgraph_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
    rosgraph_msgs
    std_msgs
)
