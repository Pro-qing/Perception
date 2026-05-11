# Attributes
uint8 START_SITE = 0        # 终点站
uint8 DOCK_SITE = 1         # 停靠站
uint8 GARAGE__SITE = 2      # 充电站

# trigger
uint8 TIME_TRI = 0
uint8 BUTTON = 1
uint8 INSTRUCTION = 2

geometry_msgs/Pose pose

string site_name
uint8 Attributes
uint8 trigger_1
uint8 trigger_2
uint32 time