Header header

uint8 LEISURE = 0        # 空闲
uint8 QUEST = 1         # 任务
uint8 BUSY = 2      # 忙碌
uint8 CHARGING = 3  #充电

string task_id
uint8 state
uint8 type
uint32 safe_steps #安全下标
string control_info  #管制信息
uint32 key_points
SiteStatus site
geometry_msgs/Pose[] points