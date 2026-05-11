# SoftwareStatus.msg
std_msgs/Header header

# --- 定位 ---
geometry_msgs/Pose loc_pose
int8 loc_floor
string loc_method    #定位方式
uint8 loc_state  #定位状态 0:OK 1:DEGRADED 2:LOST

# 感知与避障
float32 obst_path_dis               # 路线上障碍物距离 (m)
uint8 obst_safety_type               # 安全类型: 0:正常 1:急停 2:减速
string obst_safety_detail               # 安全类型: 0:正常 1:急停 2:减速

# --- 导航 ---
uint8 nav_status       # 0:NONE 1:HAS 2:TRACKING 3:ARRIVED
int32 nav_maxindex
int32 nav_curreindex
int32 nav_maxdis
int32 nav_curredis

# --- 控制 ---
int8 ctrl_state       # 0:未到点 1:到点 -1:无控制
int8 ctrl_dire       # 0:未到点 1:到点 -1:无控制
