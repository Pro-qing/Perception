# 安全命令消息
std_msgs/Header header
uint32 seq
time stamp
string frame_id

# 安全状态类型
uint8 SAFE_NORMAL = 0    # 正常
uint8 SAFE_EMERGENCY = 1 # 急停
uint8 SAFE_SLOWDOWN = 2  # 减速

uint8 safety_type        # 安全类型: 0-正常,1-急停,2-减速
float32 target_speed     # 目标速度(km/h)
float32 twist_speed     # 下发速度(km/h)

# 详细错误信息
string[] error_messages  # 错误消息列表
string state_summary     # 状态摘要