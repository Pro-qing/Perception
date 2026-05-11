# VehicleBasicState.msg
# 车辆基础状态

std_msgs/Header header

# 控制状态
int8 mode                  # 0:待机, 1:自动, 2:手动, 3:急停
int8 state                  #车体状态
float32 speed               # 速度 m/s
float32 steering_angle      # 转向角 rad
float32 acceleration        # 加速度 m/s²
float32 deceleration        # 减速度 m/s²
uint8 gear                  # 0:空档, 1:前进, 2:后退

# 货叉状态
int8 fork_state            # 0:收回, 1:伸出
float32 fork_height         # 货叉高度 m
float32 load_weight         # 载重 kg

# 传感器状态
bool is_in_position         # 到位传感器
bool is_collision           # 防撞条触发
bool is_emergency_stop      # 急停激活

# 里程时间
float32 total_mileage       # 总里程 m
float32 operation_hours     # 运行时间 h