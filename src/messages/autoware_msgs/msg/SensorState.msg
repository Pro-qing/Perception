std_msgs/Header header

# 托盘相机状态
uint8 pallet_state  # 0:空闲 1:识别中 2:已识别 3:错误
float32 pallet_angle  # 托盘角度（弧度）
float32 pallet_distance  # 托盘距离（米）

# GPIO口状态
bool[8] gpio_states

# 叉尖雷达距离
float32 tip_distance

# 拉绳传感器高度
float32 cable_height
uint8 cable_status