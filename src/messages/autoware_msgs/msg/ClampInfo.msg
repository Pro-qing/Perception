# 状态反馈
Header header
float32 current_position
float32 error_position
float32 current_speed
uint8 state  # 0:空闲 1:运动中 2:夹取完成 3:异常
uint8 grip_status  # 0:未夹取 1:夹取稳定 2:夹取不稳定
bool[2] contact_sensor  # 接触传感器
float32[2] pressure  # 左右压力值
float32[2] distance  # 左右测距值
string error_msg