uint8 mode
float64 speed
float64 angle
uint8 gear
uint8 forkarm
float64 forkarm_height
uint16 lift
bool inplace
bool crash
uint8 ctr_state
int16[3] pallet
uint8[] gpio
int16[8] param
int8 current_floor
int8 safe_state
float64 total_mileage   # 总行驶里程 (m)
float64 operation_hours # 运行小时数 (h)
geometry_msgs/Pose  pose