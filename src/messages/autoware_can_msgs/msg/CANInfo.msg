Header header

# 车辆模式
int32 devmode
# 速度控制
float64 speed
# 角度控制
float64 angle
# 加速度控制
float64 accel    
# 减速度控制
float64 decel
# 叉臂控制
int32 forkarm
# 档位控制
int32 driveshift
# 刹车控制
int32 brake

# 行驶里程统计
float64 total_mileage   # 总行驶里程 (m)
float64 operation_hours # 运行小时数 (h)
int32 temp
int32 light
int32 door
int8[5] ev_mode

float32 battery_soc
float32 battery_electric
float32 battery_voltage
float32 battery_temp
uint8 battery_state

float64 targetspeed
float64 targetangle
float64 targetaccel   
float64 targetdecel  
int32 targetfork
int32 targetdriveshift
int32 targetbrake

