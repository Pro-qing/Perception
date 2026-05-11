Header header

uint8 NORMAL = 0        # 正常
uint8 LOW = 1         # 低电压
uint8 CHARGING = 2  #充电
uint8 WARN = 3      # 故障


# 电池核心状态
float32 soc                    # 剩余电量百分比 [0.0-100.0]%
float32 electric                # 电流 (A) 正值:放电, 负值:充电
float32 voltage                # 总电压 (V)
uint8 state                    # 当前电池状态 (使用上述常量)
float32 temperature            # 电池温度 (°C)
float32 charge_power           # 充电功率 (W)