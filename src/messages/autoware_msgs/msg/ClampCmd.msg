# 控制指令
Header header
uint8 mode  # 0:停止 1:夹取 2:释放 3:调整位置
float32 position  # 目标位置（厘米或百分比）
float32 speed  # 运动速度
float32 force_limit  # 力限制（牛顿）
float32 width  # 目标宽度（用于抱夹）
uint8 object_type  # 物体类型：0:未知 1:托盘 2:纸箱 3:桶
bool emergency_stop  # 急停标志