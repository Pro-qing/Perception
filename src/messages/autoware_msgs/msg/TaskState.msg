Header header

# 任务基本信息
string task_id                     # 任务唯一标识 (UUID或任务编号)
uint8 task_state                   # 任务执行状态 (枚举见下)
uint8 task_type                    # 任务类型 (枚举见下)
string state_message              # 状态文字描述

uint32 safe_steps                # 安全步骤下标
uint32 total_steps               # 任务总步骤数
uint32 current_step             # 当前步骤
string control_info          #管制信息
KeyPoint[] path_points       # 路径关键点序列 (按顺序执行的点位)