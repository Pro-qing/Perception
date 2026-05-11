Header header
uint8 OPEN = 1              # 开门命令
uint8 CALL = 2              # 呼梯命令
uint8 command_type          # 控制命令类型
uint8 associated_doors      # 关联门（备用, bit0:1号门, bit1:2号门）
int8 floor                  # 呼梯楼层
uint8 door_id               # 电梯门号 (bit0:1号门, bit1:2号门)
uint8 door_cmd              # 0x01开门，0x02取消

