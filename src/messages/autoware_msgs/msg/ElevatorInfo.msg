Header header
uint8 command_type              # 控制命令类型
int8 floor                      # 当前楼层/已经呼叫楼层
uint8[2] door_status            # 门状态 ([0]:1号门, [1]:2号门)
uint8 card_action               # 刷卡动作
bool comm_ok                    # 通信是否正常（true:正常,false:异常）


