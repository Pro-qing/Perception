Header header
string vehicle_id  #车辆id
KeyPointArray path    #目标点信息 路径

uint8 taskCmd_state  #请求任务状态
uint8 taskCmd_type   #请求任务类型
string task_id    #任务id

bool is_map_update #地图更新 false:未完成 true：完成
string map_id      #地图id
string vehicle_ip  #车辆ip

uint32 safe_steps #安全下标
string control_info  #管制信息
