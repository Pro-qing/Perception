Header header  #头
bool is_map_update #地图更新 false:未更新 true：更新
string vehicle_id  #车辆id
string vehicle_ip  #车辆ip
geometry_msgs/Pose pose  #当前位置

string task_id    #任务id
uint8 taskInfo_state  #当前任务状态
uint8 taskInfo_type   #当前任务类型
KeyPointArray goal_pose    #目标点信息

bool charge_cmd      #充电请求

uint8 taskCmd_state  #请求任务状态
uint8 taskCmd_type   #请求任务类型