std_msgs/Header header       # stamp 存储最原始雷达数据的时间戳，作为同一帧的唯一ID
string node_name             # 节点编号+名称，如 "1_input", "2_downsample"
float64 transmission_delay   # 数据抵达此节点前，经过的总时间 (ms)
float64 processing_time      # 此节点算法的实际运行耗时 (ms)
float64 total_latency        # 截止本节点处理完，距离雷达采集的总延迟 (ms)
