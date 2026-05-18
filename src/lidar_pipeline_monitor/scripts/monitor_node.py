#!/usr/bin/env python3
import rospy
from lidar_pipeline_monitor.msg import PipelineMetrics
from collections import defaultdict
import threading

class PipelineMonitor:
    def __init__(self):
        rospy.init_node('pipeline_monitor_node')
        self.sub = rospy.Subscriber('/pipeline/metrics', PipelineMetrics, self.metrics_cb)
        
        # 用于存储同一帧点云在各个节点的处理数据
        # key: frame_stamp (原始时间戳), value: dict of node metrics
        self.frame_data = defaultdict(dict)
        self.lock = threading.Lock()
        
        # 假设我们期望收到 5 个节点的数据
        self.EXPECTED_NODES = 6
        
        rospy.loginfo("\033[1;32m[Pipeline Monitor] Started. Waiting for metrics...\033[0m")

    def metrics_cb(self, msg):
        stamp_key = msg.header.stamp.to_nsec()
        
        with self.lock:
            self.frame_data[stamp_key][msg.node_name] = msg
            
            # 当这一帧的数据收集齐了（经过了最后一步），就打印报告
            if len(self.frame_data[stamp_key]) == self.EXPECTED_NODES:
                self.print_report(stamp_key)
                del self.frame_data[stamp_key] # 打印完清理内存

        # 定期清理长期未处理完的孤儿数据（防止某些帧丢失导致内存泄漏）
        self.cleanup_old_frames()

    def print_report(self, stamp_key):
        nodes = self.frame_data[stamp_key]
        
        # 按节点名称排序 (因为前面埋点起了 "1_", "2_" 开头的名字)
        sorted_nodes = sorted(nodes.values(), key=lambda x: x.node_name)
        
        end_to_end_latency = sorted_nodes[-1].total_latency
        
        print("\n" + "="*50)
        print(f"📊 Frame Timeline (Stamp: {stamp_key})")
        print(f"⏱️  End-to-End Latency: \033[1;31m{end_to_end_latency:.2f} ms\033[0m")
        print("-" * 50)
        print(f"{'Node Name':<18} | {'Trans/Queue Wait':<16} | {'Processing Time':<16}")
        print("-" * 50)
        
        for n in sorted_nodes:
            # 这里的 Trans/Queue 等待时间，其实包含了上一级处理完到这一级开始之间的 ROS 序列化和网络耗时
            print(f"{n.node_name:<18} | {n.transmission_delay:>10.2f} ms     | {n.processing_time:>10.2f} ms")
        print("="*50)

    def cleanup_old_frames(self):
        current_time = rospy.Time.now().to_nsec()
        with self.lock:
            keys_to_delete = []
            for stamp_key in self.frame_data.keys():
                # 如果超过 1 秒钟还没收集齐这一帧，视为丢包
                if (current_time - stamp_key) > 1e9: 
                    keys_to_delete.append(stamp_key)
            for k in keys_to_delete:
                del self.frame_data[k]

if __name__ == '__main__':
    monitor = PipelineMonitor()
    rospy.spin()