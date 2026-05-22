/**
 * @file lidar_pipeline_node.cpp
 * @brief 整合节点：sensor_input + downsample + no_ground
 *
 * 核心优化：
 *   1. 消除中间的 PCL→ROS→PCL 转换（原流程有 4 次不必要的序列化/反序列化）
 *   2. 三阶段流水线直接在 PCL 点云对象间传递，零拷贝或最少拷贝
 *   3. 使用 AsyncSpinner 多线程处理，互斥锁保护行为模式状态
 *   4. 预分配点云内存，减少运行时 malloc
 *
 * 话题保持不变：
 *   订阅: /points_16, /points_mid, /scan_left, /scan_right, /lqr_targetwayp
 *   发布: /points_raw, /points_downsampled, /lidar_no_ground, /lidar_ground
 *          /points_main_calibration, /points_mid_calibration,
 *          /points_left_calibration, /points_right_calibration
 *          /car (车身 Marker，可选)
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <autoware_msgs/Waypoint.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <lidar_input_downsample_noground/sensor_input.h>
#include <lidar_input_downsample_noground/downsample.h>
#include <lidar_input_downsample_noground/no_ground.h>

#include <string>
#include <map>
#include <mutex>
#include <stdexcept>

using namespace sensor_msgs;
using namespace message_filters;

class LidarPipelineNode {
public:
    LidarPipelineNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh), pnh_(pnh), current_behavior_id_(-1)
    {
        // ---- 读取话题名称 ----
        pnh_.param<std::string>("topics/main",    topic_main_,   "/points_16");
        pnh_.param<std::string>("topics/mid",     topic_mid_,    "/points_mid");
        pnh_.param<std::string>("topics/left",    topic_left_,   "/scan_left");
        pnh_.param<std::string>("topics/right",   topic_right_,  "/scan_right");
        pnh_.param<std::string>("topics/waypoint", topic_waypoint_, "/lqr_targetwayp");

        pnh_.param<std::string>("topics/output_raw",         topic_output_raw_,         "/points_raw");
        pnh_.param<std::string>("topics/output_downsampled", topic_output_downsampled_, "/points_downsampled");
        pnh_.param<std::string>("topics/output_no_ground",   topic_output_no_ground_,   "/lidar_no_ground");
        pnh_.param<std::string>("topics/output_ground",      topic_output_ground_,      "/lidar_ground");

        // ---- 初始化三个处理器 ----
        // 注意: processor 使用同一个 pnh_，参数通过 YAML 层级隔离
        // sensor_input 读取 calibration/, filter_radius/, behaviors/, parent_frame 等
        // downsample 读取 voxel_grid/, body_filter/, crop_box/, height_filter 等
        // no_ground 读取 ransac/, ground/, iterative/, pre_filter 等
        input_processor_ = std::make_unique<SensorInputProcessor>(nh_, pnh_);
        downsample_processor_ = std::make_unique<DownsampleProcessor>(nh_, pnh_);
        no_ground_processor_ = std::make_unique<NoGroundProcessor>(nh_, pnh_);

        // ---- 发布者 ----
        pub_points_raw_        = nh_.advertise<PointCloud2>(topic_output_raw_, 10);
        pub_points_downsampled_= nh_.advertise<PointCloud2>(topic_output_downsampled_, 10);
        pub_no_ground_         = nh_.advertise<PointCloud2>(topic_output_no_ground_, 10);
        pub_ground_            = nh_.advertise<PointCloud2>(topic_output_ground_, 10);

        // ---- 订阅者 + 四路时间同步 ----
        sub_16_   = new Subscriber<PointCloud2>(nh_, topic_main_,  1, ros::TransportHints().tcpNoDelay());
        sub_mid_  = new Subscriber<PointCloud2>(nh_, topic_mid_,   1, ros::TransportHints().tcpNoDelay());
        sub_left_ = new Subscriber<LaserScan>   (nh_, topic_left_,  1, ros::TransportHints().tcpNoDelay());
        sub_right_= new Subscriber<LaserScan>   (nh_, topic_right_, 1, ros::TransportHints().tcpNoDelay());

        sync_ = new Synchronizer<SyncPolicy>(SyncPolicy(10), *sub_16_, *sub_mid_, *sub_left_, *sub_right_);
        sync_->registerCallback(boost::bind(&LidarPipelineNode::syncCallback, this, _1, _2, _3, _4));

        // ---- 订阅 waypoint 话题用于行为模式切换 ----
        sub_waypoint_ = nh_.subscribe(topic_waypoint_, 1, &LidarPipelineNode::lqrWaypointCallback, this);

        ROS_INFO("\033[1;32m[Lidar Pipeline] Integrated node initialized.\033[0m");
        ROS_INFO("  Subscribed: main=%s, mid=%s, left=%s, right=%s, waypoint=%s",
                 topic_main_.c_str(), topic_mid_.c_str(), topic_left_.c_str(),
                 topic_right_.c_str(), topic_waypoint_.c_str());
        ROS_INFO("  Published:  raw=%s, downsampled=%s, no_ground=%s, ground=%s",
                 topic_output_raw_.c_str(), topic_output_downsampled_.c_str(),
                 topic_output_no_ground_.c_str(), topic_output_ground_.c_str());
    }

    ~LidarPipelineNode() {
        delete sync_;
        delete sub_16_;
        delete sub_mid_;
        delete sub_left_;
        delete sub_right_;
    }

private:
    ros::NodeHandle nh_, pnh_;

    // ---- 三个处理器 (解耦的独立模块) ----
    std::unique_ptr<SensorInputProcessor> input_processor_;
    std::unique_ptr<DownsampleProcessor> downsample_processor_;
    std::unique_ptr<NoGroundProcessor>   no_ground_processor_;

    // ---- 话题名称 ----
    std::string topic_main_, topic_mid_, topic_left_, topic_right_;
    std::string topic_waypoint_;
    std::string topic_output_raw_, topic_output_downsampled_;
    std::string topic_output_no_ground_, topic_output_ground_;

    // ---- 发布者 ----
    ros::Publisher pub_points_raw_;
    ros::Publisher pub_points_downsampled_;
    ros::Publisher pub_no_ground_;
    ros::Publisher pub_ground_;

    // ---- 传感器订阅者 ----
    Subscriber<PointCloud2>* sub_16_;
    Subscriber<PointCloud2>* sub_mid_;
    Subscriber<LaserScan>*   sub_left_;
    Subscriber<LaserScan>*   sub_right_;
    ros::Subscriber sub_waypoint_;

    // ---- 四路同步策略 ----
    typedef sync_policies::ApproximateTime<PointCloud2, PointCloud2, LaserScan, LaserScan> SyncPolicy;
    Synchronizer<SyncPolicy>* sync_;

    // ---- 行为模式 (多线程安全) ----
    std::mutex behavior_mutex_;
    int current_behavior_id_;

    // ============== Waypoint 回调：行为模式切换 (可在任意线程调用) ==============
    void lqrWaypointCallback(const autoware_msgs::Waypoint::ConstPtr& msg) {
        if (msg->wpsattr.routeBehavior.empty()) {
            ROS_WARN_THROTTLE(5.0, "[Lidar Pipeline] routeBehavior is empty, skipping.");
            return;
        }
        int behavior_id = static_cast<int>(msg->wpsattr.routeBehavior[0]);

        // 锁必须覆盖整个行为更新流程，包括 setSensorEnabled 写入 sensor_enabled_，
        // 否则与 syncCallback 中 processInput 读取 sensor_enabled_ 存在数据竞争，
        // 并发读写 std::map 会导致内部红黑树指针损坏，引发 SIGSEGV。
        std::lock_guard<std::mutex> lock(behavior_mutex_);
        if (behavior_id == current_behavior_id_) {
            return;
        }
        current_behavior_id_ = behavior_id;

        updateBehaviorConfig(behavior_id);
    }

    void updateBehaviorConfig(int behavior_id) {
        // 从 YAML 重新读取 behaviors 配置 (只在行为切换时调用，不频繁)
        XmlRpc::XmlRpcValue behaviors;
        if (!pnh_.getParam("behaviors", behaviors)) {
            return;
        }

        std::string key = std::to_string(behavior_id);
        if (behaviors.hasMember(key)) {
            XmlRpc::XmlRpcValue& sensors = behaviors[key];
            std::map<std::string, bool> config;
            if (sensors.hasMember("main"))  config["main"]  = static_cast<bool>(sensors["main"]);
            if (sensors.hasMember("mid"))   config["mid"]   = static_cast<bool>(sensors["mid"]);
            if (sensors.hasMember("left"))  config["left"]  = static_cast<bool>(sensors["left"]);
            if (sensors.hasMember("right")) config["right"] = static_cast<bool>(sensors["right"]);

            input_processor_->setSensorEnabled(config);

            ROS_INFO("\033[1;33m[Lidar Pipeline] Behavior changed to %d: main=%d, mid=%d, left=%d, right=%d\033[0m",
                     behavior_id, config["main"], config["mid"], config["left"], config["right"]);
        } else {
            ROS_WARN_THROTTLE(5.0, "[Lidar Pipeline] Unknown behavior ID %d, keeping previous config.", behavior_id);
        }
    }

    // ============== 四路同步回调 (核心流水线) ==============
    void syncCallback(const PointCloud2::ConstPtr& msg_16,
                      const PointCloud2::ConstPtr& msg_mid,
                      const LaserScan::ConstPtr&   msg_left,
                      const LaserScan::ConstPtr&   msg_right)
    try {
        // 安全检查：消息不能为空
        if (!msg_16 || !msg_mid || !msg_left || !msg_right) {
            ROS_WARN_THROTTLE(5.0, "[Lidar Pipeline] Received null message, skipping.");
            return;
        }

        // ========== Stage 1: Sensor Input (四路合并 + 标定变换) ==========
        pcl::PointCloud<pcl::PointXYZI>::Ptr merged_cloud;
        ros::Time stamp;
        if (!input_processor_->processInput(msg_16, msg_mid, msg_left, msg_right,
                                            behavior_mutex_, merged_cloud, stamp)) {
            return;
        }

        // 安全检查：合并点云不能为空
        if (!merged_cloud || merged_cloud->empty()) {
            ROS_WARN_THROTTLE(5.0, "[Lidar Pipeline] Merged cloud is empty after input, skipping.");
            return;
        }

        // 发布 /points_raw (保持原有话题不变)
        if (pub_points_raw_.getNumSubscribers() > 0 && merged_cloud && !merged_cloud->empty()) {
            PointCloud2 output_msg;
            pcl::toROSMsg(*merged_cloud, output_msg);
            output_msg.header.stamp    = stamp;
            output_msg.header.frame_id = "velodyne";
            pub_points_raw_.publish(output_msg);
        }

        // ========== Stage 2: Downsample (直接操作 PCL，无 ROS 转换) ==========
        // 注意: processDownsample 会替换 cloud 指针为降采样后的结果
        if (!downsample_processor_->processDownsample(merged_cloud)) {
            return;
        }

        // 安全检查：降采样后点云不能为空
        if (!merged_cloud || merged_cloud->empty()) {
            ROS_WARN_THROTTLE(5.0, "[Lidar Pipeline] Cloud is empty after downsample, skipping.");
            return;
        }

        // 发布 /points_downsampled (保持原有话题不变)
        if (pub_points_downsampled_.getNumSubscribers() > 0 && merged_cloud && !merged_cloud->empty()) {
            PointCloud2 output_msg;
            pcl::toROSMsg(*merged_cloud, output_msg);
            output_msg.header.stamp    = stamp;
            output_msg.header.frame_id = "velodyne";
            pub_points_downsampled_.publish(output_msg);
        }

        // ========== Stage 3: No Ground (直接操作 PCL，无 ROS 转换) ==========
        pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud;
        pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_cloud;

        bool has_no_ground_sub = (pub_no_ground_.getNumSubscribers() > 0);
        bool has_ground_sub    = (pub_ground_.getNumSubscribers() > 0);

        if (has_no_ground_sub || has_ground_sub) {
            no_ground_processor_->processNoGround(merged_cloud, ground_cloud, no_ground_cloud);

            // 发布 /lidar_ground (保持原有话题不变)
            if (has_ground_sub && ground_cloud && !ground_cloud->empty()) {
                PointCloud2 ground_msg;
                pcl::toROSMsg(*ground_cloud, ground_msg);
                ground_msg.header.stamp    = stamp;
                ground_msg.header.frame_id = "velodyne";
                pub_ground_.publish(ground_msg);
            }

            // 发布 /lidar_no_ground (保持原有话题不变)
            if (has_no_ground_sub && no_ground_cloud && !no_ground_cloud->empty()) {
                PointCloud2 no_ground_msg;
                pcl::toROSMsg(*no_ground_cloud, no_ground_msg);
                no_ground_msg.header.stamp    = stamp;
                no_ground_msg.header.frame_id = "velodyne";
                pub_no_ground_.publish(no_ground_msg);
            }
        }

    } catch (const std::exception& e) {
        ROS_ERROR("[Lidar Pipeline] Exception in syncCallback: %s", e.what());
    } catch (...) {
        ROS_ERROR("[Lidar Pipeline] Unknown exception in syncCallback");
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_pipeline_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    LidarPipelineNode node(nh, pnh);

    // 使用 AsyncSpinner 多线程处理 (4 个线程)
    // 与原 lidar_sensor_input 保持一致
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}