#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <autoware_msgs/KeyPointArray.h>
#include <autoware_msgs/KeyPoint.h>
#include <autoware_msgs/KeyPointPyte.h>

#include <lidar_pipeline_monitor/PipelineMetrics.h>

#include <vector>
#include <mutex>
#include <cmath>

// ============================================================
//  充电桩过滤节点
//
//  功能:
//    1. 订阅 /keypoint_path 获取充电站位置
//    2. 订阅 /lqr_dire 获取控制指令 (判断是否需要过滤)
//    3. 订阅 /lidar_no_ground 输入去除地面后的点云
//    4. 根据充电站位置过滤点云中的充电桩点
//    5. 发布 /lidar_no_ground_no_charge (过滤后点云)
//
//  控制指令含义 (lqr_dire):
//    2  : 充电中 → 过滤充电桩点
//    <0 : 充电完成/离开 → 过滤充电桩点
//    其他: 正常行驶 → 直接透传
//
//  如果 charge_enable=false，则直接透传输入点云
// ============================================================

class LidarNoChargeNode {
public:
    LidarNoChargeNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh), pnh_(pnh), tf_listener_(tf_buffer_), fliter_charge_(0), charge_enable_(false)
    {
        // ---- 从 YAML 读取话题参数 ----
        pnh_.param<std::string>("input_topic",       input_topic_,       "/lidar_no_ground");
        pnh_.param<std::string>("output_topic",      output_topic_,      "/lidar_no_ground_no_charge");
        pnh_.param<std::string>("keypoint_topic",    keypoint_topic_,    "/keypoint_path");
        pnh_.param<std::string>("control_topic",     control_topic_,     "/lqr_dire");

        // ---- 从 YAML 读取充电桩参数 ----
        pnh_.param<bool>  ("charge/enable", charge_enable_, false);
        pnh_.param<double>("charge/length", charge_length_, 1.2);
        pnh_.param<double>("charge/wide",   charge_wide_,   1.2);
        pnh_.param<double>("charge/high",   charge_high_,   1.2);
        pnh_.param<double>("charge/error",  charge_error_,  1.2);

        // ---- 发布者 ----
        pub_no_charge_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 10);
        marker_pub_    = nh_.advertise<visualization_msgs::Marker>("/charge_area_marker", 1, true);

        pub_metrics_ = nh_.advertise<lidar_pipeline_monitor::PipelineMetrics>("/pipeline/metrics", 100);

        // ---- 订阅者 ----
        sub_keypoint_ = nh_.subscribe(keypoint_topic_, 10, &LidarNoChargeNode::keyPointCallback, this);
        sub_control_  = nh_.subscribe(control_topic_,  10, &LidarNoChargeNode::ctrolCallback,   this);
        sub_points_   = nh_.subscribe(input_topic_,    10, &LidarNoChargeNode::pointCloudCallback, this);

        // ---- 充电站位姿更新定时器 (10Hz) ----
        charge_timer_ = nh_.createTimer(ros::Duration(0.1), &LidarNoChargeNode::chargeTimerCallback, this);

        ROS_INFO("\033[1;32m[Lidar No Charge] Node initialized.\033[0m");
        ROS_INFO("  input_topic:       %s", input_topic_.c_str());
        ROS_INFO("  output_topic:      %s", output_topic_.c_str());
        ROS_INFO("  keypoint_topic:    %s", keypoint_topic_.c_str());
        ROS_INFO("  control_topic:     %s", control_topic_.c_str());
        ROS_INFO("  charge_enable:     %s", charge_enable_ ? "true" : "false");
        ROS_INFO("  charge size:       %.2f x %.2f x %.2f m", charge_length_, charge_wide_, charge_high_);
        ROS_INFO("  charge_error:      %.2f m", charge_error_);
    }

private:
    ros::NodeHandle nh_, pnh_;

    // 发布者
    ros::Publisher pub_no_charge_;
    ros::Publisher marker_pub_;

    ros::Publisher pub_metrics_; 

    // 订阅者
    ros::Subscriber sub_keypoint_;
    ros::Subscriber sub_control_;
    ros::Subscriber sub_points_;

    // TF
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // 定时器
    ros::Timer charge_timer_;

    // 话题参数
    std::string input_topic_;
    std::string output_topic_;
    std::string keypoint_topic_;
    std::string control_topic_;

    // 充电桩参数
    bool   charge_enable_;
    double charge_length_;
    double charge_wide_;
    double charge_high_;
    double charge_error_;

    // 共享状态 (需要互斥保护)
    std::mutex data_mutex_;
    int fliter_charge_;                         // 0=正常, 1=充电中, 2=充电结束
    std::vector<geometry_msgs::Pose> fliterpose_;  // 充电站位姿列表 (map坐标系)
    geometry_msgs::Pose transPose_;               // 充电站位姿 (velodyne坐标系)

    // ============================================================
    //  可视化: 在RViz中显示充电桩区域
    // ============================================================
    void displayPolygonInRviz(const geometry_msgs::Pose& pose, double len, double wide, double high) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "velodyne";
        marker.header.stamp = ros::Time::now();
        marker.ns = "charge_area";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose = pose;
        marker.scale.x = len;
        marker.scale.y = wide;
        marker.scale.z = high;

        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.3f;

        marker.lifetime = ros::Duration(0.5);
        marker_pub_.publish(marker);
    }

    // ============================================================
    //  坐标变换: 将位姿从一个坐标系变换到另一个坐标系
    // ============================================================
    void transformPose(const geometry_msgs::TransformStamped& tf,
                       const geometry_msgs::Pose& in_pose,
                       geometry_msgs::Pose& out_pose)
    {
        geometry_msgs::PoseStamped in_stamped, out_stamped;
        in_stamped.pose = in_pose;
        in_stamped.header.frame_id = "map";
        in_stamped.header.stamp = ros::Time(0);
        tf2::doTransform(in_stamped, out_stamped, tf);
        out_pose = out_stamped.pose;
    }

    // ============================================================
    //  关键点回调: 处理充电站位置 (来自 /keypoint_path)
    //
    //  从 KeyPointArray 的 path 中查找 type_name == "charges" 的关键点,
    //  提取其位姿并根据 error 参数进行偏移修正
    // ============================================================
    void keyPointCallback(const autoware_msgs::KeyPointArrayConstPtr& msg) {
        double error, len, wide, high;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            error = charge_error_;
            len   = charge_length_;
            wide  = charge_wide_;
            high  = charge_high_;
        }

        std::vector<geometry_msgs::Pose> new_poses;
        bool found = false;

        if (!msg->path.empty()) {
            for (const auto& point : msg->path) {
                for (const auto& type : point.types) {
                    // 如果类型名为 "charges"，则认为是充电站
                    if (type.type_name == "charges") {
                        geometry_msgs::Pose p = point.pose.pose;
                        // 根据误差修正位置 (沿朝向方向偏移)
                        if (error != 0) {
                            double yaw = tf2::getYaw(p.orientation);
                            p.position.x += cos(yaw) * error;
                            p.position.y += sin(yaw) * error;
                        }
                        new_poses.push_back(p);
                        // 在RViz中显示充电站区域 (使用velodyne坐标系下的位姿)
                        displayPolygonInRviz(p, len, wide, high);
                        found = true;
                    }
                }
            }
        }

        // 更新全局充电站位姿列表
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            fliterpose_ = new_poses;
        }

        // 如果未找到充电站，清除RViz中的Marker
        if (!found) {
            geometry_msgs::Pose empty_pose;
            displayPolygonInRviz(empty_pose, 0, 0, 0);
        }
    }

    // ============================================================
    //  控制回调: 根据指令切换充电过滤模式 (来自 /lqr_dire)
    //
    //  指令含义:
    //    2  : 充电中     → fliter_charge_ = 1
    //    <0 : 充电完成   → fliter_charge_ = 2
    //    其他: 正常行驶  → fliter_charge_ = 0
    // ============================================================
    void ctrolCallback(const std_msgs::Int8ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (charge_enable_ && msg->data == 2)      fliter_charge_ = 1;  // 充电中
        else if (charge_enable_ && msg->data < 0)  fliter_charge_ = 2;  // 充电结束
        else                                        fliter_charge_ = 0;  // 正常行驶
    }

    // ============================================================
    //  充电站定时器回调: 更新充电站在 velodyne 坐标系下的位置
    //
    //  以10Hz频率运行，将充电站位姿从 map 坐标系变换到 velodyne 坐标系
    // ============================================================
    void chargeTimerCallback(const ros::TimerEvent& event) {
        std::lock_guard<std::mutex> lock(data_mutex_);

        // 仅在充电状态下且存在充电站位姿时执行变换
        if (fliter_charge_ == 0 || fliterpose_.empty()) {
            return;
        }

        try {
            // 获取从 map 到 velodyne 的坐标变换
            geometry_msgs::TransformStamped t = tf_buffer_.lookupTransform("velodyne", "map", ros::Time(0));

            // 选择目标位姿 (多个充电站时根据状态选择)
            geometry_msgs::Pose tp;
            if (fliterpose_.size() == 1) {
                tp = fliterpose_[0];
            } else {
                tp = (fliter_charge_ == 2) ? fliterpose_[0] : fliterpose_[1];
            }

            // 执行坐标变换
            transformPose(t, tp, transPose_);
        } catch (tf2::TransformException& ex) {
            // TF变换失败时静默忽略 (可能还没收到TF数据)
        }
    }

    // ============================================================
    //  点云回调: 对输入点云进行充电桩过滤
    //
    //  流程:
    //    1. 检查是否需要充电桩过滤 (charge_enable + 充电状态)
    //    2. 如果需要过滤，根据充电站包围盒过滤点云
    //    3. 发布过滤后的点云到 /lidar_no_ground_no_charge
    // ============================================================
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {

        ros::Time cb_start = ros::Time::now(); // 【新增头】

        // 没有订阅者时不处理
        if (pub_no_charge_.getNumSubscribers() == 0) {
            return;
        }

        // 转换为 PCL 点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->empty()) {
            ROS_WARN_THROTTLE(5.0, "[Lidar No Charge] Received empty cloud, skipping.");
            return;
        }

        // 判断是否需要进行充电桩过滤
        bool need_filter = false;
        geometry_msgs::Pose charge_pose;
        double half_len = 0, half_wide = 0, half_high = 0;

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            if (charge_enable_ && fliter_charge_ != 0 && !fliterpose_.empty()) {
                need_filter = true;
                charge_pose = transPose_;
                half_len   = charge_length_ / 2.0;
                half_wide  = charge_wide_   / 2.0;
                half_high  = charge_high_   / 2.0;
            }
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>());

        if (need_filter) {
            // 需要过滤: 移除充电站包围盒内的点
            double cx = charge_pose.position.x;
            double cy = charge_pose.position.y;
            double cz = charge_pose.position.z;
            double yaw = tf2::getYaw(charge_pose.orientation);
            double cos_yaw = cos(-yaw);  // 逆旋转
            double sin_yaw = sin(-yaw);

            size_t before_size = cloud->size();

            for (const auto& pt : cloud->points) {
                // 将点变换到充电站局部坐标系
                double dx = pt.x - cx;
                double dy = pt.y - cy;
                double dz = pt.z - cz;

                // 旋转到充电站朝向
                double local_x = dx * cos_yaw - dy * sin_yaw;
                double local_y = dx * sin_yaw + dy * cos_yaw;

                // 判断是否在包围盒内
                if (local_x < -half_len || local_x > half_len ||
                    local_y < -half_wide || local_y > half_wide ||
                    dz < -half_high || dz > half_high) {
                    // 在包围盒外，保留该点
                    output_cloud->points.push_back(pt);
                }
            }

            ROS_INFO_THROTTLE(5.0,
                "[Lidar No Charge] Filtered %lu → %lu points (removed %lu charge points)",
                before_size, output_cloud->size(), before_size - output_cloud->size());
        } else {
            // 不需要过滤: 直接透传
            output_cloud = cloud;
        }

        // 设置点云属性
        output_cloud->width    = output_cloud->points.size();
        output_cloud->height   = 1;
        output_cloud->is_dense = true;

        // 发布结果
        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(*output_cloud, output_msg);
        output_msg.header.stamp    = msg->header.stamp;
        output_msg.header.frame_id = msg->header.frame_id;
        pub_no_charge_.publish(output_msg);

        // 【新增尾】
        ros::Time cb_end = ros::Time::now();
        lidar_pipeline_monitor::PipelineMetrics metric;
        metric.header.stamp = msg->header.stamp; 
        metric.node_name = "4_no_charge";
        metric.transmission_delay = (cb_start - msg->header.stamp).toSec() * 1000.0;
        metric.processing_time = (cb_end - cb_start).toSec() * 1000.0;
        metric.total_latency = (cb_end - msg->header.stamp).toSec() * 1000.0;
        pub_metrics_.publish(metric);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_no_charge_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    LidarNoChargeNode node(nh, pnh);

    ros::spin();

    return 0;
}