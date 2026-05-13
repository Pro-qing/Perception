#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <autoware_msgs/Waypoint.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>

#include <string>
#include <map>

using namespace sensor_msgs;
using namespace message_filters;

class SensorInputNode {
public:
    SensorInputNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh) {
        // ---- 从 YAML 读取基础参数 ----
        pnh_.param<std::string>("parent_frame", parent_frame_, "base_link");

        // ---- 从 YAML 读取话题名称 ----
        pnh_.param<std::string>("topics/main",    topic_main_,   "/points_16");
        pnh_.param<std::string>("topics/mid",     topic_mid_,    "/points_mid");
        pnh_.param<std::string>("topics/left",    topic_left_,   "/scan_left");
        pnh_.param<std::string>("topics/right",   topic_right_,  "/scan_right");
        pnh_.param<std::string>("topics/output",  topic_output_, "/points_raw");
        pnh_.param<std::string>("topics/waypoint", topic_waypoint_, "/lqr_targetwayp");

        // ---- 加载标定参数并计算变换矩阵 ----
        loadCalibrationParams();

        // ---- 加载单线雷达距离过滤半径 ----
        pnh_.param<double>("filter_radius/left",  filter_radius_left_,  0.0);
        pnh_.param<double>("filter_radius/right", filter_radius_right_, 0.0);
        if (filter_radius_left_ > 0.0) {
            ROS_INFO("\033[1;34m[Sensor Input] Left filter radius: %.2f m\033[0m", filter_radius_left_);
        }
        if (filter_radius_right_ > 0.0) {
            ROS_INFO("\033[1;34m[Sensor Input] Right filter radius: %.2f m\033[0m", filter_radius_right_);
        }

        // ---- 加载行为模式配置 ----
        loadBehaviorConfigs();

        // ---- 初始化默认行为：全部启用 ----
        sensor_enabled_["main"]  = true;
        sensor_enabled_["mid"]   = true;
        sensor_enabled_["left"]  = true;
        sensor_enabled_["right"] = true;
        current_behavior_id_ = -1;

        // ---- 发布者 ----
        pub_points_raw_ = nh_.advertise<PointCloud2>(topic_output_, 10);

        // ---- 发布 TF 静态变换（4个雷达相对于 base_link 的位置）----
        publishSensorTF();
        pub_main_calib_  = nh_.advertise<PointCloud2>("/points_main_calibration", 10);
        pub_mid_calib_   = nh_.advertise<PointCloud2>("/points_mid_calibration", 10);
        pub_left_calib_  = nh_.advertise<PointCloud2>("/points_left_calibration", 10);
        pub_right_calib_ = nh_.advertise<PointCloud2>("/points_right_calibration", 10);

        // ---- 订阅者 + 四路时间同步 ----
        sub_16_   = new Subscriber<PointCloud2>(nh_, topic_main_,  1, ros::TransportHints().tcpNoDelay());
        sub_mid_  = new Subscriber<PointCloud2>(nh_, topic_mid_,   1, ros::TransportHints().tcpNoDelay());
        sub_left_ = new Subscriber<LaserScan>   (nh_, topic_left_,  1, ros::TransportHints().tcpNoDelay());
        sub_right_= new Subscriber<LaserScan>   (nh_, topic_right_, 1, ros::TransportHints().tcpNoDelay());

        sync_ = new Synchronizer<SyncPolicy>(SyncPolicy(10), *sub_16_, *sub_mid_, *sub_left_, *sub_right_);
        sync_->registerCallback(boost::bind(&SensorInputNode::syncCallback, this, _1, _2, _3, _4));

        // ---- 订阅 waypoint 话题用于行为模式切换 ----
        sub_waypoint_ = nh_.subscribe(topic_waypoint_, 1, &SensorInputNode::lqrWaypointCallback, this);

        ROS_INFO("\033[1;32m[Sensor Input] Node initialized. Topics: main=%s, mid=%s, left=%s, right=%s, output=%s, waypoint=%s\033[0m",
                 topic_main_.c_str(), topic_mid_.c_str(), topic_left_.c_str(),
                 topic_right_.c_str(), topic_output_.c_str(), topic_waypoint_.c_str());
    }

    ~SensorInputNode() {
        delete sync_;
        delete sub_16_;
        delete sub_mid_;
        delete sub_left_;
        delete sub_right_;
    }

private:
    ros::NodeHandle nh_, pnh_;
    std::string parent_frame_;
    laser_geometry::LaserProjection projector_;
    ros::Publisher pub_points_raw_;
    ros::Publisher pub_main_calib_, pub_mid_calib_, pub_left_calib_, pub_right_calib_;
    tf2_ros::StaticTransformBroadcaster static_broadcaster_;

    // 话题名称（从 YAML 读取）
    std::string topic_main_, topic_mid_, topic_left_, topic_right_;
    std::string topic_output_, topic_waypoint_;

    // 传感器订阅者
    Subscriber<PointCloud2>* sub_16_;
    Subscriber<PointCloud2>* sub_mid_;
    Subscriber<LaserScan>*   sub_left_;
    Subscriber<LaserScan>*   sub_right_;

    // Waypoint 订阅者
    ros::Subscriber sub_waypoint_;

    // 四路同步策略
    typedef sync_policies::ApproximateTime<PointCloud2, PointCloud2, LaserScan, LaserScan> SyncPolicy;
    Synchronizer<SyncPolicy>* sync_;

    // 变换矩阵 (相对于 velodyne 坐标系)
    Eigen::Affine3f trans_main_, trans_mid_, trans_left_, trans_right_;
    // base_link → velodyne 变换
    Eigen::Affine3f base_to_velo_;

    // 行为模式：每个行为 ID 对应各传感器开关
    // key: 行为ID (字符串), value: map<传感器名, 是否启用>
    std::map<int, std::map<std::string, bool>> behavior_configs_;

    // 当前传感器启用状态
    std::map<std::string, bool> sensor_enabled_;
    int current_behavior_id_;

    // 单线雷达距离过滤半径（米），0 表示不过滤
    double filter_radius_left_, filter_radius_right_;

    // ============== 加载标定参数 ==============
    void loadCalibrationParams() {
        // main 雷达是 velodyne，标定参数 (calibration/main) 描述 velodyne 相对于 base_link 的位姿
        // 其它雷达的标定参数 (calibration/mid/left/right) 描述相对 velodyne 的变换
        // 最终点云统一发布到 velodyne 坐标系下
        Eigen::Affine3f trans_main_to_base  = getTransformFromParam("calibration/main");
        Eigen::Affine3f trans_mid_to_velo   = getTransformFromParam("calibration/mid");
        Eigen::Affine3f trans_left_to_velo  = getTransformFromParam("calibration/left");
        Eigen::Affine3f trans_right_to_velo = getTransformFromParam("calibration/right");

        // main (velodyne) 本身就在 velodyne 坐标系，不需要变换
        trans_main_  = Eigen::Affine3f::Identity();
        // mid/left/right 的标定参数是相对于 velodyne 的，直接使用
        trans_mid_   = trans_mid_to_velo;
        trans_left_  = trans_left_to_velo;
        trans_right_ = trans_right_to_velo;

        // base_link → velodyne 变换，用于发布 TF
        base_to_velo_ = trans_main_to_base;

        ROS_INFO("\033[1;34m[Sensor Input] Calibration matrices loaded. All points will be in velodyne frame.\033[0m");
    }

    // ============== 发布静态 TF ==============
    // TF 树: base_link → velodyne → lidar_main / lidar_mid / lidar_left / lidar_right
    void publishSensorTF() {
        std::vector<geometry_msgs::TransformStamped> transforms;

        // 1. base_link → velodyne (来自 calibration/main)
        {
            geometry_msgs::TransformStamped ts;
            ts.header.stamp = ros::Time::now();
            ts.header.frame_id = "base_link";
            ts.child_frame_id = "velodyne";

            Eigen::Vector3f pos = base_to_velo_.translation();
            ts.transform.translation.x = pos.x();
            ts.transform.translation.y = pos.y();
            ts.transform.translation.z = pos.z();

            Eigen::Matrix3f rot = base_to_velo_.rotation();
            Eigen::Quaternionf quat(rot);
            ts.transform.rotation.x = quat.x();
            ts.transform.rotation.y = quat.y();
            ts.transform.rotation.z = quat.z();
            ts.transform.rotation.w = quat.w();

            transforms.push_back(ts);

            ROS_INFO("\033[1;36m[Sensor Input] TF: base_link -> velodyne [%+.2f, %+.2f, %+.2f]\033[0m",
                     pos.x(), pos.y(), pos.z());
        }

        // 2. velodyne → lidar_main (Identity, main 就是 velodyne)
        // 3. velodyne → lidar_mid / lidar_left / lidar_right
        std::vector<std::pair<std::string, Eigen::Affine3f>> sensor_transforms = {
            {"lidar_main",  trans_main_},
            {"lidar_mid",   trans_mid_},
            {"lidar_left",  trans_left_},
            {"lidar_right", trans_right_}
        };

        for (const auto& kv : sensor_transforms) {
            geometry_msgs::TransformStamped ts;
            ts.header.stamp = ros::Time::now();
            ts.header.frame_id = "velodyne";
            ts.child_frame_id = kv.first;

            Eigen::Vector3f pos = kv.second.translation();
            ts.transform.translation.x = pos.x();
            ts.transform.translation.y = pos.y();
            ts.transform.translation.z = pos.z();

            Eigen::Matrix3f rot = kv.second.rotation();
            Eigen::Quaternionf quat(rot);
            ts.transform.rotation.x = quat.x();
            ts.transform.rotation.y = quat.y();
            ts.transform.rotation.z = quat.z();
            ts.transform.rotation.w = quat.w();

            transforms.push_back(ts);

            ROS_INFO("\033[1;36m[Sensor Input] TF: velodyne -> %s [%+.2f, %+.2f, %+.2f]\033[0m",
                     kv.first.c_str(), pos.x(), pos.y(), pos.z());
        }

        static_broadcaster_.sendTransform(transforms);
    }

    Eigen::Affine3f getTransformFromParam(const std::string& param_ns) {
        double x = 0, y = 0, z = 0, roll = 0, pitch = 0, yaw = 0;
        pnh_.param(param_ns + "/x", x, 0.0);
        pnh_.param(param_ns + "/y", y, 0.0);
        pnh_.param(param_ns + "/z", z, 0.0);
        pnh_.param(param_ns + "/roll", roll, 0.0);
        pnh_.param(param_ns + "/pitch", pitch, 0.0);
        pnh_.param(param_ns + "/yaw", yaw, 0.0);

        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.translation() << x, y, z;
        // 旋转顺序: Rz * Ry * Rx
        transform.rotate(Eigen::AngleAxisf(yaw,   Eigen::Vector3f::UnitZ()));
        transform.rotate(Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()));
        transform.rotate(Eigen::AngleAxisf(roll,  Eigen::Vector3f::UnitX()));
        return transform;
    }

    // ============== 加载行为模式配置 ==============
    void loadBehaviorConfigs() {
        // 获取 behaviors 下所有 key (行为 ID 字符串)
        XmlRpc::XmlRpcValue behaviors;
        if (!pnh_.getParam("behaviors", behaviors)) {
            ROS_WARN("[Sensor Input] No 'behaviors' config found. All sensors always enabled.");
            return;
        }

        if (behaviors.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
            ROS_WARN("[Sensor Input] 'behaviors' is not a struct. Skipping.");
            return;
        }

        for (auto it = behaviors.begin(); it != behaviors.end(); ++it) {
            int behavior_id = std::stoi(it->first);
            XmlRpc::XmlRpcValue& sensors = it->second;

            std::map<std::string, bool> config;
            if (sensors.hasMember("main"))  config["main"]  = static_cast<bool>(sensors["main"]);
            if (sensors.hasMember("mid"))   config["mid"]   = static_cast<bool>(sensors["mid"]);
            if (sensors.hasMember("left"))  config["left"]  = static_cast<bool>(sensors["left"]);
            if (sensors.hasMember("right")) config["right"] = static_cast<bool>(sensors["right"]);

            behavior_configs_[behavior_id] = config;
            ROS_INFO("[Sensor Input] Behavior %d loaded: main=%d, mid=%d, left=%d, right=%d",
                     behavior_id, config["main"], config["mid"], config["left"], config["right"]);
        }
    }

    // ============== Waypoint 回调：行为模式切换 ==============
    void lqrWaypointCallback(const autoware_msgs::Waypoint::ConstPtr& msg) {
        if (msg->wpsattr.routeBehavior.empty()) {
            ROS_WARN_THROTTLE(5.0, "[Sensor Input] routeBehavior is empty, skipping.");
            return;
        }
        int behavior_id = static_cast<int>(msg->wpsattr.routeBehavior[0]);

        if (behavior_id == current_behavior_id_) {
            return; // 行为未变化，不做处理
        }

        auto it = behavior_configs_.find(behavior_id);
        if (it != behavior_configs_.end()) {
            current_behavior_id_ = behavior_id;
            sensor_enabled_ = it->second;

            ROS_INFO("\033[1;33m[Sensor Input] Behavior changed to %d: main=%d, mid=%d, left=%d, right=%d\033[0m",
                     behavior_id,
                     sensor_enabled_["main"],
                     sensor_enabled_["mid"],
                     sensor_enabled_["left"],
                     sensor_enabled_["right"]);
        } else {
            ROS_WARN_THROTTLE(5.0, "[Sensor Input] Unknown behavior ID %d, keeping previous config.", behavior_id);
        }
    }

    // ============== 点云处理辅助函数 ==============
    void processScan(const LaserScan::ConstPtr& scan_msg, const Eigen::Affine3f& transform,
                     pcl::PointCloud<pcl::PointXYZI>::Ptr& out_cloud) {
        PointCloud2 temp_cloud2;
        try { projector_.projectLaser(*scan_msg, temp_cloud2); } catch (...) { return; }
        pcl::PointCloud<pcl::PointXYZI> raw_pcl;
        pcl::fromROSMsg(temp_cloud2, raw_pcl);
        pcl::transformPointCloud(raw_pcl, *out_cloud, transform);
    }

    // 按半径过滤点云，保留距离原点小于 max_radius 的点
    void filterByRadius(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, double max_radius) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());
        double max_radius_sq = max_radius * max_radius;
        for (const auto& pt : cloud->points) {
            if (pt.x * pt.x + pt.y * pt.y + pt.z * pt.z <= max_radius_sq) {
                filtered->points.push_back(pt);
            }
        }
        filtered->width = filtered->points.size();
        filtered->height = 1;
        filtered->is_dense = true;
        cloud = filtered;
    }

    void processCloud(const PointCloud2::ConstPtr& cloud_msg, const Eigen::Affine3f& transform,
                      pcl::PointCloud<pcl::PointXYZI>::Ptr& out_cloud) {
        pcl::PointCloud<pcl::PointXYZI> raw_pcl;
        pcl::fromROSMsg(*cloud_msg, raw_pcl);
        pcl::transformPointCloud(raw_pcl, *out_cloud, transform);
    }

    // ============== 发布标定后的单路点云 ==============
    void publishCalibratedCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                                const ros::Time& stamp, const ros::Publisher& pub) {
        if (cloud->empty() || pub.getNumSubscribers() == 0) {
            return;
        }
        PointCloud2 output_msg;
        pcl::toROSMsg(*cloud, output_msg);
        output_msg.header.stamp    = stamp;
        output_msg.header.frame_id = parent_frame_;
        pub.publish(output_msg);
    }

    // ============== 四路同步回调 ==============
    void syncCallback(const PointCloud2::ConstPtr& msg_16,
                      const PointCloud2::ConstPtr& msg_mid,
                      const LaserScan::ConstPtr&   msg_left,
                      const LaserScan::ConstPtr&   msg_right)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZI>());

        // 根据当前行为模式决定处理哪些传感器
        if (sensor_enabled_["main"]) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_main(new pcl::PointCloud<pcl::PointXYZI>());
            processCloud(msg_16, trans_main_, cloud_main);
            *merged_cloud += *cloud_main;
            publishCalibratedCloud(cloud_main, msg_16->header.stamp, pub_main_calib_);
        }

        if (sensor_enabled_["mid"]) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_mid(new pcl::PointCloud<pcl::PointXYZI>());
            processCloud(msg_mid, trans_mid_, cloud_mid);
            *merged_cloud += *cloud_mid;
            publishCalibratedCloud(cloud_mid, msg_mid->header.stamp, pub_mid_calib_);
        }

        if (sensor_enabled_["left"]) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_left(new pcl::PointCloud<pcl::PointXYZI>());
            processScan(msg_left, trans_left_, cloud_left);
            // 先发布标定后的完整点云
            publishCalibratedCloud(cloud_left, msg_left->header.stamp, pub_left_calib_);
            // 再对合并用的点云做距离过滤
            if (filter_radius_left_ > 0.0) {
                filterByRadius(cloud_left, filter_radius_left_);
            }
            *merged_cloud += *cloud_left;
        }

        if (sensor_enabled_["right"]) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_right(new pcl::PointCloud<pcl::PointXYZI>());
            processScan(msg_right, trans_right_, cloud_right);
            // 先发布标定后的完整点云
            publishCalibratedCloud(cloud_right, msg_right->header.stamp, pub_right_calib_);
            // 再对合并用的点云做距离过滤
            if (filter_radius_right_ > 0.0) {
                filterByRadius(cloud_right, filter_radius_right_);
            }
            *merged_cloud += *cloud_right;
        }

        if (merged_cloud->empty()) {
            return;
        }

        if (pub_points_raw_.getNumSubscribers() > 0) {
            PointCloud2 output_msg;
            pcl::toROSMsg(*merged_cloud, output_msg);
            output_msg.header.stamp    = msg_16->header.stamp;
            output_msg.header.frame_id = parent_frame_;
            pub_points_raw_.publish(output_msg);
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_sensor_input_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    SensorInputNode node(nh, pnh);

    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}