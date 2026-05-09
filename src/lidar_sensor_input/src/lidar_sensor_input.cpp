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

using namespace sensor_msgs;
using namespace message_filters;

class SensorInputNode {
public:
    SensorInputNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh) {
        pnh_.param<std::string>("parent_frame", parent_frame_, "base_link");
        
        // 加载并计算变换矩阵
        loadCalibrationParams();

        pub_points_raw_ = nh_.advertise<PointCloud2>("/points_raw", 10);

        sub_16_ = new Subscriber<PointCloud2>(nh_, "/points_16", 1, ros::TransportHints().tcpNoDelay());
        sub_mid_ = new Subscriber<PointCloud2>(nh_, "/points_mid", 1, ros::TransportHints().tcpNoDelay());
        sub_left_ = new Subscriber<LaserScan>(nh_, "/scan_left", 1, ros::TransportHints().tcpNoDelay());
        sub_right_ = new Subscriber<LaserScan>(nh_, "/scan_right", 1, ros::TransportHints().tcpNoDelay());

        sync_ = new Synchronizer<SyncPolicy>(SyncPolicy(10), *sub_16_, *sub_mid_, *sub_left_, *sub_right_);
        sync_->registerCallback(boost::bind(&SensorInputNode::syncCallback, this, _1, _2, _3, _4));

        ROS_INFO("\033[1;32m[Sensor Input] Node initialized. Waiting for sensor data...\033[0m");
    }

    ~SensorInputNode() {
        delete sync_; delete sub_16_; delete sub_mid_; delete sub_left_; delete sub_right_;
    }

private:
    ros::NodeHandle nh_, pnh_;
    std::string parent_frame_;
    laser_geometry::LaserProjection projector_;
    ros::Publisher pub_points_raw_;

    Subscriber<PointCloud2>* sub_16_;
    Subscriber<PointCloud2>* sub_mid_;
    Subscriber<LaserScan>* sub_left_;
    Subscriber<LaserScan>* sub_right_;

    typedef sync_policies::ApproximateTime<PointCloud2, PointCloud2, LaserScan, LaserScan> SyncPolicy;
    Synchronizer<SyncPolicy>* sync_;

    // 存储最终到 base_link 的绝对变换矩阵
    Eigen::Affine3f trans_main_, trans_mid_, trans_left_, trans_right_;

    void loadCalibrationParams() {
        // 1. 读取基础参数
        Eigen::Affine3f trans_main_to_base  = getTransformFromParam("calibration/main");
        Eigen::Affine3f trans_mid_to_velo   = getTransformFromParam("calibration/mid");
        Eigen::Affine3f trans_left_to_velo  = getTransformFromParam("calibration/left");
        Eigen::Affine3f trans_right_to_velo = getTransformFromParam("calibration/right");

        // 2. 计算绝对变换矩阵 (串联变换链)
        // 主雷达直接到 base_link
        trans_main_  = trans_main_to_base; 
        
        // 辅助雷达: 先到 velodyne，再跟随着 velodyne 一起转到 base_link
        // 在 Eigen 中，T_final = T_parent * T_child
        trans_mid_   = trans_main_to_base * trans_mid_to_velo;
        trans_left_  = trans_main_to_base * trans_left_to_velo;
        trans_right_ = trans_main_to_base * trans_right_to_velo;

        ROS_INFO("\033[1;34m[Sensor Input] Calibration matrices loaded & chained successfully.\033[0m");
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
        // 注意旋转顺序：先 Roll (X)，再 Pitch (Y)，最后 Yaw (Z)，等价于 Rz * Ry * Rx
        transform.rotate(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));
        transform.rotate(Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()));
        transform.rotate(Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()));
        return transform;
    }

    void processScan(const LaserScan::ConstPtr& scan_msg, const Eigen::Affine3f& transform, pcl::PointCloud<pcl::PointXYZI>::Ptr& out_cloud) {
        PointCloud2 temp_cloud2;
        try { projector_.projectLaser(*scan_msg, temp_cloud2); } catch (...) { return; }
        pcl::PointCloud<pcl::PointXYZI> raw_pcl;
        pcl::fromROSMsg(temp_cloud2, raw_pcl);
        pcl::transformPointCloud(raw_pcl, *out_cloud, transform);
    }

    void processCloud(const PointCloud2::ConstPtr& cloud_msg, const Eigen::Affine3f& transform, pcl::PointCloud<pcl::PointXYZI>::Ptr& out_cloud) {
        pcl::PointCloud<pcl::PointXYZI> raw_pcl;
        pcl::fromROSMsg(*cloud_msg, raw_pcl);
        pcl::transformPointCloud(raw_pcl, *out_cloud, transform);
    }

    void syncCallback(const PointCloud2::ConstPtr& msg_16, 
                      const PointCloud2::ConstPtr& msg_mid, 
                      const LaserScan::ConstPtr& msg_left, 
                      const LaserScan::ConstPtr& msg_right) 
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_main(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_mid(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_left(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_right(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZI>());

        // 所有点云都被转换到了统一的 parent_frame_ (base_link) 下
        processCloud(msg_16, trans_main_, cloud_main);
        processCloud(msg_mid, trans_mid_, cloud_mid);
        processScan(msg_left, trans_left_, cloud_left);
        processScan(msg_right, trans_right_, cloud_right);

        merged_cloud->reserve(cloud_main->size() + cloud_mid->size() + cloud_left->size() + cloud_right->size());
        *merged_cloud += *cloud_main;
        *merged_cloud += *cloud_mid;
        *merged_cloud += *cloud_left;
        *merged_cloud += *cloud_right;

        if (pub_points_raw_.getNumSubscribers() > 0) {
            PointCloud2 output_msg;
            pcl::toROSMsg(*merged_cloud, output_msg);
            output_msg.header.stamp = msg_16->header.stamp;
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