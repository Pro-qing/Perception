#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <autoware_msgs/CloudCluster.h>
#include <autoware_msgs/CloudClusterArray.h>
#include <autoware_msgs/DetectedObject.h>
#include <autoware_msgs/DetectedObjectArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>

#include <lidar_pipeline_monitor/PipelineMetrics.h>

#include <vector>
#include <cmath>
#include <string>

// ============================================================
//  Euclidean Cluster Detector
//
//  算法原理:
//    1. 订阅去地面点云 (/lidar_no_ground)
//    2. 使用 PCL EuclideanClusterExtraction 进行欧几里得聚类
//    3. 对每个簇计算包围盒 (AABB)、质心、尺寸
//    4. 按参数过滤无效障碍物 (尺寸/距离/高度)
//    5. 发布 CloudClusterArray 和 DetectedObjectArray
// ============================================================

class LidarEuclideanClusterDetectorNode {
public:
    LidarEuclideanClusterDetectorNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh), pnh_(pnh), cluster_id_counter_(0)
    {
        // ---- 从 YAML 读取话题参数 ----
        pnh_.param<std::string>("input_topic",            input_topic_,            "/lidar_no_ground");
        pnh_.param<std::string>("cloud_clusters_topic",   cloud_clusters_topic_,   "/detection/lidar_detector/cloud_clusters");
        pnh_.param<std::string>("objects_topic",          objects_topic_,          "/detection/lidar_detector/objects");

        // ---- 从 YAML 读取聚类参数 ----
        pnh_.param<double>("cluster_tolerance", cluster_tolerance_, 0.8);
        pnh_.param<int>   ("min_cluster_size",  min_cluster_size_,  10);
        pnh_.param<int>   ("max_cluster_size",  max_cluster_size_,  5000);
        pnh_.param<double>("kdtree_eps",        kdtree_eps_,        0.0);

        // ---- 从 YAML 读取障碍物过滤参数 ----
        pnh_.param<bool>  ("obstacle_filter/enable",     filter_enable_,   true);
        pnh_.param<double>("obstacle_filter/min_height", filter_min_height_, -2.0);
        pnh_.param<double>("obstacle_filter/max_height", filter_max_height_,  3.0);
        pnh_.param<double>("obstacle_filter/min_width",  filter_min_width_,   0.2);
        pnh_.param<double>("obstacle_filter/max_width",  filter_max_width_,  50.0);
        pnh_.param<double>("obstacle_filter/min_length", filter_min_length_,  0.2);
        pnh_.param<double>("obstacle_filter/max_length", filter_max_length_, 50.0);
        pnh_.param<double>("obstacle_filter/max_distance", filter_max_distance_, 150.0);

        // ---- 发布者 ----
        pub_cloud_clusters_ = nh_.advertise<autoware_msgs::CloudClusterArray>(cloud_clusters_topic_, 10);
        pub_objects_        = nh_.advertise<autoware_msgs::DetectedObjectArray>(objects_topic_, 10);

        pub_metrics_ = nh_.advertise<lidar_pipeline_monitor::PipelineMetrics>("/pipeline/metrics", 100);

        // ---- 订阅者 ----
        sub_points_ = nh_.subscribe(input_topic_, 10,
            &LidarEuclideanClusterDetectorNode::pointCloudCallback, this);

        ROS_INFO("\033[1;32m[Lidar Euclidean Cluster Detector] Node initialized.\033[0m");
        ROS_INFO("  input_topic:          %s", input_topic_.c_str());
        ROS_INFO("  cloud_clusters_topic: %s", cloud_clusters_topic_.c_str());
        ROS_INFO("  objects_topic:        %s", objects_topic_.c_str());
        ROS_INFO("  cluster_tolerance:    %.3f m", cluster_tolerance_);
        ROS_INFO("  min_cluster_size:     %d", min_cluster_size_);
        ROS_INFO("  max_cluster_size:     %d", max_cluster_size_);
        ROS_INFO("  kdtree_eps:           %.3f", kdtree_eps_);
        ROS_INFO("  obstacle_filter:      %s", filter_enable_ ? "true" : "false");
        if (filter_enable_) {
            ROS_INFO("  height range:         [%.2f, %.2f] m", filter_min_height_, filter_max_height_);
            ROS_INFO("  width range:          [%.2f, %.2f] m", filter_min_width_,  filter_max_width_);
            ROS_INFO("  length range:         [%.2f, %.2f] m", filter_min_length_, filter_max_length_);
            ROS_INFO("  max_distance:         %.2f m", filter_max_distance_);
        }
    }

private:
    ros::NodeHandle nh_, pnh_;
    ros::Publisher  pub_cloud_clusters_;
    ros::Publisher  pub_objects_;
    ros::Subscriber sub_points_;

    ros::Publisher pub_metrics_; 

    // 话题参数
    std::string input_topic_;
    std::string cloud_clusters_topic_;
    std::string objects_topic_;

    // 聚类参数
    double cluster_tolerance_;
    int    min_cluster_size_;
    int    max_cluster_size_;
    double kdtree_eps_;

    // 障碍物过滤参数
    bool   filter_enable_;
    double filter_min_height_;
    double filter_max_height_;
    double filter_min_width_;
    double filter_max_width_;
    double filter_min_length_;
    double filter_max_length_;
    double filter_max_distance_;

    // 簇 ID 计数器
    uint32_t cluster_id_counter_;

    // ============== 构建 CloudCluster 消息 ==============
    autoware_msgs::CloudCluster buildCloudCluster(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster_cloud,
        const std_msgs::Header& header,
        uint32_t id)
    {
        autoware_msgs::CloudCluster cluster_msg;
        cluster_msg.header = header;
        cluster_msg.id     = id;
        cluster_msg.label  = "unknown";
        cluster_msg.score  = 0.0;

        // 将簇点云转换为 ROS 消息
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cluster_cloud, cloud_msg);
        cloud_msg.header = header;
        cluster_msg.cloud = cloud_msg;

        // 计算 AABB 包围盒
        pcl::PointXYZI min_pt, max_pt;
        pcl::getMinMax3D(*cluster_cloud, min_pt, max_pt);

        // min_point
        geometry_msgs::PointStamped min_point;
        min_point.header = header;
        min_point.point.x = min_pt.x;
        min_point.point.y = min_pt.y;
        min_point.point.z = min_pt.z;
        cluster_msg.min_point = min_point;

        // max_point
        geometry_msgs::PointStamped max_point;
        max_point.header = header;
        max_point.point.x = max_pt.x;
        max_point.point.y = max_pt.y;
        max_point.point.z = max_pt.z;
        cluster_msg.max_point = max_point;

        // 计算质心 (centroid)
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cluster_cloud, centroid);

        geometry_msgs::PointStamped centroid_point;
        centroid_point.header = header;
        centroid_point.point.x = centroid[0];
        centroid_point.point.y = centroid[1];
        centroid_point.point.z = centroid[2];
        cluster_msg.centroid_point = centroid_point;

        // avg_point (与质心相同)
        geometry_msgs::PointStamped avg_point;
        avg_point.header = header;
        avg_point.point.x = centroid[0];
        avg_point.point.y = centroid[1];
        avg_point.point.z = centroid[2];
        cluster_msg.avg_point = avg_point;

        // 计算尺寸 (dimensions): width(Y), length(X), height(Z)
        double dx = max_pt.x - min_pt.x;
        double dy = max_pt.y - min_pt.y;
        double dz = max_pt.z - min_pt.z;

        geometry_msgs::Vector3 dimensions;
        dimensions.x = dx;  // length
        dimensions.y = dy;  // width
        dimensions.z = dz;  // height
        cluster_msg.dimensions = dimensions;

        // 计算朝向角度 (基于协方差矩阵的主方向)
        cluster_msg.estimated_angle = 0.0;

        // 计算协方差矩阵特征值和特征向量
        if (cluster_cloud->size() >= 3) {
            Eigen::Matrix3f covariance;
            Eigen::Vector4f centroid_for_cov;
            pcl::compute3DCentroid(*cluster_cloud, centroid_for_cov);
            pcl::computeCovarianceMatrix(*cluster_cloud, centroid_for_cov, covariance);

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
            Eigen::Vector3f eigen_values = solver.eigenvalues();
            Eigen::Matrix3f eigen_vectors = solver.eigenvectors();

            geometry_msgs::Vector3 ev;
            ev.x = eigen_values[0];
            ev.y = eigen_values[1];
            ev.z = eigen_values[2];
            cluster_msg.eigen_values = ev;

            cluster_msg.eigen_vectors.resize(3);
            for (int i = 0; i < 3; ++i) {
                geometry_msgs::Vector3 vec;
                vec.x = eigen_vectors(0, i);
                vec.y = eigen_vectors(1, i);
                vec.z = eigen_vectors(2, i);
                cluster_msg.eigen_vectors[i] = vec;
            }

            // 使用主特征向量计算朝向角 (绕 Z 轴)
            double angle = std::atan2(eigen_vectors(1, 2), eigen_vectors(0, 2));
            cluster_msg.estimated_angle = angle;
        }

        cluster_msg.indicator_state = 3;  // INDICATOR_NONE

        return cluster_msg;
    }

    // ============== 判断障碍物是否有效 ==============
    bool isValidObstacle(const autoware_msgs::CloudCluster& cluster) {
        if (!filter_enable_) {
            return true;
        }

        double length = cluster.dimensions.x;
        double width  = cluster.dimensions.y;
        double height = cluster.dimensions.z;

        // 高度过滤
        if (height < filter_min_height_ || height > filter_max_height_) {
            return false;
        }

        // 宽度过滤
        if (width < filter_min_width_ || width > filter_max_width_) {
            return false;
        }

        // 长度过滤
        if (length < filter_min_length_ || length > filter_max_length_) {
            return false;
        }

        // 距离过滤 (基于质心到原点的距离)
        double cx = cluster.centroid_point.point.x;
        double cy = cluster.centroid_point.point.y;
        double distance = std::sqrt(cx * cx + cy * cy);
        if (distance > filter_max_distance_) {
            return false;
        }

        return true;
    }

    // ============== 主回调 ==============
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        
        ros::Time cb_start = ros::Time::now(); // 【新增头】

        bool has_cluster_sub = (pub_cloud_clusters_.getNumSubscribers() > 0);
        bool has_object_sub  = (pub_objects_.getNumSubscribers() > 0);

        if (!has_cluster_sub && !has_object_sub) {
            return;
        }

        // 转换为 PCL 点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->empty()) {
            ROS_WARN_THROTTLE(5.0, "[Euclidean Cluster] Received empty cloud, skipping.");
            return;
        }

        // ---- 构建 KD-Tree ----
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
        tree->setInputCloud(cloud);
        tree->setEpsilon(kdtree_eps_);

        // ---- 欧几里得聚类提取 ----
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance(cluster_tolerance_);
        ec.setMinClusterSize(min_cluster_size_);
        ec.setMaxClusterSize(max_cluster_size_);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        // ---- 准备发布消息 ----
        autoware_msgs::CloudClusterArray cloud_cluster_array;
        cloud_cluster_array.header = msg->header;

        autoware_msgs::DetectedObjectArray object_array;
        object_array.header = msg->header;

        int valid_count = 0;

        for (const auto& indices : cluster_indices) {
            // 提取单个簇的点云
            pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZI>());
            cluster_cloud->points.reserve(indices.indices.size());
            for (const auto& idx : indices.indices) {
                cluster_cloud->points.push_back(cloud->points[idx]);
            }
            cluster_cloud->width    = cluster_cloud->points.size();
            cluster_cloud->height   = 1;
            cluster_cloud->is_dense = true;

            // 构建 CloudCluster 消息
            uint32_t cluster_id = cluster_id_counter_++;
            autoware_msgs::CloudCluster cluster_msg = buildCloudCluster(cluster_cloud, msg->header, cluster_id);

            // 障碍物有效性过滤
            if (!isValidObstacle(cluster_msg)) {
                continue;
            }

            valid_count++;

            // 添加到 CloudClusterArray
            cloud_cluster_array.clusters.push_back(cluster_msg);

            // 构建 DetectedObject 消息
            autoware_msgs::DetectedObject obj;
            obj.header   = msg->header;
            obj.id       = cluster_id;
            obj.label    = "unknown";
            obj.score    = 0.0;
            obj.valid    = true;
            obj.space_frame = msg->header.frame_id;

            // 位姿 = 质心
            obj.pose.position.x = cluster_msg.centroid_point.point.x;
            obj.pose.position.y = cluster_msg.centroid_point.point.y;
            obj.pose.position.z = cluster_msg.centroid_point.point.z;
            obj.pose.orientation.w = 1.0;

            // 尺寸
            obj.dimensions = cluster_msg.dimensions;

            // 点云
            obj.pointcloud = cluster_msg.cloud;

            // 速度 (无估计, 置零)
            obj.velocity.linear.x = 0.0;
            obj.velocity.linear.y = 0.0;
            obj.velocity.linear.z = 0.0;
            obj.acceleration.linear.x = 0.0;
            obj.acceleration.linear.y = 0.0;
            obj.acceleration.linear.z = 0.0;

            obj.pose_reliable         = true;
            obj.velocity_reliable     = false;
            obj.acceleration_reliable = false;

            obj.indicator_state = 3;  // INDICATOR_NONE
            obj.behavior_state  = 0;  // FORWARD_STATE

            object_array.objects.push_back(obj);
        }

        // ---- 发布结果 ----
        if (has_cluster_sub) {
            pub_cloud_clusters_.publish(cloud_cluster_array);
        }

        if (has_object_sub) {
            pub_objects_.publish(object_array);
        }

        // 【新增尾】
        ros::Time cb_end = ros::Time::now();
        lidar_pipeline_monitor::PipelineMetrics metric;
        metric.header.stamp = msg->header.stamp; 
        metric.node_name = "4_cluster";
        metric.transmission_delay = (cb_start - msg->header.stamp).toSec() * 1000.0;
        metric.processing_time = (cb_end - cb_start).toSec() * 1000.0;
        metric.total_latency = (cb_end - msg->header.stamp).toSec() * 1000.0;
        pub_metrics_.publish(metric);

        ROS_INFO_THROTTLE(2.0,
            "[Euclidean Cluster] Input points: %lu, Clusters found: %lu, Valid obstacles: %d",
            cloud->size(), cluster_indices.size(), valid_count);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_euclidean_cluster_detector_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    LidarEuclideanClusterDetectorNode node(nh, pnh);

    ros::spin();

    return 0;
}