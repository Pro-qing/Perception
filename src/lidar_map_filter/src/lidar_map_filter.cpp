/**
 * @file lidar_map_filter.cpp
 * @brief 基于地图点云 KD-Tree 的静态背景过滤器实现
 *
 * 算法流程:
 *   1. 启动时加载静态地图 PCD 文件到内存
 *   2. 每帧回调:
 *      a. 根据地图构建模式, 决定使用全地图或局部裁剪地图
 *      b. 对地图构建 KD-Tree
 *      c. 根据搜索模式(最近邻/半径搜索), 对当前帧每个点查询地图
 *      d. 距离 < 阈值 → 背景(滤除), 距离 ≥ 阈值 → 新障碍物(保留)
 *   3. 发布: 过滤后点云、背景点云(调试)、局部地图(调试)
 */

#include "lidar_map_filter/lidar_map_filter.h"

// ============================================================
//  构造函数
// ============================================================
LidarMapFilterNode::LidarMapFilterNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh)
{
    // ---- 从 YAML 读取话题参数 ----
    pnh_.param<std::string>("input_topic",      input_topic_,      "/lidar_no_ground");
    pnh_.param<std::string>("output_topic",     output_topic_,     "/lidar_map_filtered");
    pnh_.param<std::string>("background_topic", background_topic_, "/lidar_map_background");
    pnh_.param<std::string>("local_map_topic",  local_map_topic_,  "/lidar_local_map");

    // ---- 从 YAML 读取地图参数 ----
    pnh_.param<std::string>("map_pcd_path", map_pcd_path_, "");

    // ---- 从 YAML 读取搜索模式参数 ----
    pnh_.param<std::string>("search_mode",       search_mode_,       "nearest");
    pnh_.param<double>     ("distance_threshold", distance_threshold_, 0.2);

    // ---- 从 YAML 读取地图构建模式参数 ----
    pnh_.param<bool>  ("use_local_map",     use_local_map_,      true);
    pnh_.param<double>("local_map_x_front", local_map_x_front_,  5.0);
    pnh_.param<double>("local_map_x_rear",  local_map_x_rear_,   3.0);
    pnh_.param<double>("local_map_y_left",  local_map_y_left_,   2.5);
    pnh_.param<double>("local_map_y_right", local_map_y_right_,  2.5);

    // ---- 参数验证 ----
    if (map_pcd_path_.empty()) {
        ROS_ERROR("[Lidar Map Filter] map_pcd_path is empty! Please set the PCD file path.");
        ros::shutdown();
        return;
    }

    if (search_mode_ != "nearest" && search_mode_ != "radius") {
        ROS_WARN("[Lidar Map Filter] Unknown search_mode '%s', defaulting to 'nearest'.",
                 search_mode_.c_str());
        search_mode_ = "nearest";
    }

    // ---- 加载静态地图 PCD 文件 ----
    full_map_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(map_pcd_path_, *full_map_cloud_) == -1) {
        ROS_ERROR("[Lidar Map Filter] Failed to load PCD file: %s", map_pcd_path_.c_str());
        ros::shutdown();
        return;
    }

    ROS_INFO("\033[1;32m[Lidar Map Filter] Node initialized.\033[0m");
    ROS_INFO("  map_pcd_path:        %s (%lu points)",
             map_pcd_path_.c_str(), full_map_cloud_->size());
    ROS_INFO("  input_topic:         %s", input_topic_.c_str());
    ROS_INFO("  output_topic:        %s", output_topic_.c_str());
    ROS_INFO("  background_topic:    %s", background_topic_.c_str());
    ROS_INFO("  local_map_topic:     %s", local_map_topic_.c_str());
    ROS_INFO("  search_mode:         %s", search_mode_.c_str());
    ROS_INFO("  distance_threshold:  %.3f m", distance_threshold_);
    ROS_INFO("  use_local_map:       %s", use_local_map_ ? "true" : "false");
    if (use_local_map_) {
        ROS_INFO("  local_map range:     front=%.1f, rear=%.1f, left=%.1f, right=%.1f m",
                 local_map_x_front_, local_map_x_rear_,
                 local_map_y_left_, local_map_y_right_);
    }

    // ---- 发布者 ----
    pub_filtered_   = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 10);
    pub_background_ = nh_.advertise<sensor_msgs::PointCloud2>(background_topic_, 10);
    pub_local_map_  = nh_.advertise<sensor_msgs::PointCloud2>(local_map_topic_, 10);
    pub_metrics_    = nh_.advertise<lidar_pipeline_monitor::PipelineMetrics>("/pipeline/metrics", 100);

    // ---- 订阅者 ----
    sub_points_ = nh_.subscribe(input_topic_, 10, &LidarMapFilterNode::pointCloudCallback, this);
}

// ============================================================
//  局部地图裁剪
// ============================================================
pcl::PointCloud<pcl::PointXYZI>::Ptr LidarMapFilterNode::cropLocalMap(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
    double x_front, double x_rear,
    double y_left, double y_right)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cropped(new pcl::PointCloud<pcl::PointXYZI>());

    // CropBox: min(x, y, z, 1), max(x, y, z, 1)
    // x: [-rear, +front], y: [-right, +left], z: 不限制
    pcl::CropBox<pcl::PointXYZI> crop;
    crop.setInputCloud(input_cloud);
    crop.setMin(Eigen::Vector4f(-static_cast<float>(x_rear),
                                 -static_cast<float>(y_right),
                                 -100.0f, 1.0f));
    crop.setMax(Eigen::Vector4f( static_cast<float>(x_front),
                                  static_cast<float>(y_left),
                                  100.0f, 1.0f));
    crop.setNegative(false);  // 保留框内点
    crop.filter(*cropped);

    return cropped;
}

// ============================================================
//  最近邻搜索模式过滤
// ============================================================
void LidarMapFilterNode::filterByNearest(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    pcl::KdTreeFLANN<pcl::PointXYZI>& kdtree,
    pcl::PointCloud<pcl::PointXYZI>::Ptr& filtered,
    pcl::PointCloud<pcl::PointXYZI>::Ptr& background)
{
    filtered.reset(new pcl::PointCloud<pcl::PointXYZI>());
    background.reset(new pcl::PointCloud<pcl::PointXYZI>());
    filtered->reserve(cloud->size());
    background->reserve(cloud->size());

    std::vector<int> indices(1);
    std::vector<float> sqr_dists(1);

    for (size_t i = 0; i < cloud->size(); ++i) {
        const auto& pt = cloud->points[i];

        // 查找最近的1个点
        if (kdtree.nearestKSearch(pt, 1, indices, sqr_dists) > 0) {
            float dist = std::sqrt(sqr_dists[0]);

            if (dist < distance_threshold_) {
                // 最近距离 < 阈值 → 背景(墙壁/树木等), 滤除
                background->points.push_back(pt);
            } else {
                // 最近距离 ≥ 阈值 → 新障碍物, 保留
                filtered->points.push_back(pt);
            }
        } else {
            // KD-Tree 搜索失败, 保留该点
            filtered->points.push_back(pt);
        }
    }

    filtered->width    = filtered->points.size();
    filtered->height   = 1;
    filtered->is_dense = true;

    background->width    = background->points.size();
    background->height   = 1;
    background->is_dense = true;
}

// ============================================================
//  半径搜索模式过滤
// ============================================================
void LidarMapFilterNode::filterByRadius(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    pcl::KdTreeFLANN<pcl::PointXYZI>& kdtree,
    pcl::PointCloud<pcl::PointXYZI>::Ptr& filtered,
    pcl::PointCloud<pcl::PointXYZI>::Ptr& background)
{
    filtered.reset(new pcl::PointCloud<pcl::PointXYZI>());
    background.reset(new pcl::PointCloud<pcl::PointXYZI>());
    filtered->reserve(cloud->size());
    background->reserve(cloud->size());

    std::vector<int> indices;
    std::vector<float> sqr_dists;
    float radius = static_cast<float>(distance_threshold_);

    for (size_t i = 0; i < cloud->size(); ++i) {
        const auto& pt = cloud->points[i];

        // 在半径内搜索邻居
        int found = kdtree.radiusSearch(pt, radius, indices, sqr_dists);

        if (found > 0) {
            // 半径内找到地图点 → 背景, 滤除
            background->points.push_back(pt);
        } else {
            // 半径内无地图点 → 新障碍物, 保留
            filtered->points.push_back(pt);
        }
    }

    filtered->width    = filtered->points.size();
    filtered->height   = 1;
    filtered->is_dense = true;

    background->width    = background->points.size();
    background->height   = 1;
    background->is_dense = true;
}

// ============================================================
//  主回调函数
// ============================================================
void LidarMapFilterNode::pointCloudCallback(
    const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    ros::Time cb_start = ros::Time::now();

    // 至少一个下游订阅者时才处理
    bool has_filtered_sub   = (pub_filtered_.getNumSubscribers() > 0);
    bool has_background_sub = (pub_background_.getNumSubscribers() > 0);
    bool has_local_map_sub  = (pub_local_map_.getNumSubscribers() > 0);

    if (!has_filtered_sub && !has_background_sub && !has_local_map_sub) {
        return;
    }

    // 转换为 PCL 点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*msg, *cloud);

    if (cloud->empty()) {
        ROS_WARN_THROTTLE(5.0, "[Lidar Map Filter] Received empty cloud, skipping.");
        return;
    }

    // ---- 确定用于构建 KD-Tree 的地图点云 ----
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_for_kdtree;

    if (use_local_map_) {
        // 局部地图模式: 裁剪完整地图到 velodyne 周围的局部区域
        map_for_kdtree = cropLocalMap(full_map_cloud_,
                                       local_map_x_front_, local_map_x_rear_,
                                       local_map_y_left_, local_map_y_right_);

        // 发布局部地图 (调试)
        if (has_local_map_sub) {
            sensor_msgs::PointCloud2 local_map_msg;
            pcl::toROSMsg(*map_for_kdtree, local_map_msg);
            local_map_msg.header.stamp    = msg->header.stamp;
            local_map_msg.header.frame_id = msg->header.frame_id;
            pub_local_map_.publish(local_map_msg);
        }
    } else {
        // 全地图模式: 直接使用完整地图
        map_for_kdtree = full_map_cloud_;
    }

    if (map_for_kdtree->empty()) {
        ROS_WARN_THROTTLE(5.0, "[Lidar Map Filter] Map cloud is empty (use_local_map=%s), skipping.",
                          use_local_map_ ? "true" : "false");
        return;
    }

    // ---- 构建 KD-Tree ----
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(map_for_kdtree);

    // ---- 根据搜索模式进行过滤 ----
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr background_cloud;

    if (search_mode_ == "radius") {
        filterByRadius(cloud, kdtree, filtered_cloud, background_cloud);
    } else {
        filterByNearest(cloud, kdtree, filtered_cloud, background_cloud);
    }

    // ---- 发布过滤结果 ----
    if (has_filtered_sub && !filtered_cloud->empty()) {
        sensor_msgs::PointCloud2 filtered_msg;
        pcl::toROSMsg(*filtered_cloud, filtered_msg);
        filtered_msg.header.stamp    = msg->header.stamp;
        filtered_msg.header.frame_id = msg->header.frame_id;
        pub_filtered_.publish(filtered_msg);
    }

    if (has_background_sub && !background_cloud->empty()) {
        sensor_msgs::PointCloud2 background_msg;
        pcl::toROSMsg(*background_cloud, background_msg);
        background_msg.header.stamp    = msg->header.stamp;
        background_msg.header.frame_id = msg->header.frame_id;
        pub_background_.publish(background_msg);
    }

    // ---- 性能监控 ----
    ros::Time cb_end = ros::Time::now();
    lidar_pipeline_monitor::PipelineMetrics metric;
    metric.header.stamp = msg->header.stamp;
    metric.node_name = "4_map_filter";
    metric.transmission_delay = (cb_start - msg->header.stamp).toSec() * 1000.0;
    metric.processing_time = (cb_end - cb_start).toSec() * 1000.0;
    metric.total_latency = (cb_end - msg->header.stamp).toSec() * 1000.0;
    pub_metrics_.publish(metric);

    ROS_INFO_THROTTLE(5.0,
        "[Lidar Map Filter] input=%lu, filtered=%lu, background=%lu, map=%lu, time=%.1f ms",
        cloud->size(), filtered_cloud->size(), background_cloud->size(),
        map_for_kdtree->size(), metric.processing_time);
}