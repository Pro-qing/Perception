/**
 * @file lidar_map_filter.h
 * @brief 基于地图点云 KD-Tree 的静态背景过滤器
 *
 * 功能:
 *   1. 启动时加载静态地图 PCD 文件
 *   2. 支持两种地图构建模式: 全地图 / 局部地图(以 velodyne 为中心裁剪)
 *   3. 支持两种搜索模式: 最近邻搜索 / 半径搜索
 *   4. 滤除与地图重合的背景点(墙壁/树木等)，保留新障碍物点
 *   5. 发布调试信息: 背景点、局部地图
 */

#ifndef LIDAR_MAP_FILTER_H
#define LIDAR_MAP_FILTER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/crop_box.h>

#include <lidar_pipeline_monitor/PipelineMetrics.h>

#include <string>

class LidarMapFilterNode {
public:
    LidarMapFilterNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);

private:
    ros::NodeHandle nh_, pnh_;

    // ---- 发布者 ----
    ros::Publisher pub_filtered_;       // 过滤后的障碍物点云
    ros::Publisher pub_background_;     // 被滤除的背景点云 (调试)
    ros::Publisher pub_local_map_;      // 局部地图点云 (调试)
    ros::Publisher pub_metrics_;        // 性能监控

    // ---- 订阅者 ----
    ros::Subscriber sub_points_;

    // ---- 话题参数 ----
    std::string input_topic_;
    std::string output_topic_;
    std::string background_topic_;
    std::string local_map_topic_;

    // ---- 地图参数 ----
    std::string map_pcd_path_;

    // ---- 搜索模式参数 ----
    std::string search_mode_;    // "nearest" 或 "radius"
    double distance_threshold_;  // 最近邻距离阈值 / 半径搜索半径

    // ---- 地图构建模式参数 ----
    bool use_local_map_;
    double local_map_x_front_;
    double local_map_x_rear_;
    double local_map_y_left_;
    double local_map_y_right_;

    // ---- 地图数据 ----
    pcl::PointCloud<pcl::PointXYZI>::Ptr full_map_cloud_;  // 完整静态地图

    // ---- 核心处理函数 ----
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

    // ---- 辅助函数 ----
    /**
     * @brief 裁剪地图到以原点为中心的局部区域
     * @param input_cloud  输入完整地图
     * @param x_front      前方范围(米)
     * @param x_rear       后方范围(米)
     * @param y_left       左侧范围(米)
     * @param y_right      右侧范围(米)
     * @return 裁剪后的局部地图
     */
    pcl::PointCloud<pcl::PointXYZI>::Ptr cropLocalMap(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
        double x_front, double x_rear,
        double y_left, double y_right);

    /**
     * @brief 最近邻搜索模式过滤
     * @param cloud        输入当前帧点云
     * @param kdtree       已构建的 KD-Tree
     * @param filtered     输出: 障碍物点
     * @param background   输出: 背景点
     */
    void filterByNearest(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
        pcl::KdTreeFLANN<pcl::PointXYZI>& kdtree,
        pcl::PointCloud<pcl::PointXYZI>::Ptr& filtered,
        pcl::PointCloud<pcl::PointXYZI>::Ptr& background);

    /**
     * @brief 半径搜索模式过滤
     * @param cloud        输入当前帧点云
     * @param kdtree       已构建的 KD-Tree
     * @param filtered     输出: 障碍物点
     * @param background   输出: 背景点
     */
    void filterByRadius(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
        pcl::KdTreeFLANN<pcl::PointXYZI>& kdtree,
        pcl::PointCloud<pcl::PointXYZI>::Ptr& filtered,
        pcl::PointCloud<pcl::PointXYZI>::Ptr& background);
};

#endif // LIDAR_MAP_FILTER_H