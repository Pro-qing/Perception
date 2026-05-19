#ifndef LIDAR_INPUT_DOWNSAMPLE_NOGROUND_NO_GROUND_H
#define LIDAR_INPUT_DOWNSAMPLE_NOGROUND_NO_GROUND_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>

#include <vector>
#include <cmath>
#include <algorithm>

class NoGroundProcessor {
public:
    NoGroundProcessor(ros::NodeHandle& nh, ros::NodeHandle& pnh);

    /**
     * @brief 处理地面分割，直接操作 PCL 点云 (避免 ROS↔PCL 转换)
     * @param cloud        [输入] 待处理点云
     * @param ground_cloud [输出] 地面点云
     * @param no_ground_cloud [输出] 非地面点云
     */
    void processNoGround(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
        pcl::PointCloud<pcl::PointXYZI>::Ptr& ground_cloud,
        pcl::PointCloud<pcl::PointXYZI>::Ptr& no_ground_cloud);

private:
    ros::NodeHandle nh_, pnh_;

    // RANSAC 参数
    int    ransac_max_iterations_;
    double ransac_distance_threshold_;
    double ransac_probability_;
    double ransac_eps_angle_;
    double ransac_eps_angle_rad_;
    bool   ransac_optimize_coeff_;
    bool   use_perpendicular_;

    // 地面约束参数
    double ground_max_height_;
    double ground_min_height_;

    // 迭代拟合参数
    bool   iterative_enable_;
    int    iterative_max_iters_;
    double iterative_height_thresh_;

    // 预过滤参数
    bool   pre_filter_enable_;
    double pre_filter_min_z_;
    double pre_filter_max_z_;

    bool segmentGroundPlane(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
        pcl::PointIndices::Ptr& inlier_indices,
        pcl::ModelCoefficients::Ptr& coefficients);

    void filterGroundByHeight(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
        const pcl::PointIndices::Ptr& inlier_indices,
        pcl::PointIndices::Ptr& true_ground_indices,
        pcl::PointIndices::Ptr& remaining_indices);
};

#endif // LIDAR_INPUT_DOWNSAMPLE_NOGROUND_NO_GROUND_H