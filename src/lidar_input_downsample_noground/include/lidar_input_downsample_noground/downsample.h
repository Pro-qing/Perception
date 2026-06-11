#ifndef LIDAR_INPUT_DOWNSAMPLE_NOGROUND_DOWNSAMPLE_H
#define LIDAR_INPUT_DOWNSAMPLE_NOGROUND_DOWNSAMPLE_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>

#include <vector>
#include <cmath>

struct Point2D {
    double x, y;
};

class DownsampleProcessor {
public:
    DownsampleProcessor(ros::NodeHandle& nh, ros::NodeHandle& pnh);

    /**
     * @brief 处理降采样，直接操作 PCL 点云 (避免 ROS↔PCL 转换)
     * @param cloud [输入/输出] 点云，原地处理
     * @return true 如果处理后点云非空
     */
    bool processDownsample(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);

private:
    ros::NodeHandle nh_, pnh_;

    // VoxelGrid 参数
    double leaf_size_x_, leaf_size_y_, leaf_size_z_;
    int min_points_per_voxel_;
    bool downsample_all_data_;

    // Body Filter 参数
    bool body_filter_enable_;
    double body_min_z_, body_max_z_;
    std::vector<Point2D> body_polygon_;

    // Marker 参数
    bool publish_marker_;
    std::string marker_topic_;
    double marker_r_, marker_g_, marker_b_, marker_a_;
    ros::Publisher pub_marker_array_;

    ros::WallTimer timer_marker_;
    double marker_timer_rate_;

    // CropBox 参数
    bool crop_box_enable_;
    double crop_min_x_, crop_max_x_;
    double crop_min_y_, crop_max_y_;
    double crop_min_z_, crop_max_z_;
    bool crop_negative_;

    // 高度过滤参数
    bool height_filter_enable_;
    double min_height_, max_height_;

    void loadBodyPolygon();
    bool isPointInsidePolygon(double px, double py) const;
    void timerCallback(const ros::WallTimerEvent& event);
    void publishBodyMarker();
};

#endif // LIDAR_INPUT_DOWNSAMPLE_NOGROUND_DOWNSAMPLE_H