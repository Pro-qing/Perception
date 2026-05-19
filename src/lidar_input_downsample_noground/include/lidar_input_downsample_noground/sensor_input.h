#ifndef LIDAR_INPUT_DOWNSAMPLE_NOGROUND_SENSOR_INPUT_H
#define LIDAR_INPUT_DOWNSAMPLE_NOGROUND_SENSOR_INPUT_H

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
#include <lidar_pipeline_monitor/PipelineMetrics.h>

#include <string>
#include <map>
#include <mutex>

class SensorInputProcessor {
public:
    SensorInputProcessor(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~SensorInputProcessor();

    /**
     * @brief 设置行为模式 (线程安全)
     * @param sensor_enabled 传感器启用状态
     */
    void setSensorEnabled(const std::map<std::string, bool>& sensor_enabled);

    /**
     * @brief 处理传感器输入，将四路点云合并变换到 velodyne 坐标系
     * @param msg_16    主雷达 (16线) 消息
     * @param msg_mid   中置雷达消息
     * @param msg_left  左侧单线雷达消息
     * @param msg_right 右侧单线雷达消息
     * @param mutex     保护 sensor_enabled_ 的互斥锁
     * @param merged_cloud [输出] 合并后的点云 (velodyne 坐标系)
     * @param stamp     [输出] 时间戳
     * @return true 如果成功处理
     */
    bool processInput(
        const sensor_msgs::PointCloud2::ConstPtr& msg_16,
        const sensor_msgs::PointCloud2::ConstPtr& msg_mid,
        const sensor_msgs::LaserScan::ConstPtr& msg_left,
        const sensor_msgs::LaserScan::ConstPtr& msg_right,
        std::mutex& mutex,
        pcl::PointCloud<pcl::PointXYZI>::Ptr& merged_cloud,
        ros::Time& stamp);

    /** @brief 获取行为模式 (线程安全) */
    std::map<std::string, bool> getSensorEnabled() const;

private:
    ros::NodeHandle nh_, pnh_;
    std::string parent_frame_;
    laser_geometry::LaserProjection projector_;

    ros::Publisher pub_main_calib_, pub_mid_calib_, pub_left_calib_, pub_right_calib_;
    tf2_ros::StaticTransformBroadcaster static_broadcaster_;

    // 变换矩阵 (相对于 velodyne 坐标系)
    Eigen::Affine3f trans_main_, trans_mid_, trans_left_, trans_right_;
    Eigen::Affine3f base_to_velo_;

    // 行为模式
    std::map<int, std::map<std::string, bool>> behavior_configs_;
    std::map<std::string, bool> sensor_enabled_;

    // 单线雷达距离过滤半径
    double filter_radius_left_, filter_radius_right_;

    void loadCalibrationParams();
    void publishSensorTF();
    Eigen::Affine3f getTransformFromParam(const std::string& param_ns);
    void loadBehaviorConfigs();

    void processScan(const sensor_msgs::LaserScan::ConstPtr& scan_msg,
                     const Eigen::Affine3f& transform,
                     pcl::PointCloud<pcl::PointXYZI>::Ptr& out_cloud);

    void filterByRadius(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, double max_radius);

    void processCloud(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
                      const Eigen::Affine3f& transform,
                      pcl::PointCloud<pcl::PointXYZI>::Ptr& out_cloud);

    void publishCalibratedCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                                const ros::Time& stamp, const ros::Publisher& pub);
};

#endif // LIDAR_INPUT_DOWNSAMPLE_NOGROUND_SENSOR_INPUT_H