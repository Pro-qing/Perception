/**
 * @file obstacle_area_detection.hpp
 * @brief 库区障碍物检测主类头文件
 *
 * 该模块是自动驾驶感知系统中的库区障碍物检测节点，用于检测多个停车位（库位）
 * 区域内是否存在障碍物。仅使用mid360雷达的点云数据，通过预处理流水线后，
 * 对每个活跃库位进行独立的区域检测，并发布每个库位的检测状态。
 *
 * 数据处理流程:
 *   1. 接收mid360点云
 *   2. 点云预处理(标定→ROI→变换→降采样→Z轴滤波→地面分割→车体过滤)
 *   3. 遍历所有活跃库位，对每个库位进行区域检测
 *   4. 每个库位独立维护滑动窗口防抖
 *   5. 发布检测结果和可视化信息
 *
 * 扩展性设计:
 *   - 支持多库位同时检测(ParkingSpot结构体)
 *   - 每个库位独立维护检测状态和历史记录
 *   - 位标志输出兼容现有下游系统
 */
#ifndef __OBSTACLE_AREA_DETECTION_HPP_
#define __OBSTACLE_AREA_DETECTION_HPP_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <std_msgs/UInt32.h>
#include <autoware_msgs/KeyPointArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <memory>
#include <deque>
#include <map>
#include <sys/stat.h>

/** @brief 点类型别名，使用XYZI格式的PCL点 */
typedef pcl::PointXYZI PointT;
/** @brief 点云类型别名 */
typedef pcl::PointCloud<PointT> PointCloud;


/**
 * @brief 单个库位信息结构体
 *
 * 存储每个库位的标识、位姿和检测状态。
 * 支持多个库位同时存在，每个库位独立维护检测结果。
 */
struct ParkingSpot {
    std::string id;                        ///< 库位唯一标识(如 "carports_0", "carports_1")
    geometry_msgs::PoseStamped map_pose;   ///< 库位在map坐标系下的位姿
    bool has_obstacle;                     ///< 当前是否确认有障碍物(经过防抖后的最终结果)
    bool raw_detection;                    ///< 当前帧原始检测结果(未经防抖)
    bool enable;                           ///< 是否启用检测(基于距离判断)
};

/**
 * @brief 库区检测区域参数结构体
 *
 * 定义在目标点局部坐标系下的3D边界框，
 * 所有库位共享同一套区域参数。
 */
struct AreaBounds {
    double min_x, max_x;  ///< X轴范围(车辆前后方向)
    double min_y, max_y;  ///< Y轴范围(车辆左右方向)
    double min_z, max_z;  ///< Z轴范围(垂直方向)
};

/**
 * @brief 点云预处理结果结构体
 */
struct PreprocessResult {
    PointCloud::Ptr ground_cloud;    ///< 分割出的地面点云
    PointCloud::Ptr filtered_cloud;  ///< 过滤后的障碍物点云(已去除地面和车体)
    std_msgs::Header header;         ///< 消息头(包含时间戳和坐标系)
    bool valid;                      ///< 预处理是否成功
};

/**
 * @brief 库区障碍物检测主类
 *
 * 实现了多库位障碍物检测流水线，包括:
 * - mid360点云预处理(标定、ROI、降采样、地面分割、车体过滤)
 * - 多库位管理(从导航路径获取库位列表)
 * - 每个库位独立的区域障碍物检测
 * - 双层检测(Layer1内部区域 + Layer2外围聚类)
 * - 滑动窗口防抖 + 施密特触发器
 * - 检测结果发布与可视化
 */
class ObstacleAreaDetection {
private:
    ros::NodeHandle& nh_;           ///< 全局节点句柄
    ros::NodeHandle& private_nh_;   ///< 私有节点句柄(用于获取参数)

    // ============ 订阅者 ============
    ros::Subscriber mid_cloud_sub_;        ///< mid360点云订阅(单雷达，无需同步)
    ros::Subscriber keypoint_path_sub_;    ///< 导航关键点路径订阅(获取库位列表)
    ros::Subscriber current_pose_sub_;     ///< 当前车辆位姿订阅(用于距离判断)

    // ============ 发布者 ============
    ros::Publisher ground_pub_;              ///< 地面点云发布
    ros::Publisher obstacle_pub_;            ///< 障碍物点云发布
    ros::Publisher calibration_pub_;         ///< 标定后点云发布
    ros::Publisher fused_cloud_pub_;         ///< 预处理后点云发布
    ros::Publisher area_status_pub_;         ///< 库区检测状态发布(位标志)
    ros::Publisher area_marker_pub_;         ///< 库区可视化标记发布(边界框+状态文本)
    ros::Publisher target_region_cloud_pub_; ///< 目标区域内点云发布
    ros::Publisher proximity_cloud_pub_;    ///< 外围聚类检测点云发布
    ros::Publisher proximity_marker_pub_;   ///< 外围聚类检测可视化标记发布

    ros::Timer publish_timer_;               ///< 定时发布检测状态的定时器(10Hz)
    ros::Timer yaml_reload_timer_;           ///< yaml文件变更检查定时器(1Hz)
    tf::TransformListener tf_listener_;      ///< TF变换监听器

    // ============ 基础参数(从launch文件加载) ============
    std::string target_frame_;          ///< 目标坐标系(通常是velodyne)
    std::string points_mid_topic_;      ///< mid360点云topic名称
    double voxel_leaf_size_;            ///< 体素降采样叶子大小(0表示不降采样)
    double z_axis_min_;                 ///< Z轴滤波最小值
    double z_axis_max_;                 ///< Z轴滤波最大值
    double ground_threshold_;           ///< RANSAC地面分割距离阈值
    double roi_radius_;                 ///< ROI区域半径(米)
    bool use_roi_filter_;               ///< 是否启用ROI滤波
    double distance_threshold_;         ///< 到目标点的距离阈值，小于此值时关闭enable(避免自检)
    double area_enable_distance_;       ///< 库位检测启用距离(米)

    // ============ 防误判参数 ============
    int min_region_points_;             ///< 区域内最小点数阈值
    int area_history_size_;             ///< 滑动窗口大小(帧数)
    int area_confirm_threshold_;        ///< 确认阈值(窗口内需有N帧检测到才确认)
    int area_clear_threshold_;          ///< 清除阈值(窗口内需<=N帧检测到才清除)

    // ============ 外围聚类检测参数(Layer 2) ============
    bool enable_proximity_cluster_;           ///< 是否启用外围聚类检测
    double cluster_tolerance_;                ///< 聚类容差(米)
    int cluster_min_points_;                  ///< 最小聚类点数
    int cluster_max_points_;                  ///< 最大聚类点数
    double cluster_max_z_range_;              ///< 聚类z范围上限(米)
    double cluster_min_centroid_z_;           ///< 聚类重心最低高度(米)
    double expand_margin_x_;                  ///< X方向扩展边界(米)
    double expand_margin_y_;                  ///< Y方向扩展边界(米)
    double proximity_threshold_;              ///< 聚类到库位框最大距离(米)
    bool proximity_debug_;                    ///< 外围聚类调试日志开关

    // ============ YAML动态重载 ============
    std::string yaml_file_path_;        ///< yaml配置文件路径(从launch传入)
    time_t last_yaml_mod_time_;         ///< yaml文件上次修改时间戳

    // ============ 运行时状态 ============
    std::vector<ParkingSpot> parking_spots_;                              ///< 所有活跃库位列表
    std::map<std::string, std::deque<bool>> spot_detection_history_;      ///< 每个库位的滑动窗口历史
    std::map<std::string, bool> spot_confirmed_;                          ///< 每个库位的确认状态
    geometry_msgs::PoseStamped current_pose_;  ///< 当前车辆在map坐标系下的位姿

    // ============ 中距雷达标定参数(从yaml动态加载) ============
    double mid_x_;        ///< X轴平移偏移
    double mid_y_;        ///< Y轴平移偏移
    double mid_z_;        ///< Z轴平移偏移
    double mid_roll_;     ///< 绕X轴旋转偏移(弧度)
    double mid_pitch_;    ///< 绕Y轴旋转偏移(弧度)
    double mid_yaw_;      ///< 绕Z轴旋转偏移(弧度)

    // ============ 库位检测区域参数 ============
    AreaBounds area_bounds_;                     ///< 检测区域边界框

    // ============ 车体轮廓参数(用于过滤车体自身的点) ============
    std::vector<Eigen::Vector2f> car_vertices_;  ///< 车体XY平面多边形顶点
    double car_min_z_, car_max_z_;               ///< 车体Z轴范围

public:
    /**
     * @brief 构造函数
     * @param nh 全局节点句柄
     * @param private_nh 私有节点句柄
     *
     * 初始化所有参数、订阅者、发布者和定时器
     */
    ObstacleAreaDetection(ros::NodeHandle& nh, ros::NodeHandle& private_nh);

private:
    // ============ 核心回调 ============

    /**
     * @brief mid360点云回调 - 核心处理入口
     * @param msg mid360点云消息
     *
     * 处理流程:
     * 1. 预处理点云
     * 2. 遍历所有活跃库位
     * 3. 对每个库位进行区域检测
     * 4. 更新每个库位的防抖状态
     * 5. 发布检测结果
     */
    void midCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

    /**
     * @brief 关键点路径回调 - 获取库位列表
     *
     * 从导航路径中提取所有 carports 类型的关键点，
     * 更新 parking_spots_ 列表。目标点切换时清除相关历史记录。
     */
    void keyPointPathCallback(const autoware_msgs::KeyPointArray::ConstPtr& msg);

    /**
     * @brief 当前位姿回调 - 距离判断
     *
     * 更新当前车辆位姿，并根据距离更新每个库位的enable状态。
     * 当车辆距离库位过近(< distance_threshold)时禁用，避免车辆自身被误检。
     * 当车辆距离库位过远(> area_enable_distance)时禁用，节省计算资源。
     */
    void currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    // ============ 点云预处理 ============

    /**
     * @brief mid360点云预处理
     * @param input_msg 原始点云消息
     * @param result 输出的预处理结果
     * @return 预处理是否成功
     *
     * 处理流程: 标定变换 → ROI滤波 → 坐标系变换 → 体素降采样 → Z轴滤波 → 地面分割 → 车体过滤
     */
    bool preprocessMidCloud(const sensor_msgs::PointCloud2::ConstPtr& input_msg,
                            PreprocessResult& result);

    /**
     * @brief 中距雷达标定变换
     * @param msgPtr 原始点云
     * @param header 消息头
     * @return 标定后的点云
     */
    PointCloud::Ptr pointsMidCalibration(PointCloud::Ptr msgPtr, std_msgs::Header header);

    /**
     * @brief ROI区域滤波
     * @param input 输入点云
     * @param output 输出点云(仅保留ROI半径内的点)
     */
    void applyROIFilter(const PointCloud::Ptr& input, PointCloud::Ptr& output);

    /**
     * @brief 点云坐标系变换
     * @param input 输入点云
     * @param output 输出点云
     * @param source_frame 源坐标系
     * @param target_frame 目标坐标系
     * @return 变换是否成功
     */
    bool transformPointCloud(const PointCloud::Ptr& input, PointCloud::Ptr& output,
                            const std::string& source_frame, const std::string& target_frame);

    /**
     * @brief RANSAC地面平面分割
     * @param input 输入点云
     * @param ground 输出地面点云
     * @param obstacles 输出障碍物点云(非地面点)
     */
    void segmentGroundPlane(const PointCloud::Ptr& input,
                           PointCloud::Ptr& ground,
                           PointCloud::Ptr& obstacles);

    // ============ 区域检测 ============

    /**
     * @brief 检测单个库位区域内的障碍物
     * @param spot 库位信息
     * @param obstacle_cloud 障碍物点云(已预处理)
     * @param header 消息头
     * @return 是否检测到障碍物
     *
     * 将目标点从map坐标系变换到velodyne坐标系，然后将每个障碍物点变换到
     * 目标点的局部坐标系中，检查是否落在预定义的边界框内。
     */
    bool checkAreaForSpot(const ParkingSpot& spot,
                          const PointCloud::Ptr& obstacle_cloud,
                          const std_msgs::Header& header);

    /**
     * @brief 库位外围聚类检测 - Layer 2补充检测
     * @param spot 库位信息
     * @param mid_cloud 预处理后的mid360点云
     * @param header 消息头
     * @return 是否检测到库位外围的邻近障碍物
     *
     * 当Layer 1未检测到障碍物时触发。对点云做欧几里得聚类，
     * 检查是否有聚类"贴着"库位边界框外侧。
     */
    bool checkProximityCluster(const ParkingSpot& spot,
                               const PointCloud::Ptr& mid_cloud,
                               const std_msgs::Header& header);

    /**
     * @brief 更新单个库位的防抖状态
     * @param spot_id 库位ID
     * @param raw_detection 当前帧原始检测结果
     *
     * 使用滑动窗口 + 施密特触发器:
     *   确认: noise_count >= confirm_threshold
     *   清除: noise_count <= clear_threshold
     *   中间状态: 保持当前状态不变
     */
    void updateSpotDetectionState(const std::string& spot_id, bool raw_detection);

    /**
     * @brief 将目标点位姿从map坐标系变换到velodyne坐标系
     * @param input_pose map坐标系下的位姿
     * @return velodyne坐标系下的位姿
     */
    geometry_msgs::Pose transformTargetPoseToVelodyne(const geometry_msgs::PoseStamped& input_pose);

    // ============ 可视化 ============

    /**
     * @brief 发布库区检测状态的可视化标记
     *
     * 在RVIZ中绘制所有库位的3D边界框 + 状态文本
     */
    void publishAreaMarkers(const std_msgs::Header& header);

    /**
     * @brief 发布各类点云
     */
    void publishPointClouds(const PointCloud::Ptr& ground,
                           const PointCloud::Ptr& obstacles,
                           const std_msgs::Header& header);

    /**
     * @brief 发布外围聚类检测的可视化信息
     */
    void publishProximityInfo(const ParkingSpot& spot,
                              const PointCloud::Ptr& cluster_cloud,
                              const geometry_msgs::Point& centroid,
                              double distance_to_box,
                              size_t point_count,
                              const std_msgs::Header& header);

    // ============ 辅助函数 ============

    /**
     * @brief 定时器回调 - 10Hz发布检测状态
     */
    void timerCallback(const ros::TimerEvent& event);

    /**
     * @brief yaml文件变更检查回调 - 1Hz检查配置文件是否被修改
     */
    void checkAndReloadYaml(const ros::TimerEvent& event);

    /** @brief 设置ROS日志级别(从参数加载) */
    void setLogLevel();

    /**
     * @brief 生成库位唯一ID
     * @param index 库位在路径中的索引
     * @return 库位ID字符串(如 "carports_0")
     */
    std::string generateSpotId(int index);
};


#endif