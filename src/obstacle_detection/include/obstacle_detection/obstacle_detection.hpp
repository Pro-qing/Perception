/**
 * @file obstacle_detection.hpp
 * @brief 障碍物检测主类的头文件
 *
 * 该模块是自动驾驶感知系统中的障碍物检测节点，主要用于电梯和库位场景下的
 * 障碍物检测。通过融合中距雷达(points_mid)和补盲雷达(fused_points_tip)的
 * 点云数据，对目标区域（电梯/库位）进行障碍物检测，判断目标区域是否存在
 * 异物/噪声点，并通过位标志向下游系统报告检测结果。
 *
 * 数据处理流程:
 *   1. 同步接收两路点云(mid + tip)
 *   2. 对mid点云进行预处理(标定→ROI→坐标变换→降采样→Z轴滤波→地面分割→车体过滤)
 *   3. 将mid和tip点云融合
 *   4. 根据目标点类型(电梯/库位)对目标区域进行障碍物检测
 *   5. 发布检测结果和可视化信息
 *
 * 目标点启用条件:
 *   - 库位(GARAGE): feedback_status中任务类型为1且距离<10m时启用
 *   - 电梯(ELEVATOR): 电梯门打开 + 电梯控制标志为±4 + 楼层匹配时启用
 *   - 当车辆距离目标点过近(< distance_threshold)时禁用，避免误检
 */
#ifndef __OBSTACLE_DETECTION_HPP_
#define __OBSTACLE_DETECTION_HPP_

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
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <std_msgs/UInt32.h>
#include <autoware_msgs/KeyPointArray.h>
#include <autoware_remove_msgs/State.h>
#include <autoware_msgs/ElevatorInfo.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <memory>
#include <deque>
#include <sys/stat.h>

/** @brief 点类型别名，使用XYZI格式的PCL点 */
typedef pcl::PointXYZI PointT;
/** @brief 点云类型别名 */
typedef pcl::PointCloud<PointT> PointCloud;


/**
 * @brief 目标点类型枚举
 *
 * 定义了两种目标点类型，决定了不同的检测逻辑和启用条件
 */
enum PointType_e {
    POINT_TYPE_ELEVATOR = 0,        ///< 电梯目标点 - 需检测电梯轿厢内是否有异物
    POINT_TYPE_GARAGE = 1,          ///< 库位目标点 - 需检测停车位是否有障碍物
};

/**
 * @brief 目标点信息结构体
 *
 * 存储当前导航目标点的信息，包括在map坐标系下的位姿、类型和启用状态
 */
struct TargetPoint {
    geometry_msgs::PoseStamped map_pose;  ///< 目标点在map坐标系下的位姿
    PointType_e type_e;                   ///< 目标点类型(电梯/库位)
    bool enable;                          ///< 是否启用检测(true=正在检测目标区域)
};

/**
 * @brief 障碍物检测主类
 *
 * 实现了完整的障碍物检测流水线，包括:
 * - 双雷达点云同步与融合
 * - 点云预处理(标定、ROI、降采样、地面分割、车体过滤)
 * - 目标区域障碍物检测(基于目标点局部坐标系的边界框检测)
 * - 欧几里得聚类(仅电梯场景)
 * - 检测结果发布与可视化
 */
class ObstacleDetection {
private:
    ros::NodeHandle& nh_;           ///< 全局节点句柄
    ros::NodeHandle& private_nh_;   ///< 私有节点句柄(用于获取参数)

    // ============ 消息过滤器(用于双雷达时间同步) ============
    /** @brief 时间同步策略：近似时间同步，对齐mid和tip两路点云 */
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2>;
    message_filters::Subscriber<sensor_msgs::PointCloud2> mid_cloud_sub_;   ///< 中距雷达点云订阅者
    message_filters::Subscriber<sensor_msgs::PointCloud2> tip_cloud_sub_;   ///< 补盲雷达点云订阅者
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;       ///< 时间同步器

    // ============ 订阅者 ============
    ros::Subscriber keyPointPath_sub_;      ///< 导航关键点路径订阅(获取目标点位姿和类型)
    ros::Subscriber feedback_status_sub_;   ///< 反馈状态订阅(库位场景的启用条件)
    ros::Subscriber elevator_info_sub_;     ///< 电梯信息订阅(电梯场景的启用条件)
    ros::Subscriber lqr_dire_sub_;          ///< 电梯控制方向标志订阅(±4表示进出电梯)
    ros::Subscriber floor_set_sub_;         ///< 目标楼层设置订阅
    ros::Subscriber current_pose_sub_;      ///< 当前车辆位姿订阅(用于距离判断)

    // ============ 发布者 ============
    ros::Publisher ground_pub_;              ///< 地面点云发布
    ros::Publisher obstacle_pub_;            ///< 障碍物点云发布
    ros::Publisher cluster_pub_;             ///< 彩色聚类点云发布
    ros::Publisher marker_pub_;              ///< 可视化标记(MarkerArray)发布
    ros::Publisher calibration_pub_;         ///< 标定后点云发布
    ros::Publisher target_region_pub_;       ///< 目标区域标记发布
    ros::Publisher target_region_cloud_pub_; ///< 目标区域内点云发布
    ros::Publisher obstacle_detection_pub_;  ///< 障碍物检测状态发布(位标志)
    ros::Publisher fused_cloud_pub_;         ///< 融合后点云发布
    ros::Publisher proximity_cloud_pub_;    ///< 外围聚类检测点云发布
    ros::Publisher proximity_marker_pub_;   ///< 外围聚类检测可视化标记发布

    ros::Timer publish_timer_;               ///< 定时发布检测状态的定时器(10Hz)
    ros::Timer yaml_reload_timer_;           ///< yaml文件变更检查定时器(1Hz)
    tf::TransformListener tf_listener_;      ///< TF变换监听器

    // ============ 基础参数(从launch文件加载) ============
    std::string target_frame_;          ///< 目标坐标系(通常是velodyne)
    std::string points_mid_topic_;      ///< 中距雷达点云topic名称
    double voxel_leaf_size_;            ///< 体素降采样叶子大小(0表示不降采样)
    double cluster_tolerance_;          ///< 欧几里得聚类容差(米)
    int min_cluster_size_;              ///< 最小聚类点数
    int max_cluster_size_;              ///< 最大聚类点数
    double z_axis_min_;                 ///< Z轴滤波最小值
    double z_axis_max_;                 ///< Z轴滤波最大值
    double ground_threshold_;           ///< RANSAC地面分割距离阈值
    double roi_radius_;                 ///< ROI区域半径(米)
    bool use_roi_filter_;               ///< 是否启用ROI滤波
    double distance_threshold_;         ///< 到目标点的距离阈值，小于此值时关闭enable
    int min_region_points_;             ///< 目标区域内最小点数阈值，低于此值不判定为有障碍物
    int garage_history_size_;           ///< 库位检测滑动窗口大小(帧数)
    int garage_confirm_threshold_;      ///< 库位检测确认阈值(窗口内需有N帧检测到才确认)
    int garage_clear_threshold_;        ///< 库位检测清除阈值(窗口内需<=N帧检测到才清除，防状态跳变)
    double garage_enable_distance_;     ///< 库位检测启用距离(米)
    std::deque<bool> garage_detection_history_;  ///< 库位检测历史记录(滑动窗口)

    // ============ 库位外围聚类检测参数(Layer 2) ============
    bool garage_enable_proximity_cluster_;       ///< 是否启用外围聚类检测
    double garage_cluster_tolerance_;            ///< 库位聚类容差(米)
    int garage_cluster_min_points_;              ///< 最小聚类点数
    int garage_cluster_max_points_;              ///< 最大聚类点数(过滤墙壁等大结构)
    double garage_cluster_max_z_range_;          ///< 聚类z范围上限(米)
    double garage_cluster_min_centroid_z_;       ///< 聚类重心最低高度(米)
    double garage_expand_margin_x_;              ///< X方向扩展边界(米)
    double garage_expand_margin_y_;              ///< Y方向扩展边界(米)
    double garage_proximity_threshold_;          ///< 聚类到库位框最大距离(米)
    bool garage_proximity_debug_;                ///< 外围聚类调试日志开关

    // ============ YAML动态重载 ============
    std::string yaml_file_path_;        ///< yaml配置文件路径(从launch传入)
    time_t last_yaml_mod_time_;         ///< yaml文件上次修改时间戳

    // ============ 运行时状态 ============
    int8_t elevator_control_flag_;      ///< 电梯控制标志(±4表示进出电梯)
    int8_t floor_set_;                  ///< 当前设置的目标楼层
    TargetPoint targetPoint_;           ///< 当前目标点信息
    geometry_msgs::PoseStamped current_pose_;  ///< 当前车辆在map坐标系下的位姿

    // ============ 中距雷达标定参数(从yaml动态加载) ============
    double mid_x_;        ///< X轴平移偏移
    double mid_y_;        ///< Y轴平移偏移
    double mid_z_;        ///< Z轴平移偏移
    double mid_roll_;     ///< 绕X轴旋转偏移(弧度)
    double mid_pitch_;    ///< 绕Y轴旋转偏移(弧度)
    double mid_yaw_;      ///< 绕Z轴旋转偏移(弧度)

    // ============ 电梯检测区域参数(在目标点局部坐标系下的边界框) ============
    double elevator_min_x_, elevator_max_x_;  ///< X轴范围
    double elevator_min_y_, elevator_max_y_;  ///< Y轴范围
    double elevator_min_z_, elevator_max_z_;  ///< Z轴范围

    // ============ 车体轮廓参数(用于过滤车体自身的点) ============
    std::vector<Eigen::Vector2f> car_vertices_;  ///< 车体XY平面多边形顶点
    double car_min_z_, car_max_z_;                ///< 车体Z轴范围

    // ============ 库位检测区域参数(在目标点局部坐标系下的边界框) ============
    double carports_min_x_, carports_max_x_;  ///< X轴范围
    double carports_min_y_, carports_max_y_;  ///< Y轴范围
    double carports_min_z_, carports_max_z_;  ///< Z轴范围

    // ============ 检测状态 ============
    bool target_region_has_noise_;              ///< 目标区域内是否检测到噪点/障碍物
    autoware_remove_msgs::State feedbackStatus_; ///< 反馈状态缓存

    std_msgs::UInt32 obstacle_detection_;       ///< 障碍物检测位标志(bit0=电梯, bit1=库位)

public:
    /**
     * @brief 构造函数
     * @param nh 全局节点句柄
     * @param private_nh 私有节点句柄
     *
     * 初始化所有参数、订阅者、发布者和定时器
     */
    ObstacleDetection(ros::NodeHandle& nh, ros::NodeHandle& private_nh);

private:
    /**
     * @brief 障碍物信息结构体
     *
     * 存储单个聚类障碍物的几何信息(用于电梯场景的聚类分析)
     */
    struct ObstacleInfo {
        geometry_msgs::Point center;       ///< 障碍物中心点(在target_frame坐标系下)
        geometry_msgs::Vector3 dimensions; ///< 障碍物尺寸(长宽高)
        double distance;                   ///< 到原点的距离(XY平面)
        int point_count;                   ///< 聚类包含的点数
    };

    /**
     * @brief 中距雷达预处理结果结构体
     */
    struct MidProcessResult {
        PointCloud::Ptr ground_cloud;    ///< 分割出的地面点云
        PointCloud::Ptr filtered_cloud;  ///< 过滤后的障碍物点云(已去除地面和车体)
        std_msgs::Header header;         ///< 消息头(包含时间戳和坐标系)
        bool valid;                      ///< 预处理是否成功
    };

    /**
     * @brief 定时器回调 - 以10Hz频率发布障碍物检测状态
     *
     * 确保下游系统即使没有新的点云输入也能持续收到检测状态
     */
    void timerCallback(const ros::TimerEvent& event);

    /**
     * @brief yaml文件变更检查回调 - 以1Hz频率检查配置文件是否被修改
     *
     * 通过stat()获取文件修改时间，与上次记录的时间戳比较。
     * 如果检测到文件被修改，调用rosparam load将新配置加载到参数服务器，
     * 已有的getParam()调用会在下一帧自动获取新值。
     *
     * 使用方式: 在编辑yaml文件并保存后，最多1秒内自动生效。
     */
    void checkAndReloadYaml(const ros::TimerEvent& event);

    /**
     * @brief 双雷达同步回调 - 核心处理入口
     * @param mid_msg 中距雷达点云消息
     * @param tip_msg 补盲雷达点云消息
     *
     * 处理流程:
     * 1. 检查目标点是否启用
     * 2. 预处理mid点云
     * 3. 变换tip点云到目标坐标系
     * 4. 融合两路点云
     * 5. 对目标区域进行障碍物检测
     * 6. 根据目标类型设置检测标志位
     */
    void syncCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& mid_msg,
                           const sensor_msgs::PointCloud2::ConstPtr& tip_msg);

    /**
     * @brief 中距雷达点云预处理
     * @param input_msg 原始点云消息
     * @param result 输出的预处理结果
     * @return 预处理是否成功
     *
     * 处理流程: 标定变换 → ROI滤波 → 坐标系变换 → 体素降采样 → Z轴滤波 → 地面分割 → 车体过滤
     */
    bool preprocessMidCloud(const sensor_msgs::PointCloud2::ConstPtr& input_msg,
                            MidProcessResult& result);

    /**
     * @brief 关键点路径回调 - 获取目标点信息
     *
     * 从导航路径的最后一个关键点中提取目标类型(电梯/库位)和位姿
     * 电梯通过robustElevatorCheck验证connects类型数据
     */
    void keyPointPathCallback(const autoware_msgs::KeyPointArray::ConstPtr& msg);

    /**
     * @brief 反馈状态回调 - 库位场景启用条件
     *
     * 当任务类型为1且距离目标<10m时启用库位检测
     */
    void feedbackStatusCallback(const autoware_remove_msgs::State::ConstPtr& msg);

    /**
     * @brief 电梯信息回调 - 电梯场景启用条件
     *
     * 启用条件: 电梯门打开 + 控制标志为±4 + 楼层匹配
     */
    void elevatorInfoCallback(const autoware_msgs::ElevatorInfo::ConstPtr& msg);

    /**
     * @brief 当前位姿回调 - 距离判断
     *
     * 当车辆距离目标点过近(< distance_threshold)时禁用检测，避免车辆自身被误检
     */
    void currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    /**
     * @brief 点云坐标系变换
     * @param input 输入点云
     * @param output 输出点云
     * @param source_frame 源坐标系
     * @param target_frame 目标坐标系
     * @return 变换是否成功
     *
     * 使用TF查找变换矩阵，然后用PCL进行点云变换
     */
    bool transformPointCloud(const PointCloud::Ptr& input, PointCloud::Ptr& output,
                            const std::string& source_frame, const std::string& target_frame);

    /** @brief 电梯控制方向标志回调 */
    void lqrDireCallback(const std_msgs::Int8::ConstPtr& msg);

    /** @brief 目标楼层设置回调 */
    void floorSetCallback(const std_msgs::Int8::ConstPtr& msg);

    /**
     * @brief ROI区域滤波
     * @param input 输入点云
     * @param output 输出点云(仅保留ROI半径内的点)
     *
     * 基于XY平面距离进行滤波，去除远处的无效点
     */
    void applyROIFilter(const PointCloud::Ptr& input, PointCloud::Ptr& output);

    /**
     * @brief RANSAC地面平面分割
     * @param input 输入点云
     * @param ground 输出地面点云
     * @param obstacles 输出障碍物点云(非地面点)
     *
     * 使用RANSAC算法拟合平面模型，将点云分为地面和非地面两部分
     */
    void segmentGroundPlane(const PointCloud::Ptr& input,
                           PointCloud::Ptr& ground,
                           PointCloud::Ptr& obstacles);

    /**
     * @brief 欧几里得聚类
     * @param cloud 输入点云
     * @param cluster_indices 输出的聚类索引
     *
     * 仅在电梯场景下使用，用于对障碍物点云进行聚类分析
     */
    void performClustering(const PointCloud::Ptr& cloud,
                          std::vector<pcl::PointIndices>& cluster_indices);

    /**
     * @brief 处理聚类结果 - 计算每个聚类的几何信息
     * @param cloud 输入点云
     * @param cluster_indices 聚类索引
     * @param obstacles 输出的障碍物信息列表
     * @param header 消息头
     */
    void processClusters(const PointCloud::Ptr& cloud,
                        const std::vector<pcl::PointIndices>& cluster_indices,
                        std::vector<ObstacleInfo>& obstacles,
                        const std_msgs::Header& header);

    /**
     * @brief 发布各类点云(地面、障碍物、聚类)
     */
    void publishPointClouds(const PointCloud::Ptr& ground,
                           const PointCloud::Ptr& obstacles,
                           const std::vector<pcl::PointIndices>& cluster_indices,
                           const std_msgs::Header& header);

    /**
     * @brief 发布彩色聚类点云
     *
     * 为每个聚类分配不同颜色，便于在RVIZ中可视化区分不同障碍物
     */
    void publishColoredClusters(const PointCloud::Ptr& cloud,
                               const std::vector<pcl::PointIndices>& cluster_indices,
                               const std_msgs::Header& header);

    /**
     * @brief 发布障碍物信息可视化标记
     */
    void publishObstacleInfo(const std::vector<ObstacleInfo>& obstacles,
                            const std_msgs::Header& header);

    /**
     * @brief 根据距离设置颜色(近红远绿)
     */
    void setColorByDistance(std_msgs::ColorRGBA& color, double distance);

    /** @brief 设置ROS日志级别(从参数加载) */
    void setLogLevel();

    /**
     * @brief 中距雷达标定变换
     * @param msgPtr 原始点云
     * @param header 消息头
     * @return 标定后的点云
     *
     * 根据yaml配置的6DOF参数(平移+旋转)对中距雷达进行外参标定补偿
     * 标定结果会发布到/calibration_points_mid供调试
     */
    PointCloud::Ptr points_mid_calibration(PointCloud::Ptr msgPtr, std_msgs::Header header);

    /**
     * @brief 检测目标区域内的障碍物 - 核心检测逻辑
     * @param obstacle_cloud 障碍物点云(已融合mid+tip)
     * @param header 消息头
     *
     * 将目标点从map坐标系变换到velodyne坐标系，然后将每个障碍物点变换到
     * 目标点的局部坐标系中，检查是否落在预定义的边界框内。
     * 包含三重验证机制(用于调试和确保检测准确性)。
     *
     * 检测结果存储在 target_region_has_noise_ 成员变量中
     */
    void checkTargetPointRegion(const PointCloud::Ptr& obstacle_cloud,
                                             const std_msgs::Header& header);

    /**
     * @brief 发布目标区域可视化标记
     *
     * 在RVIZ中绘制目标区域的3D边界框(线框)
     * 有噪点时显示红色，无噪点时显示白色(电梯)或青色(库位)
     */
    void publishTargetRegionMarker(const std_msgs::Header& header);

    /**
     * @brief 添加单个边界框标记到MarkerArray
     */
    void addSingleMarkerToArray(visualization_msgs::MarkerArray& marker_array, 
                            const std_msgs::Header& header,
                            const Eigen::Vector3f& center_point,
                            const std::vector<Eigen::Vector2f>& polygon_vertices,
                            float min_z, float max_z,
                            const std::string& ns, int id,
                            float r, float g, float b, float yaw_angle);

    /**
     * @brief 电梯数据健壮性检查
     * @param data 电梯connects类型的数据字符串
     * @return 是否为有效的电梯目标
     *
     * 解析格式如 "'A1,1'" 或 "''A1,1''" 的电梯标识数据，
     * 验证第二个引号内值为"1"且最后值为"0"
     */
    bool robustElevatorCheck(const std::string& data);

    /**
     * @brief 将目标点位姿从map坐标系变换到velodyne坐标系
     * @param input_pose map坐标系下的位姿
     * @return velodyne坐标系下的位姿
     */
    geometry_msgs::Pose transformTargetPoseToVelodyne(const geometry_msgs::PoseStamped& input_pose);

    /**
     * @brief 库位外围聚类检测 - Layer 2补充检测
     * @param mid_cloud 预处理后的mid360点云(已去除地面和车体)
     * @param header 消息头
     * @return 是否检测到库位外围的邻近障碍物
     *
     * 当Layer 1(内部区域检测)未检测到障碍物时触发。
     * 对mid360点云做欧几里得聚类，检查是否有聚类"贴着"库位边界框外侧。
     * 通过多重过滤条件(点数、z范围、重心高度、邻近距离)确保检测准确性。
     *
     * 独立实现，不调用电梯场景的performClustering()。
     */
    bool checkGarageProximityCluster(const PointCloud::Ptr& mid_cloud,
                                     const std_msgs::Header& header);
};



#endif