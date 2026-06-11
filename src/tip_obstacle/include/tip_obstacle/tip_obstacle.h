/**
 * @file tip_obstacle.h
 * @brief 叉车货叉区域单线激光雷达避障节点 —— 头文件
 *
 * 功能概述:
 *   利用安装在叉车货叉上的 1~2 个单线激光雷达，实现近距离避障检测。
 *   支持单叉臂 (tip_type=1) 和双叉臂 (tip_type=0) 两种工作模式。
 *   核心流程: LaserScan → 角度/Y轴盲区过滤 → TF 外参变换 → 点云融合发布
 *   额外功能: 库位动态安全走廊裁切、可视区域可视化、YAML 参数热加载。
 */
#ifndef TIP_OBSTACLE_H
#define TIP_OBSTACLE_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <laser_geometry/laser_geometry.h>    // 2D LaserScan → 3D PointCloud2 投影

#include <message_filters/sync_policies/approximate_time.h>  // 近似时间同步策略
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>  // ROS ↔ PCL 消息转换
#include <pcl/common/transforms.h>             // PCL 点云变换 (旋转+平移)

#include <tf/transform_broadcaster.h>          // TF 广播 (雷达外参)
#include <tf/transform_listener.h>             // TF 监听 (库位坐标变换)

// 车辆状态消息、速度控制消息与可视化 Marker
#include <autoware_remove_msgs/State.h>
#include <geometry_msgs/TwistStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

#include <mutex>
#include <thread>
#include <atomic>
#include <string>
#include <vector>
#include <cmath>

// ==================== 数据结构定义 ====================

/**
 * @brief 雷达外参结构体
 * 描述单线雷达相对于主雷达 (velodyne) 的 6DOF 安装位姿。
 * 数据来源: lidar_calibration.yaml
 */
struct TfParam {
    double x = 0;      // X 方向平移 (米)
    double y = 0;      // Y 方向平移 (米)
    double z = 0;      // Z 方向平移 (米)
    double roll = 0;   // 绕 X 轴旋转 (弧度)
    double pitch = 0;  // 绕 Y 轴旋转 (弧度)
    double yaw = 0;    // 绕 Z 轴旋转 (弧度)
};

/**
 * @brief 盲区过滤参数结构体
 * 分别控制左/右雷达的角度过滤范围和 Y 轴过滤范围。
 * 数据来源: tip_obstacle.yaml (normal_filter / pallet_id_filter 段)
 */
struct FilterParam {
    int left_filter_enable = 0;      // 左雷达过滤开关 (1=开启)
    int right_filter_enable = 0;     // 右雷达过滤开关 (1=开启)
    double left_min_angle = 0.0;     // 左雷达盲区角度下界 (度, [0,360))
    double left_max_angle = 0.0;     // 左雷达盲区角度上界 (度, [0,360))
    double left_min_y = 0.0;         // 左雷达盲区 Y 轴下界 (米)
    double left_max_y = 0.0;         // 左雷达盲区 Y 轴上界 (米)
    double right_min_angle = 0.0;    // 右雷达盲区角度下界 (度, [0,360))
    double right_max_angle = 0.0;    // 右雷达盲区角度上界 (度, [0,360))
    double right_min_y = 0.0;        // 右雷达盲区 Y 轴下界 (米)
    double right_max_y = 0.0;        // 右雷达盲区 Y 轴上界 (米)
};

/**
 * @brief 全局应用配置结构体 (统一管理所有可配置参数，消除魔法数字)
 * 数据来源: tip_obstacle.yaml 的各配置段
 */
struct AppConfig {
    // ---------- 超时参数 ----------
    double state_timeout = 2.0;       // 状态超时时间 (秒): 倒车/泊车状态的有效持续时间
    double tf_timeout = 0.05;         // TF 查找等待超时 (秒)

    // ---------- 运动阈值 ----------
    double reverse_velocity = -0.01;  // 倒车速度阈值 (m/s): vx < 此值视为倒车
    double forward_velocity = 0.2;    // 前进速度阈值 (m/s): vx > 此值重置防撞
    double turning_angular_threshold = 0.05;  // 转向角速度阈值 (rad/s): |wz| > 此值触发防撞

    // ---------- 距离阈值 ----------
    double valid_distance_min = 0.01;         // 最小有效距离 (米): 小于此值的距离数据被忽略
    double carport_activation_dist = 5.0;     // 库位激活距离 (米): 距库位 < 此值时激活走廊裁切
    float max_detect_distance = 255.0f;       // 最大检测距离 (米): 无有效点时的默认值

    // ---------- 任务类型白名单 ----------
    std::vector<int> valid_task_types = {0, 1, 2};  // 允许更新库位距离的任务类型列表

    // ---------- 可视化参数 ----------
    double marker_line_width = 0.05;   // Marker 线宽 (米)
    double marker_color_r = 0.0;       // Marker 颜色 R 分量
    double marker_color_g = 1.0;       // Marker 颜色 G 分量 (默认绿色)
    double marker_color_b = 0.0;       // Marker 颜色 B 分量
    double marker_color_a = 1.0;       // Marker 颜色透明度

    int visibility_marker_enable = 0;           // 可视区域 Marker 开关 (1=开启)
    double visibility_ref_distance = 0.5;       // 可视区域参考距离 (米): 扇形半径
    int tf_broadcast_enable = 1;                // TF 广播开关 (1=开启)

    // ---------- 过滤参数集 ----------
    FilterParam normal_filter;   // 常规过滤参数 (pallet_id_state ≤ 0 时使用)
    FilterParam pallet_filter;   // 托盘过滤参数 (pallet_id_state > 0 时使用)
};

// ==================== 主节点类 ====================

/**
 * @class TipObstacleNode
 * @brief 叉车货叉避障节点
 *
 * 核心工作流:
 *   1. 接收 /scan_bleft 和 /scan_bright 的 2D 激光数据
 *   2. 通过 laser_geometry 将 2D scan 转为 PointCloud2 (局部坐标系)
 *   3. 在雷达原始坐标系下做角度和 Y 轴盲区过滤
 *   4. 乘以 YAML 中的 TF 外参矩阵，映射到 velodyne 坐标系
 *   5. 左右点云拼接融合后发布
 *   6. (条件触发) 在 base_link 下做库位走廊裁切
 */
class TipObstacleNode {
public:
    /**
     * @brief 构造函数: 初始化参数、启动 YAML 监听、创建 ROS 通信
     * @param nh  全局 NodeHandle
     * @param pnh 私有 NodeHandle (~)
     */
    TipObstacleNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);

    /** @brief 析构函数: 停止 YAML 监听线程并等待其退出 */
    ~TipObstacleNode();

private:
    // ==================== 配置加载 ====================

    /** @brief 加载 3 个 YAML 配置文件 (外参/库位/策略) */
    void loadYAML();

    /**
     * @brief YAML 文件监听线程 (后台运行)
     * 使用 Linux inotify 机制监听 3 个 YAML 文件的写入/移动事件，
     * 任一文件变更时自动重新加载全部配置。
     */
    void watchYAMLThread();

    // ==================== ROS 回调函数 ====================

    /**
     * @brief /arrived_flag 话题回调: 更新托盘到达状态
     * data > 0 时切换到托盘过滤参数集, 否则使用常规过滤参数集
     */
    void palletIdCallback(const std_msgs::Int8::ConstPtr &msg);

    /**
     * @brief 双叉臂模式同步回调: 同时接收左右雷达数据
     * 使用 message_filters::ApproximateTime 策略同步
     */
    void scanCallbackSync(const sensor_msgs::LaserScan::ConstPtr &msg1,
                          const sensor_msgs::LaserScan::ConstPtr &msg2);

    /**
     * @brief 单叉臂模式回调: 仅接收左雷达数据
     */
    void scanCallbackSingle(const sensor_msgs::LaserScan::ConstPtr &msg);

    // ==================== 核心处理函数 ====================

    /**
     * @brief 单雷达点云处理管线: 投影→过滤→TF变换
     *
     * 处理流程:
     *   1. LaserScan → PointCloud2 (laser_geometry 投影)
     *   2. 防撞开关判断 (基于 last_reverse_time 超时)
     *   3. 根据 pallet_id_state_ 选择 normal_filter 或 pallet_filter
     *   4. 在雷达局部坐标系下做角度 + Y 轴盲区过滤
     *   5. 乘以 TF 外参矩阵变换到 velodyne 坐标系
     *
     * @param scan_msg  输入的 LaserScan 消息
     * @param is_left   true=左雷达, false=右雷达
     * @return 过滤并变换后的点云 (velodyne 坐标系)
     */
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterAndTransformCloud(
        const sensor_msgs::LaserScan& scan_msg, bool is_left);

    /**
     * @brief 根据 TfParam 构造 4×4 仿射变换矩阵
     * 旋转顺序: Yaw (Z) → Pitch (Y) → Roll (X) —— 欧拉角 ZYX 约定
     */
    Eigen::Affine3f getTransformMatrix(const TfParam& param);

    /**
     * @brief 计算点云中所有点到雷达安装位置的最短欧氏距离
     * @param cloud  输入点云 (velodyne 坐标系)
     * @param is_left  true=左雷达, false=右雷达 (用于确定雷达安装位置)
     * @return 最短距离 (米); 无有效点时返回 max_detect_distance (默认 255)
     */
    float calculateMinDisToLidar(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, bool is_left);

    // ==================== 库位走廊与可视化 ====================

    /**
     * @brief /twist_cmd 话题回调: 根据速度指令更新倒车/转向时间戳
     * vx < reverse_velocity 或 |wz| > turning_angular_threshold → 记录时间戳 (触发防撞)
     * vx > forward_velocity → 重置时间戳 (关闭防撞)
     */
    void twistCmdCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);

    /**
     * @brief /feedback_status 话题回调: 更新任务类型和库位距离
     * 仅当任务类型在 valid_task_types 白名单中且距离 > valid_distance_min 时更新
     */
    void feedbackStatusCallback(const autoware_remove_msgs::State::ConstPtr& msg);

    /**
     * @brief 库位动态安全走廊裁切 (条件性二次裁切)
     *
     * 激活条件: 同时满足 is_reversing (倒车/转向中) AND is_parking (接近库位)
     * 处理流程:
     *   1. 将点云从 velodyne 变换到 base_link
     *   2. 根据 dis_to_carport_ 动态计算 AABB 裁切范围
     *   3. 仅保留走廊包络内的点
     *   4. 逆变换回 velodyne 坐标系
     *
     * @param cloud  输入/输出点云 (velodyne 坐标系, 原地修改)
     * @param stamp  点云时间戳 (用于 TF 同步)
     */
    void applyCarportFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const ros::Time& stamp);

    /** @brief 发布库位走廊 3D 包围盒 Marker (绿色线框, frame=base_link) */
    void publishCarportMarker();

    /**
     * @brief 发布可视区域 Marker (蓝色扇形, frame=map)
     * 可视区域 = 盲区角度的补集, 绘制为径向线 + 弧线
     */
    void publishVisibilityMarker();

    // ==================== ROS 句柄 ====================
    ros::NodeHandle nh_;     // 全局 NodeHandle
    ros::NodeHandle pnh_;    // 私有 NodeHandle (~)

    // ==================== Publishers ====================
    ros::Publisher pc_fused_pub_;         // 融合点云 (默认话题: fused_points_tip)
    ros::Publisher pc_left_pub_;          // 左雷达点云 (默认话题: bleft_points_tip)
    ros::Publisher pc_right_pub_;         // 右雷达点云 (默认话题: bright_points_tip)
    ros::Publisher min_dis_pub_;          // 最近距离 (话题: tip_dis, 类型: Float32)
    ros::Publisher carport_marker_pub_;   // 库位走廊 Marker (话题: carport)
    ros::Publisher visibility_marker_pub_; // 可视区域 Marker (话题: visibility_region)

    // ==================== Subscribers ====================
    ros::Subscriber pallet_id_sub_;           // /arrived_flag (Int8): 托盘到达状态
    ros::Subscriber single_scan_sub_;         // /scan_bleft: 单叉臂模式单订阅
    ros::Subscriber twist_cmd_sub_;           // /twist_cmd (TwistStamped): 速度指令
    ros::Subscriber feedback_status_sub_;     // /feedback_status (State): 反馈状态

    // 双叉臂模式: message_filters 时间同步
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>> sub_scan_left_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>> sub_scan_right_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan> SyncPolicy;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    // ==================== TF 与激光投影 ====================
    laser_geometry::LaserProjection projector_;   // LaserScan → PointCloud2 投影器
    tf::TransformBroadcaster tf_broadcaster_;     // TF 广播器 (发布雷达外参)
    tf::TransformListener tf_listener_;           // TF 监听器 (查找坐标变换)
    ros::Timer tf_timer_;                         // 10Hz 定时器: 周期广播 TF

    // ==================== 基本配置参数 ====================
    int tip_type_;                // 叉臂类型: 0=双叉臂, 1=单叉臂
    bool debug_mode_;             // 调试模式: true 时绕过所有过滤逻辑
    std::string parent_frame_;    // 父坐标系名称 (默认: velodyne)
    std::string left_child_frame_;   // 左雷达子坐标系名称 (默认: bleft_laser)
    std::string right_child_frame_;  // 右雷达子坐标系名称 (默认: bright_laser)
    std::string base_link_frame_;    // base_link 坐标系名称 (默认: base_link)
    std::string left_scan_frame_;    // 左雷达扫描坐标系名称 (默认: scan_bleft_link)
    std::string right_scan_frame_;   // 右雷达扫描坐标系名称 (默认: scan_bright_link)

    // ==================== 三个核心 YAML 文件路径 ====================
    std::string tf_yaml_path_;        // 雷达外参 YAML 路径 (lidar_calibration.yaml)
    std::string carport_yaml_path_;   // 库位几何参数 YAML 路径 (obstacle_detection.yaml)
    std::string config_yaml_path_;    // 全局策略配置 YAML 路径 (tip_obstacle.yaml)

    // ==================== 全局配置与线程安全 ====================
    AppConfig app_cfg_;               // 全局应用配置 (多线程共享)
    std::mutex cfg_mutex_;            // 保护 app_cfg_ 的互斥锁

    // ==================== 雷达外参与线程安全 ====================
    TfParam left_tf_;                 // 左雷达外参 (多线程共享)
    TfParam right_tf_;                // 右雷达外参 (多线程共享)
    std::mutex tf_mutex_;             // 保护 left_tf_ / right_tf_ 的互斥锁

    // ==================== 库位几何参数 (base_link 坐标系下) ====================
    double carports_min_x_ = -0.7;   // 走廊 X 方向下界 (米)
    double carports_max_x_ = 0.9;    // 走廊 X 方向上界 (米)
    double carports_min_y_ = -0.6;   // 走廊 Y 方向下界 (米)
    double carports_max_y_ = 0.6;    // 走廊 Y 方向上界 (米)
    double carports_min_z_ = 0.0;    // 走廊 Z 方向下界 (米)
    double carports_max_z_ = 1.0;    // 走廊 Z 方向上界 (米)

    // ==================== YAML 热加载线程 ====================
    std::thread yaml_watcher_thread_;      // 后台监听线程
    std::atomic<bool> thread_running_;     // 线程运行标志 (false 时线程退出)

    // ==================== 状态机变量 (原子变量, 多线程安全) ====================
    std::atomic<int> pallet_id_state_{-1};       // 托盘到达状态: >0 表示到达托盘
    std::atomic<float> dis_to_carport_{0.0f};    // 到库位的当前距离 (米)
    std::atomic<double> last_reverse_time_{0.0}; // 最后一次倒车/转向的时间戳 (秒)
    std::atomic<double> last_parking_time_{0.0}; // 最后一次进入库位激活区域的时间戳 (秒)
    std::atomic<int> current_task_type_{-1};     // 当前任务类型
    bool marker_published_{false};               // 库位 Marker 是否已发布 (用于 DELETE 清除)
};

#endif // TIP_OBSTACLE_H