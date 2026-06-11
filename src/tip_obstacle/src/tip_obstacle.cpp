/**
 * @file tip_obstacle.cpp
 * @brief 叉车货叉区域单线激光雷达避障节点 —— 实现文件
 *
 * 核心功能:
 *   1. 接收 1~2 个单线雷达的 LaserScan 数据
 *   2. 在雷达局部坐标系下做角度和 Y 轴盲区过滤
 *   3. 通过 TF 外参矩阵变换到主雷达 (velodyne) 坐标系
 *   4. 左右点云融合发布，同时输出最近障碍物距离
 *   5. (条件触发) 库位动态安全走廊裁切
 *   6. (可选) 可视区域和库位走廊 Marker 可视化
 */
#include "tip_obstacle/tip_obstacle.h"
#include <yaml-cpp/yaml.h>
#include <sys/inotify.h>    // Linux inotify: 文件系统事件监听
#include <poll.h>           // poll(): I/O 多路复用
#include <unistd.h>
#include <algorithm>

// ==================== 构造函数 ====================
// 初始化所有成员、加载配置、创建 ROS 通信、启动 YAML 监听线程
TipObstacleNode::TipObstacleNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) 
    : nh_(nh), pnh_(pnh), thread_running_(true), base_link_frame_("base_link")
{
    // ============================================================
    // 第 1 步: 获取基础 ROS 参数 (从 launch 文件或参数服务器)
    // ============================================================
    pnh_.param<bool>("debug", debug_mode_, false);
    if (debug_mode_) {
        // 调试模式: 绕过所有逻辑过滤 (距离/角度/库位裁切)，直接发布原始分离点云
        ROS_WARN("\n=======================================================\n"
                 " TIP_OBSTACLE DEBUG MODE IS ENABLED! \n"
                 " All logic filters (distance, angle, carport) are BYPASSED.\n"
                 " Publishing RAW separated & fused point clouds to Velodyne!\n"
                 "=======================================================");
    }

    pnh_.param<int>("tip_type", tip_type_, 0);                     // 0=双叉臂, 1=单叉臂
    pnh_.param<std::string>("parent_frame", parent_frame_, "velodyne");         // 父坐标系
    pnh_.param<std::string>("left_child_frame", left_child_frame_, "bleft_laser");   // 左雷达子坐标系
    pnh_.param<std::string>("right_child_frame", right_child_frame_, "bright_laser"); // 右雷达子坐标系
    pnh_.param<std::string>("left_scan_frame", left_scan_frame_, "scan_bleft_link");  // 左雷达扫描坐标系
    pnh_.param<std::string>("right_scan_frame", right_scan_frame_, "scan_bright_link"); // 右雷达扫描坐标系

    // 输出话题名称 (可通过 launch 参数自定义)
    std::string out_fused_pc, out_left_pc, out_right_pc;
    pnh_.param<std::string>("out_fused_points_cloud", out_fused_pc, "fused_points_tip");
    pnh_.param<std::string>("out_bleft_points_cloud", out_left_pc, "bleft_points_tip");
    pnh_.param<std::string>("out_bright_points_cloud", out_right_pc, "bright_points_tip");

    // ============================================================
    // 第 2 步: 获取三个核心 YAML 配置文件路径
    // ============================================================
    pnh_.param<std::string>("tf_yaml_path", tf_yaml_path_, "");          // 雷达外参 (lidar_calibration.yaml)
    pnh_.param<std::string>("carport_yaml_path", carport_yaml_path_, ""); // 库位几何 (obstacle_detection.yaml)
    pnh_.param<std::string>("config_yaml_path", config_yaml_path_, "");  // 全局策略 (tip_obstacle.yaml)

    // ============================================================
    // 第 3 步: 加载配置文件并启动 YAML 热加载监听线程
    // ============================================================
    loadYAML();
    yaml_watcher_thread_ = std::thread(&TipObstacleNode::watchYAMLThread, this);

    // ============================================================
    // 第 4 步: 设置 ROS 通信 (Publishers & Subscribers)
    // ============================================================
    // --- Publishers ---
    pc_fused_pub_  = nh_.advertise<sensor_msgs::PointCloud2>(out_fused_pc, 10);   // 融合点云
    pc_left_pub_   = nh_.advertise<sensor_msgs::PointCloud2>(out_left_pc, 10);    // 左雷达点云
    pc_right_pub_  = nh_.advertise<sensor_msgs::PointCloud2>(out_right_pc, 10);   // 右雷达点云
    min_dis_pub_   = nh_.advertise<std_msgs::Float32>("tip_dis", 10);             // 最近距离
    
    // --- Subscribers ---
    pallet_id_sub_ = nh_.subscribe("/arrived_flag", 10, &TipObstacleNode::palletIdCallback, this);        // 托盘到达状态
    twist_cmd_sub_ = nh_.subscribe("/twist_cmd", 10, &TipObstacleNode::twistCmdCallback, this);            // 速度指令
    feedback_status_sub_ = nh_.subscribe("/feedback_status", 10, &TipObstacleNode::feedbackStatusCallback, this); // 反馈状态
    carport_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("carport", 1, true);              // 库位走廊 Marker
    visibility_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visibility_region", 1, true); // 可视区域 Marker

    // 根据 tip_type 选择订阅模式
    if (tip_type_ == 0) {
        // 双叉臂模式: 使用 message_filters 同步左右雷达数据
        sub_scan_left_.reset(new message_filters::Subscriber<sensor_msgs::LaserScan>(
            nh_, "/scan_bleft", 1, ros::TransportHints().tcpNoDelay()));
        sub_scan_right_.reset(new message_filters::Subscriber<sensor_msgs::LaserScan>(
            nh_, "/scan_bright", 1, ros::TransportHints().tcpNoDelay()));
        // ApproximateTime 同步策略, 队列大小 10
        sync_.reset(new message_filters::Synchronizer<SyncPolicy>(
            SyncPolicy(10), *sub_scan_left_, *sub_scan_right_));
        sync_->registerCallback(boost::bind(&TipObstacleNode::scanCallbackSync, this, _1, _2));
    } else {
        // 单叉臂模式: 仅订阅左雷达
        single_scan_sub_ = nh_.subscribe("/scan_bleft", 10, &TipObstacleNode::scanCallbackSingle, this);
    }

    // ============================================================
    // 第 5 步: 定时发布 TF 外参 (10Hz, 受 tf_broadcast_enable 开关控制)
    // ============================================================
    {
        AppConfig cfg;
        { std::lock_guard<std::mutex> lock(cfg_mutex_); cfg = app_cfg_; }
        if (!cfg.tf_broadcast_enable) {
            ROS_INFO("TF broadcast is DISABLED by config.");
        }
    }
    // 创建 10Hz 定时器，周期性广播 velodyne→bleft_laser 和 velodyne→bright_laser 的 TF 变换
    tf_timer_ = nh_.createTimer(ros::Duration(0.1), [this](const ros::TimerEvent&) {
        // 读取配置 (线程安全快照)
        AppConfig cfg;
        { std::lock_guard<std::mutex> lock(cfg_mutex_); cfg = app_cfg_; }
        if (!cfg.tf_broadcast_enable) return;  // TF 广播被禁用

        std::lock_guard<std::mutex> lock(tf_mutex_);
        ros::Time now = ros::Time::now();

        // 广播左雷达外参 TF: velodyne → bleft_laser
        tf::Transform tf_left;
        tf_left.setOrigin(tf::Vector3(left_tf_.x, left_tf_.y, left_tf_.z));
        tf::Quaternion q_left;
        q_left.setRPY(left_tf_.roll, left_tf_.pitch, left_tf_.yaw);
        tf_left.setRotation(q_left);
        tf_broadcaster_.sendTransform(tf::StampedTransform(tf_left, now, parent_frame_, left_child_frame_));

        // 广播右雷达外参 TF: velodyne → bright_laser
        tf::Transform tf_right;
        tf_right.setOrigin(tf::Vector3(right_tf_.x, right_tf_.y, right_tf_.z));
        tf::Quaternion q_right;
        q_right.setRPY(right_tf_.roll, right_tf_.pitch, right_tf_.yaw);
        tf_right.setRotation(q_right);
        tf_broadcaster_.sendTransform(tf::StampedTransform(tf_right, now, parent_frame_, right_child_frame_));
    });
}

// ==================== 析构函数 ====================
// 停止 YAML 监听线程并等待其安全退出
TipObstacleNode::~TipObstacleNode() {
    thread_running_ = false;  // 通知线程退出
    if (yaml_watcher_thread_.joinable()) yaml_watcher_thread_.join();  // 等待线程结束
}

// ==================== YAML 配置加载 ====================
// 加载 3 个 YAML 配置文件: 雷达外参、库位几何、全局策略
void TipObstacleNode::loadYAML() {
    // -------------------------------------------------------
    // 1. 读取雷达外参 TF
    //    支持两种 YAML 格式:
    //    - 新格式: calibration.bleft.x / calibration.bright.x
    //    - 旧格式: tf_calibration.bleft_x / tf_calibration.bright_x
    // -------------------------------------------------------
    try {
        YAML::Node config = YAML::LoadFile(tf_yaml_path_);
        std::lock_guard<std::mutex> lock(tf_mutex_);

        if (config["calibration"]) {
            // 新格式: calibration.bleft.{x,y,z,roll,pitch,yaw}
            auto cal_node = config["calibration"];
            if (cal_node["bleft"]) {
                auto bl = cal_node["bleft"];
                left_tf_.x = bl["x"].as<double>(0.0);
                left_tf_.y = bl["y"].as<double>(0.0);
                left_tf_.z = bl["z"].as<double>(0.0);
                left_tf_.roll = bl["roll"].as<double>(0.0);
                left_tf_.pitch = bl["pitch"].as<double>(0.0);
                left_tf_.yaw = bl["yaw"].as<double>(0.0);
            }
            if (cal_node["bright"]) {
                auto br = cal_node["bright"];
                right_tf_.x = br["x"].as<double>(0.0);
                right_tf_.y = br["y"].as<double>(0.0);
                right_tf_.z = br["z"].as<double>(0.0);
                right_tf_.roll = br["roll"].as<double>(0.0);
                right_tf_.pitch = br["pitch"].as<double>(0.0);
                right_tf_.yaw = br["yaw"].as<double>(0.0);
            }
        } else if (config["tf_calibration"]) {
            // 旧格式: tf_calibration.bleft_x 等扁平键名
            auto tf_node = config["tf_calibration"];
            left_tf_.x = tf_node["bleft_x"].as<double>(0.0);
            left_tf_.y = tf_node["bleft_y"].as<double>(0.0);
            left_tf_.z = tf_node["bleft_z"].as<double>(0.0);
            left_tf_.yaw = tf_node["bleft_yaw"].as<double>(0.0);
            left_tf_.pitch = tf_node["bleft_pitch"].as<double>(0.0);
            left_tf_.roll = tf_node["bleft_roll"].as<double>(0.0);

            right_tf_.x = tf_node["bright_x"].as<double>(0.0);
            right_tf_.y = tf_node["bright_y"].as<double>(0.0);
            right_tf_.z = tf_node["bright_z"].as<double>(0.0);
            right_tf_.yaw = tf_node["bright_yaw"].as<double>(0.0);
            right_tf_.pitch = tf_node["bright_pitch"].as<double>(0.0);
            right_tf_.roll = tf_node["bright_roll"].as<double>(0.0);
        }
        ROS_INFO("TipObstacle TF YAML Loaded Successfully.");
    } catch (const YAML::Exception& e) {
        ROS_ERROR("Failed to load Lidar TF YAML: %s", e.what());
    }

    // -------------------------------------------------------
    // 2. 读取库位尺寸参数 (carports_min/max_x/y/z)
    //    定义库位走廊在 base_link 坐标系下的 AABB 包围盒
    // -------------------------------------------------------
    try {
        YAML::Node carport_config = YAML::LoadFile(carport_yaml_path_);
        if (carport_config["carports_min_x"]) {
            std::lock_guard<std::mutex> lock(cfg_mutex_);
            carports_min_x_ = carport_config["carports_min_x"].as<double>();
            carports_max_x_ = carport_config["carports_max_x"].as<double>();
            carports_min_y_ = carport_config["carports_min_y"].as<double>();
            carports_max_y_ = carport_config["carports_max_y"].as<double>();
            carports_min_z_ = carport_config["carports_min_z"].as<double>();
            carports_max_z_ = carport_config["carports_max_z"].as<double>();
        }
        ROS_INFO("TipObstacle Carport YAML Loaded Successfully.");
    } catch (const YAML::Exception& e) {
        ROS_WARN("Failed to load Carport YAML: %s", e.what());
    }

    // -------------------------------------------------------
    // 3. 读取全局防撞策略与过滤配置
    //    包含: 超时参数、运动阈值、距离阈值、可视化参数、
    //          常规过滤参数集 (normal_filter) 和托盘过滤参数集 (pallet_id_filter)
    // -------------------------------------------------------
    try {
        YAML::Node cfg = YAML::LoadFile(config_yaml_path_);
        std::lock_guard<std::mutex> lock(cfg_mutex_);

        // 超时参数段
        if (cfg["timeouts"]) {
            app_cfg_.state_timeout = cfg["timeouts"]["state_timeout"].as<double>(2.0);
            app_cfg_.tf_timeout = cfg["timeouts"]["tf_timeout"].as<double>(0.05);
        }
        // 运动与距离阈值段
        if (cfg["thresholds"]) {
            app_cfg_.reverse_velocity = cfg["thresholds"]["reverse_velocity"].as<double>(-0.01);
            app_cfg_.forward_velocity = cfg["thresholds"]["forward_velocity"].as<double>(0.2);
            app_cfg_.turning_angular_threshold = cfg["thresholds"]["turning_angular_threshold"].as<double>(0.05);
            app_cfg_.valid_distance_min = cfg["thresholds"]["valid_distance_min"].as<double>(0.01);
            app_cfg_.carport_activation_dist = cfg["thresholds"]["carport_activation_dist"].as<double>(5.0);
            app_cfg_.max_detect_distance = cfg["thresholds"]["max_detect_distance"].as<float>(255.0f);
        }
        // 任务类型白名单段
        if (cfg["valid_task_types"]) {
            app_cfg_.valid_task_types = cfg["valid_task_types"].as<std::vector<int>>();
        }
        // 可视化参数段
        if (cfg["visualization"]) {
            app_cfg_.marker_line_width = cfg["visualization"]["marker_line_width"].as<double>(0.05);
            app_cfg_.marker_color_r = cfg["visualization"]["marker_color_r"].as<double>(0.0);
            app_cfg_.marker_color_g = cfg["visualization"]["marker_color_g"].as<double>(1.0);
            app_cfg_.marker_color_b = cfg["visualization"]["marker_color_b"].as<double>(0.0);
            app_cfg_.marker_color_a = cfg["visualization"]["marker_color_a"].as<double>(1.0);
            app_cfg_.visibility_marker_enable = cfg["visualization"]["visibility_marker_enable"].as<int>(0);
            app_cfg_.visibility_ref_distance = cfg["visualization"]["visibility_ref_distance"].as<double>(0.5);
            app_cfg_.tf_broadcast_enable = cfg["visualization"]["tf_broadcast_enable"].as<int>(1);
        }
        // 常规过滤参数集 (pallet_id_state ≤ 0 时使用)
        if (cfg["normal_filter"]) {
            auto nf = cfg["normal_filter"];
            app_cfg_.normal_filter.left_filter_enable = nf["left_filter_enable"].as<int>(0);
            app_cfg_.normal_filter.left_min_angle = nf["left_min_angle"].as<double>(0.0);
            app_cfg_.normal_filter.left_max_angle = nf["left_max_angle"].as<double>(0.0);
            app_cfg_.normal_filter.left_min_y = nf["left_min_y"].as<double>(0.0);
            app_cfg_.normal_filter.left_max_y = nf["left_max_y"].as<double>(0.0);
            app_cfg_.normal_filter.right_filter_enable = nf["right_filter_enable"].as<int>(0);
            app_cfg_.normal_filter.right_min_angle = nf["right_min_angle"].as<double>(0.0);
            app_cfg_.normal_filter.right_max_angle = nf["right_max_angle"].as<double>(0.0);
            app_cfg_.normal_filter.right_min_y = nf["right_min_y"].as<double>(0.0);
            app_cfg_.normal_filter.right_max_y = nf["right_max_y"].as<double>(0.0);
        }
        // 托盘过滤参数集 (pallet_id_state > 0 时使用，到达托盘时切换)
        if (cfg["pallet_id_filter"]) {
            auto pf = cfg["pallet_id_filter"];
            app_cfg_.pallet_filter.left_filter_enable = pf["pallet_id_left_filter_enable"].as<int>(0);
            app_cfg_.pallet_filter.left_min_angle = pf["pallet_id_left_min_angle"].as<double>(0.0);
            app_cfg_.pallet_filter.left_max_angle = pf["pallet_id_left_max_angle"].as<double>(0.0);
            app_cfg_.pallet_filter.left_min_y = pf["pallet_id_left_min_y"].as<double>(0.0);
            app_cfg_.pallet_filter.left_max_y = pf["pallet_id_left_max_y"].as<double>(0.0);
            app_cfg_.pallet_filter.right_filter_enable = pf["pallet_id_right_filter_enable"].as<int>(0);
            app_cfg_.pallet_filter.right_min_angle = pf["pallet_id_right_min_angle"].as<double>(0.0);
            app_cfg_.pallet_filter.right_max_angle = pf["pallet_id_right_max_angle"].as<double>(0.0);
            app_cfg_.pallet_filter.right_min_y = pf["pallet_id_right_min_y"].as<double>(0.0);
            app_cfg_.pallet_filter.right_max_y = pf["pallet_id_right_max_y"].as<double>(0.0);
        }
        ROS_INFO("TipObstacle Config YAML Loaded Successfully.");
    } catch (const YAML::Exception& e) {
        ROS_WARN("Failed to load TipObstacle Config YAML: %s", e.what());
    }
}

// ==================== YAML 热加载监听线程 ====================
// 使用 Linux inotify 机制监听 3 个 YAML 文件的写入/移动事件
// 任一文件变更时自动重新加载全部配置，无需重启节点
void TipObstacleNode::watchYAMLThread() {
    // 创建 inotify 实例 (非阻塞模式)
    int fd = inotify_init1(IN_NONBLOCK);
    if (fd < 0) return;

    // 辅助 lambda: 为指定文件路径添加 inotify 监听
    // 返回 watch descriptor, 同时提取文件名用于后续事件匹配
    auto add_watch = [&](const std::string& path, std::string& file_name) -> int {
        if (path.empty()) return -1;
        size_t last_slash = path.find_last_of('/');
        std::string dir_path = path.substr(0, last_slash);  // 目录路径
        file_name = path.substr(last_slash + 1);            // 文件名
        // 监听文件写入完成和移动事件
        return inotify_add_watch(fd, dir_path.c_str(), IN_CLOSE_WRITE | IN_MOVED_TO);
    };

    // 为 3 个 YAML 文件分别添加监听
    std::string file_name1, file_name2, file_name3;
    int wd1 = add_watch(tf_yaml_path_, file_name1);        // 雷达外参
    int wd2 = add_watch(carport_yaml_path_, file_name2);    // 库位几何
    int wd3 = add_watch(config_yaml_path_, file_name3);     // 全局策略

    // poll 循环: 每 500ms 检查一次文件事件
    pollfd pfd = {fd, POLLIN, 0};

    while (thread_running_) {
        int ret = poll(&pfd, 1, 500);  // 超时 500ms
        if (ret > 0 && (pfd.revents & POLLIN)) {
            // 读取 inotify 事件缓冲区
            char buffer[4096] __attribute__ ((aligned(__alignof__(struct inotify_event))));
            ssize_t len = read(fd, buffer, sizeof(buffer));
            // 遍历所有事件
            for (char *ptr = buffer; ptr < buffer + len; ) {
                struct inotify_event *event = (struct inotify_event *) ptr;
                if (event->len) {
                    std::string ev_name(event->name);
                    // 匹配任意一个监听的文件名
                    if (ev_name == file_name1 || ev_name == file_name2 || ev_name == file_name3) {
                        ROS_WARN("YAML File [%s] changed, reloading all parameters...", event->name);
                        loadYAML();  // 重新加载全部配置
                    }
                }
                ptr += sizeof(struct inotify_event) + event->len;
            }
        }
    }
    
    // 清理: 移除 inotify 监听并关闭文件描述符
    if (wd1 >= 0) inotify_rm_watch(fd, wd1);
    if (wd2 >= 0) inotify_rm_watch(fd, wd2);
    if (wd3 >= 0) inotify_rm_watch(fd, wd3);
    close(fd);
}

// ==================== /arrived_flag 回调 ====================
// 更新托盘到达状态，控制过滤参数集的切换
// data > 0: 切换到 pallet_filter (托盘模式)
// data ≤ 0: 使用 normal_filter (常规模式)
void TipObstacleNode::palletIdCallback(const std_msgs::Int8::ConstPtr &msg) {
    pallet_id_state_ = msg->data;
}

// ==================== /twist_cmd 回调 ====================
// 根据速度指令判断车辆是否在倒车或转向，更新时间戳以触发/关闭防撞
void TipObstacleNode::twistCmdCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    // 读取配置 (线程安全快照)
    AppConfig cfg;
    { std::lock_guard<std::mutex> lock(cfg_mutex_); cfg = app_cfg_; }

    float vx = msg->twist.linear.x;          // 线速度 (前进为正)
    float wz = fabs(msg->twist.angular.z);    // 角速度绝对值

    ROS_INFO_THROTTLE(1.0, "Received TwistCmd: vx=%.3f m/s, wz=%.3f rad/s", vx, wz);

    // 倒车 (vx < reverse_velocity) 或 转向 (|wz| > 阈值) → 记录时间戳，触发防撞
    if (vx < cfg.reverse_velocity || wz > cfg.turning_angular_threshold) {
        last_reverse_time_.store(ros::Time::now().toSec());
    } 
    // 前进 (vx > forward_velocity) → 重置时间戳，关闭防撞
    else if (vx > cfg.forward_velocity) {
        last_reverse_time_.store(0.0);
    }
}

// ==================== /feedback_status 回调 ====================
// 接收车辆反馈状态，更新任务类型和到库位的距离
void TipObstacleNode::feedbackStatusCallback(const autoware_remove_msgs::State::ConstPtr& msg) {
    AppConfig cfg;
    { std::lock_guard<std::mutex> lock(cfg_mutex_); cfg = app_cfg_; }

    // 更新当前任务类型
    current_task_type_.store(msg->TaskInfo.type);

    // 检查任务类型是否在 YAML 允许的白名单中
    auto& v_types = cfg.valid_task_types;
    if (std::find(v_types.begin(), v_types.end(), msg->TaskInfo.type) != v_types.end()) {
        float current_dis = msg->TaskInfo.site.dis;  // 当前到库位的距离
        ROS_INFO_THROTTLE(1.0, "Received FeedbackStatus: type=%d, distance=%.3f m",
                          msg->TaskInfo.type, current_dis);
        // 仅当距离大于最小有效距离时更新
        if (current_dis > cfg.valid_distance_min) {
            dis_to_carport_.store(current_dis); 
            // 距离小于库位激活阈值时，记录泊车时间戳 (激活走廊裁切)
            if (current_dis < cfg.carport_activation_dist) {
                last_parking_time_.store(ros::Time::now().toSec());
            }
        }
    } 
}

// ==================== TF 外参矩阵构造 ====================
// 根据 TfParam 的 6DOF 参数构造 4×4 仿射变换矩阵
// 旋转顺序: Yaw (Z轴) → Pitch (Y轴) → Roll (X轴) —— 欧拉角 ZYX 约定
Eigen::Affine3f TipObstacleNode::getTransformMatrix(const TfParam& param) {
    Eigen::Affine3f mat = Eigen::Affine3f::Identity();
    mat.translation() << param.x, param.y, param.z;           // 平移分量
    mat.rotate(Eigen::AngleAxisf(param.yaw, Eigen::Vector3f::UnitZ()));    // 先绕 Z 轴旋转 (Yaw)
    mat.rotate(Eigen::AngleAxisf(param.pitch, Eigen::Vector3f::UnitY()));  // 再绕 Y 轴旋转 (Pitch)
    mat.rotate(Eigen::AngleAxisf(param.roll, Eigen::Vector3f::UnitX()));   // 最后绕 X 轴旋转 (Roll)
    return mat;
}

// ==================== 单雷达点云处理管线 ====================
// 核心处理函数: 投影 → 防撞开关判断 → 盲区过滤 → TF 变换
pcl::PointCloud<pcl::PointXYZI>::Ptr TipObstacleNode::filterAndTransformCloud(
    const sensor_msgs::LaserScan& scan_msg, bool is_left) 
{
    // --- 步骤 1: LaserScan → PointCloud2 投影 ---
    // 使用 laser_geometry 将 2D 激光扫描数据投影为 3D 点云
    // 此时点云坐标原点是雷达自身的 (0,0,0)
    sensor_msgs::PointCloud2 pc2_msg;
    projector_.projectLaser(scan_msg, pc2_msg);
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_raw(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(pc2_msg, *cloud_raw);

    // 读取配置 (线程安全快照)
    AppConfig cfg;
    { std::lock_guard<std::mutex> lock(cfg_mutex_); cfg = app_cfg_; }

    // --- 步骤 2: 防撞开关判断 ---
    // 基于 YAML 中配置的超时时间进行判断:
    //   如果当前时间与最后一次倒车/转向的时间差 < state_timeout，则防撞开启
    double current_time = ros::Time::now().toSec();
    bool is_reversing = (current_time - last_reverse_time_.load()) < cfg.state_timeout;
    bool enable_collision = is_reversing; 

    ROS_INFO_THROTTLE(1.0, "[filterAndTransform] is_left=%d, enable_collision=%d, is_reversing=%d, "
                      "time_diff=%.3f, last_reverse_time=%.3f, raw_points=%zu",
                      is_left, enable_collision, is_reversing,
                      current_time - last_reverse_time_.load(), last_reverse_time_.load(),
                      cloud_raw->points.size());

    // 防撞未激活且非调试模式 → 返回空点云 (不发布任何避障数据)
    if (!enable_collision && !debug_mode_) {
        ROS_INFO_THROTTLE(1.0, "[filterAndTransform] Collision disabled, returning empty cloud.");
        return cloud_filtered;
    }

    // --- 步骤 3: 根据托盘状态选择对应的过滤参数集 ---
    // pallet_id_state > 0: 使用 pallet_filter (托盘模式，到达托盘时的特殊过滤)
    // 否则:               使用 normal_filter (常规模式)
    bool enable_filter = false;
    double min_ang = 0, max_ang = 0, min_y = 0, max_y = 0;

    if (pallet_id_state_ > 0) {
        // 托盘模式: 使用 pallet_id_filter 段的参数
        enable_filter = is_left ? (cfg.pallet_filter.left_filter_enable == 1) : (cfg.pallet_filter.right_filter_enable == 1);
        min_ang = is_left ? cfg.pallet_filter.left_min_angle : cfg.pallet_filter.right_min_angle;
        max_ang = is_left ? cfg.pallet_filter.left_max_angle : cfg.pallet_filter.right_max_angle;
        min_y   = is_left ? cfg.pallet_filter.left_min_y : cfg.pallet_filter.right_min_y;
        max_y   = is_left ? cfg.pallet_filter.left_max_y : cfg.pallet_filter.right_max_y;
    } else {
        // 常规模式: 使用 normal_filter 段的参数
        enable_filter = is_left ? (cfg.normal_filter.left_filter_enable == 1) : (cfg.normal_filter.right_filter_enable == 1);
        min_ang = is_left ? cfg.normal_filter.left_min_angle : cfg.normal_filter.right_min_angle;
        max_ang = is_left ? cfg.normal_filter.left_max_angle : cfg.normal_filter.right_max_angle;
        min_y   = is_left ? cfg.normal_filter.left_min_y : cfg.normal_filter.right_min_y;
        max_y   = is_left ? cfg.normal_filter.left_max_y : cfg.normal_filter.right_max_y;
    }

    // --- 步骤 4: 盲区过滤 (在雷达原始坐标系下进行) ---
    // 对每个点进行两项检查:
    //   a) 角度过滤: 计算 atan2(y,x)，剔除落在 [min_angle, max_angle] 盲区范围内的点
    //   b) Y轴过滤:  剔除 Y 坐标落在 [min_y, max_y] 盲区范围内的点
    // 注意: 当 min > max 时，表示盲区跨越 0°/360° 边界，需要特殊处理
    for (const auto& pt : cloud_raw->points) {
        if (!debug_mode_ && enable_filter) {
            // 角度过滤: 将角度归一化到 [0, 360) 度
            double angle = atan2(pt.y, pt.x);
            angle = fmod((angle * 180.0 / M_PI) + 360.0, 360.0);
            
            // 判断是否在盲区角度范围内
            bool is_in_blind_angle = false;
            if (min_ang > max_ang) {
                // 盲区跨越 0°: 角度 > min 或 角度 < max 即在盲区内
                if (angle > min_ang || angle < max_ang) is_in_blind_angle = true;
            } else {
                // 正常情况: min < angle < max 即在盲区内
                if (angle > min_ang && angle < max_ang) is_in_blind_angle = true;
            }
            if (is_in_blind_angle) continue;  // 跳过盲区内的点

            // Y轴过滤
            bool is_in_blind_y = false;
            if (min_y > max_y) {
                // Y 范围跨越 0: y > min 或 y < max 即在盲区内
                if (pt.y > min_y || pt.y < max_y) is_in_blind_y = true;
            } else {
                // 正常情况: min < y < max 即在盲区内
                if (pt.y > min_y && pt.y < max_y) is_in_blind_y = true;
            }
            if (is_in_blind_y) continue;  // 跳过 Y 轴盲区内的点
        } 
        cloud_filtered->points.push_back(pt);  // 保留通过过滤的点
    }

    // --- 步骤 5: TF 外参变换 ---
    // 将过滤后的局部点云从雷达坐标系变换到 velodyne 坐标系
    TfParam tf_cfg;
    {
        std::lock_guard<std::mutex> lock(tf_mutex_);
        tf_cfg = is_left ? left_tf_ : right_tf_;
    }
    pcl::transformPointCloud(*cloud_filtered, *cloud_filtered, getTransformMatrix(tf_cfg));

    return cloud_filtered;
}

// ==================== 最近距离计算 ====================
// 计算点云中所有点到雷达安装位置的最短欧氏距离
// 距离是在 velodyne 坐标系下，以雷达外参的平移分量 (安装位置) 为参考点
float TipObstacleNode::calculateMinDisToLidar(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, bool is_left) {
    AppConfig cfg;
    { std::lock_guard<std::mutex> lock(cfg_mutex_); cfg = app_cfg_; }
    
    // 初始化为最大检测距离 (默认 255m)，表示无有效点
    float min_dis = cfg.max_detect_distance; 
    
    // 获取雷达安装位置 (外参平移分量)
    TfParam tf_cfg;
    {
        std::lock_guard<std::mutex> lock(tf_mutex_);
        tf_cfg = is_left ? left_tf_ : right_tf_;
    }

    // 遍历点云，计算每个点到雷达安装位置的欧氏距离
    for (const auto& pt : cloud->points) {
        float dx = pt.x - tf_cfg.x;
        float dy = pt.y - tf_cfg.y;
        float dz = pt.z - tf_cfg.z;
        float r = sqrt(dx * dx + dy * dy + dz * dz);
        if (r < min_dis) {
            min_dis = r;
        }
    }
    return min_dis;
}

// ==================== 库位动态安全走廊裁切 ====================
// 条件性二次裁切: 仅在倒车且接近库位时激活
// 在 base_link 坐标系下根据库位距离动态计算 AABB 裁切范围
void TipObstacleNode::applyCarportFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const ros::Time& stamp) {
    if (debug_mode_) return;  // 调试模式跳过
    
    AppConfig cfg;
    { std::lock_guard<std::mutex> lock(cfg_mutex_); cfg = app_cfg_; }

    // 判断两个激活条件是否同时满足:
    // 1. is_reversing: 当前正在倒车/转向 (时间戳未超时)
    // 2. is_parking:   当前正在接近库位 (时间戳未超时)
    double current_time = ros::Time::now().toSec();
    bool is_reversing = (current_time - last_reverse_time_.load()) < cfg.state_timeout;
    bool is_parking   = (current_time - last_parking_time_.load()) < cfg.state_timeout;

    // 两个条件必须同时满足才激活走廊裁切
    if (!is_reversing || !is_parking) return;

    // --- 步骤 1: velodyne → base_link 坐标变换 ---
    // 走廊裁切在 base_link 坐标系下进行 (因为库位几何参数是相对于 base_link 定义的)
    bool need_tf_transform = (parent_frame_ != base_link_frame_);
    Eigen::Affine3f tf_velodyne_to_baselink = Eigen::Affine3f::Identity();
    
    if (need_tf_transform) {
        tf::StampedTransform transform;
        try {
            // 通过 TF 监听器查找 velodyne → base_link 的变换
            tf_listener_.waitForTransform(base_link_frame_, parent_frame_, stamp, ros::Duration(cfg.tf_timeout));
            tf_listener_.lookupTransform(base_link_frame_, parent_frame_, stamp, transform);
        } catch (tf::TransformException &ex) {
            ROS_WARN_THROTTLE(1.0, "Carport TF time sync failed: %s", ex.what());
            return;  // TF 查找失败，跳过本次裁切
        }

        // 构造变换矩阵
        tf_velodyne_to_baselink.translation() << transform.getOrigin().x(), 
                                                 transform.getOrigin().y(), 
                                                 transform.getOrigin().z();
        tf::Quaternion q = transform.getRotation();
        Eigen::Quaternionf eigen_q(q.w(), q.x(), q.y(), q.z());
        tf_velodyne_to_baselink.rotate(eigen_q);

        // 将点云从 velodyne 变换到 base_link
        pcl::transformPointCloud(*cloud, *cloud, tf_velodyne_to_baselink);
    }

    // --- 步骤 2: 动态计算 AABB 裁切范围 ---
    // X 范围根据当前到库位的距离 (dis_to_carport_) 动态调整:
    //   abs_min_x = -dis + (min_x - cx)   → 走廊起始位置随距离移动
    //   abs_max_x = max(-dis + (max_x - cx), 0)  → 走廊末端不超过车身
    float current_dis = dis_to_carport_.load();
    pcl::PointCloud<pcl::PointXYZI>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZI>());

    double min_x, max_x, min_y, max_y, min_z, max_z;
    {
        std::lock_guard<std::mutex> lock(cfg_mutex_);
        min_x = carports_min_x_; max_x = carports_max_x_;
        min_y = carports_min_y_; max_y = carports_max_y_;
        min_z = carports_min_z_; max_z = carports_max_z_;
    }

    // 计算走廊中心点 (用于将局部坐标转换为绝对坐标)
    double cx = (min_x + max_x) / 2.0;
    double cy = (min_y + max_y) / 2.0;

    // 动态 AABB 范围 (base_link 坐标系下)
    double abs_min_x = -current_dis + (min_x - cx);
    double original_abs_max_x = -current_dis + (max_x - cx);
    double abs_max_x = std::max(original_abs_max_x, 0.0);  // 确保不超过车身前端

    double abs_min_y = min_y - cy;
    double abs_max_y = max_y - cy;
    double abs_min_z = min_z;
    double abs_max_z = max_z;

    // --- 步骤 3: AABB 裁切 ---
    // 仅保留在走廊包络内的点
    for (const auto& pt : cloud->points) {
        if (pt.x >= abs_min_x && pt.x <= abs_max_x &&
            pt.y >= abs_min_y && pt.y <= abs_max_y &&
            pt.z >= abs_min_z && pt.z <= abs_max_z) 
        {
            cropped_cloud->points.push_back(pt);
        }
    }

    cropped_cloud->width = cropped_cloud->points.size();
    cropped_cloud->height = 1;
    cropped_cloud->is_dense = true;

    // --- 步骤 4: 逆变换回 velodyne 坐标系 ---
    if (need_tf_transform) {
        Eigen::Affine3f tf_baselink_to_velodyne = tf_velodyne_to_baselink.inverse();
        pcl::transformPointCloud(*cropped_cloud, *cloud, tf_baselink_to_velodyne);
    } else {
        *cloud = *cropped_cloud;
    }
}

// ==================== 可视区域 Marker 发布 ====================
// 发布每个雷达的可视角度区域 (盲区的补集) 到 map 坐标系
// 使用蓝色半透明扇形表示，与库位走廊的绿色线框区分
void TipObstacleNode::publishVisibilityMarker() {
    AppConfig cfg;
    { std::lock_guard<std::mutex> lock(cfg_mutex_); cfg = app_cfg_; }

    if (!cfg.visibility_marker_enable) return;  // 可视区域 Marker 被禁用

    visualization_msgs::MarkerArray marker_array;

    // 辅助结构: 存储可视区域参数
    struct VisParam {
        int filter_enable;
        double min_angle, max_angle;   // 盲区角度范围 [0, 360)
        double min_y, max_y;           // 盲区 Y 轴范围
    };

    // 根据托盘状态选择对应的过滤参数
    auto getVisParam = [&](bool is_left) -> VisParam {
        VisParam vp;
        if (pallet_id_state_ > 0) {
            // 托盘模式
            vp.filter_enable = is_left ? cfg.pallet_filter.left_filter_enable : cfg.pallet_filter.right_filter_enable;
            vp.min_angle     = is_left ? cfg.pallet_filter.left_min_angle : cfg.pallet_filter.right_min_angle;
            vp.max_angle     = is_left ? cfg.pallet_filter.left_max_angle : cfg.pallet_filter.right_max_angle;
            vp.min_y         = is_left ? cfg.pallet_filter.left_min_y : cfg.pallet_filter.right_min_y;
            vp.max_y         = is_left ? cfg.pallet_filter.left_max_y : cfg.pallet_filter.right_max_y;
        } else {
            // 常规模式
            vp.filter_enable = is_left ? cfg.normal_filter.left_filter_enable : cfg.normal_filter.right_filter_enable;
            vp.min_angle     = is_left ? cfg.normal_filter.left_min_angle : cfg.normal_filter.right_min_angle;
            vp.max_angle     = is_left ? cfg.normal_filter.left_max_angle : cfg.normal_filter.right_max_angle;
            vp.min_y         = is_left ? cfg.normal_filter.left_min_y : cfg.normal_filter.right_min_y;
            vp.max_y         = is_left ? cfg.normal_filter.left_max_y : cfg.normal_filter.right_max_y;
        }
        return vp;
    };

    double ref_dist = cfg.visibility_ref_distance;  // 扇形半径 (参考距离)
    int marker_id = 0;

    // 为每个雷达生成可视区域 Marker
    // 使用 tf_listener 查找 map→scan_frame 的变换，将 Marker 发布到 map 坐标系
    auto publishForLidar = [&](bool is_left, const TfParam& tf_cfg, const std::string& scan_frame) {
        VisParam vp = getVisParam(is_left);
        if (!vp.filter_enable) return;  // 该雷达的过滤未启用

        // 查找 map → scan_frame 的 TF 变换
        tf::StampedTransform map_to_scan;
        try {
            tf_listener_.waitForTransform("map", scan_frame, ros::Time(0), ros::Duration(0.1));
            tf_listener_.lookupTransform("map", scan_frame, ros::Time(0), map_to_scan);
        } catch (tf::TransformException &ex) {
            ROS_WARN_THROTTLE(2.0, "Visibility Marker TF lookup failed [%s]: %s", scan_frame.c_str(), ex.what());
            return;
        }

        // 构造 map → scan_frame 的变换矩阵
        Eigen::Affine3f tf_map_to_scan = Eigen::Affine3f::Identity();
        tf_map_to_scan.translation() << map_to_scan.getOrigin().x(),
                                        map_to_scan.getOrigin().y(),
                                        map_to_scan.getOrigin().z();
        tf::Quaternion q = map_to_scan.getRotation();
        Eigen::Quaternionf eigen_q(q.w(), q.x(), q.y(), q.z());
        tf_map_to_scan.rotate(eigen_q);

        // 创建 LINE_LIST 类型 Marker
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = is_left ? "visibility_left" : "visibility_right";
        marker.id = marker_id++;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = cfg.marker_line_width;
        // 可视区域使用半透明蓝色，与库位走廊的绿色区分
        marker.color.r = 0.0;
        marker.color.g = 0.5;
        marker.color.b = 1.0;
        marker.color.a = 0.6;

        // 辅助函数: 将雷达局部坐标系下的角度转换为 map 坐标系下的 2D 点
        auto angleToPoint = [&](double angle_deg) -> geometry_msgs::Point {
            double angle_rad = angle_deg * M_PI / 180.0;
            // 雷达局部坐标系下的点 (在 ref_dist 距离处)
            Eigen::Vector3f local_pt(ref_dist * cos(angle_rad), ref_dist * sin(angle_rad), 0.0);
            // 变换到 map 坐标系
            Eigen::Vector3f map_pt = tf_map_to_scan * local_pt;
            geometry_msgs::Point p;
            p.x = map_pt.x();
            p.y = map_pt.y();
            p.z = map_pt.z();
            return p;
        };

        // 雷达在 map 坐标系下的位置 (作为扇形原点)
        Eigen::Vector3f origin_eigen = tf_map_to_scan * Eigen::Vector3f(0.0, 0.0, 0.0);
        geometry_msgs::Point origin;
        origin.x = origin_eigen.x();
        origin.y = origin_eigen.y();
        origin.z = origin_eigen.z();

        // 计算可视角度区间并绘制径向线和弧线
        // 弧线用若干线段近似 (arc_segments 段)
        const int arc_segments = 8;

        auto addArcAndRays = [&](double vis_start_deg, double vis_end_deg) {
            if (vis_start_deg >= vis_end_deg) return;

            // 两条径向线: 从原点到 ref_dist 距离处
            geometry_msgs::Point ps = angleToPoint(vis_start_deg);
            geometry_msgs::Point pe = angleToPoint(vis_end_deg);

            marker.points.push_back(origin);
            marker.points.push_back(ps);

            marker.points.push_back(origin);
            marker.points.push_back(pe);

            // 弧线: 用 arc_segments 段线段近似
            for (int i = 0; i < arc_segments; ++i) {
                double a1 = vis_start_deg + (vis_end_deg - vis_start_deg) * i / arc_segments;
                double a2 = vis_start_deg + (vis_end_deg - vis_start_deg) * (i + 1) / arc_segments;
                marker.points.push_back(angleToPoint(a1));
                marker.points.push_back(angleToPoint(a2));
            }
        };

        // 根据盲区角度范围计算可视区域 (盲区的补集)
        if (vp.min_angle > vp.max_angle) {
            // 盲区跨越 0°: [min_angle, 360) ∪ [0, max_angle]
            // 可视区域为: [max_angle, min_angle] (不跨 0° 的连续区间)
            addArcAndRays(vp.max_angle, vp.min_angle);
        } else {
            // 盲区: [min_angle, max_angle]
            // 可视区域为两段: [0, min_angle] ∪ [max_angle, 360)
            addArcAndRays(0.0, vp.min_angle);
            addArcAndRays(vp.max_angle, 360.0);
        }

        // Y 轴边界线: 沿 y=min_y 和 y=max_y 画水平线段
        // 仅在 Y 轴过滤开启 (min_y != max_y) 时绘制
        if (fabs(vp.min_y - vp.max_y) > 1e-6) {
            for (double y_val : {vp.min_y, vp.max_y}) {
                Eigen::Vector3f local_p1(0.0, y_val, 0.0);
                Eigen::Vector3f local_p2(ref_dist, y_val, 0.0);
                Eigen::Vector3f map_p1 = tf_map_to_scan * local_p1;
                Eigen::Vector3f map_p2 = tf_map_to_scan * local_p2;
                geometry_msgs::Point p1, p2;
                p1.x = map_p1.x(); p1.y = map_p1.y(); p1.z = map_p1.z();
                p2.x = map_p2.x(); p2.y = map_p2.y(); p2.z = map_p2.z();
                marker.points.push_back(p1);
                marker.points.push_back(p2);
            }
        }

        if (!marker.points.empty()) {
            marker_array.markers.push_back(marker);
        }
    };

    // 为左雷达和右雷达 (双叉臂模式) 分别生成可视区域 Marker
    {
        publishForLidar(true, left_tf_, left_scan_frame_);
        if (tip_type_ == 0) {
            // 双叉臂模式才需要右雷达的可视区域
            publishForLidar(false, right_tf_, right_scan_frame_);
        }
    }

    // 发布 Marker 数组
    if (!marker_array.markers.empty()) {
        visibility_marker_pub_.publish(marker_array);
    }
}

// ==================== 库位走廊包围盒 Marker 发布 ====================
// 发布库位安全走廊的 3D 包围盒线框 (绿色，frame=base_link)
// 仅在 is_reversing && is_parking 双条件满足时显示，否则清除
void TipObstacleNode::publishCarportMarker() {
    AppConfig cfg;
    { std::lock_guard<std::mutex> lock(cfg_mutex_); cfg = app_cfg_; }

    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker box_marker;

    box_marker.header.frame_id = base_link_frame_;
    box_marker.header.stamp = ros::Time::now();
    box_marker.ns = "carport_boundary";
    box_marker.id = 0;

    // 检查双条件: 倒车中 + 泊车中
    double current_time = ros::Time::now().toSec();
    bool is_reversing = (current_time - last_reverse_time_.load()) < cfg.state_timeout;
    bool is_parking   = (current_time - last_parking_time_.load()) < cfg.state_timeout;

    // 条件不满足时，如果之前已发布过 Marker，则发送 DELETE 指令清除
    if (!is_parking || !is_reversing) {
        if (marker_published_) {
            box_marker.action = visualization_msgs::Marker::DELETE;
            marker_array.markers.push_back(box_marker);
            carport_marker_pub_.publish(marker_array);
            marker_published_ = false; 
        }
        return;
    }

    marker_published_ = true;

    // 配置 Marker 外观: LINE_LIST 类型的 3D 线框
    box_marker.type = visualization_msgs::Marker::LINE_LIST;
    box_marker.action = visualization_msgs::Marker::ADD;
    box_marker.pose.orientation.w = 1.0;
    
    // 使用 YAML 中配置的可视化参数 (颜色、线宽)
    box_marker.scale.x = cfg.marker_line_width; 
    box_marker.color.r = cfg.marker_color_r;
    box_marker.color.g = cfg.marker_color_g;
    box_marker.color.b = cfg.marker_color_b;
    box_marker.color.a = cfg.marker_color_a;

    // 计算走廊 AABB 的 8 个顶点 (在 base_link 坐标系下)
    float current_dis = dis_to_carport_.load();
    double min_x, max_x, min_y, max_y, min_z, max_z;
    {
        std::lock_guard<std::mutex> lock(cfg_mutex_);
        min_x = carports_min_x_; max_x = carports_max_x_;
        min_y = carports_min_y_; max_y = carports_max_y_;
        min_z = carports_min_z_; max_z = carports_max_z_;
    }

    // 计算走廊中心点
    double cx = (min_x + max_x) / 2.0;
    double cy = (min_y + max_y) / 2.0;

    // 动态 AABB 范围 (与 applyCarportFilter 中的计算一致)
    double abs_min_x = -current_dis + (min_x - cx);
    double original_abs_max_x = -current_dis + (max_x - cx);
    double abs_max_x = std::max(original_abs_max_x, 0.0);  // 不超过车身前端
    
    double abs_min_y = min_y - cy;
    double abs_max_y = max_y - cy;
    double abs_min_z = min_z; 
    double abs_max_z = max_z; 

    // 定义包围盒的 8 个顶点
    geometry_msgs::Point p[8];
    // 底面 4 个顶点 (z = min_z)
    p[0].x = abs_min_x; p[0].y = abs_min_y; p[0].z = abs_min_z;
    p[1].x = abs_max_x; p[1].y = abs_min_y; p[1].z = abs_min_z;
    p[2].x = abs_max_x; p[2].y = abs_max_y; p[2].z = abs_min_z;
    p[3].x = abs_min_x; p[3].y = abs_max_y; p[3].z = abs_min_z;
    // 顶面 4 个顶点 (z = max_z)
    p[4].x = abs_min_x; p[4].y = abs_min_y; p[4].z = abs_max_z;
    p[5].x = abs_max_x; p[5].y = abs_min_y; p[5].z = abs_max_z;
    p[6].x = abs_max_x; p[6].y = abs_max_y; p[6].z = abs_max_z;
    p[7].x = abs_min_x; p[7].y = abs_max_y; p[7].z = abs_max_z;

    // 定义 12 条边 (底面4条 + 顶面4条 + 竖直4条)
    int edges[12][2] = {
        {0,1}, {1,2}, {2,3}, {3,0},  // 底面
        {4,5}, {5,6}, {6,7}, {7,4},  // 顶面
        {0,4}, {1,5}, {2,6}, {3,7}   // 竖直边
    };

    // 将 12 条边添加到 Marker (LINE_LIST 格式: 每两个点构成一条线段)
    for (int i = 0; i < 12; ++i) {
        box_marker.points.push_back(p[edges[i][0]]);
        box_marker.points.push_back(p[edges[i][1]]);
    }

    marker_array.markers.push_back(box_marker);
    carport_marker_pub_.publish(marker_array);
}

// ==================== 双叉臂模式同步回调 ====================
// 同时接收左右雷达数据，执行完整的处理管线
void TipObstacleNode::scanCallbackSync(const sensor_msgs::LaserScan::ConstPtr &msg1,
                                       const sensor_msgs::LaserScan::ConstPtr &msg2) 
{
    // 步骤 1: 分别对左右雷达执行 投影→过滤→TF变换
    auto left_cloud = filterAndTransformCloud(*msg1, true);
    auto right_cloud = filterAndTransformCloud(*msg2, false);

    // 步骤 2: 库位走廊裁切 (条件触发)
    applyCarportFilter(left_cloud, msg1->header.stamp);
    applyCarportFilter(right_cloud, msg2->header.stamp);

    // 步骤 3: 发布可视化 Marker
    publishCarportMarker();
    publishVisibilityMarker();

    // 步骤 4: 计算左右雷达各自的最近障碍物距离，取最小值
    float min_dis_left = calculateMinDisToLidar(left_cloud, true);
    float min_dis_right = calculateMinDisToLidar(right_cloud, false);
    float final_min_dis = std::min(min_dis_left, min_dis_right);

    // 步骤 5: 发布左雷达点云 (frame=velodyne)
    sensor_msgs::PointCloud2 leftOutMsg, rightOutMsg;
    pcl::toROSMsg(*left_cloud, leftOutMsg);
    leftOutMsg.header.frame_id = parent_frame_;
    leftOutMsg.header.stamp = msg1->header.stamp;
    pc_left_pub_.publish(leftOutMsg);

    // 步骤 6: 发布右雷达点云 (frame=velodyne)
    pcl::toROSMsg(*right_cloud, rightOutMsg);
    rightOutMsg.header.frame_id = parent_frame_;
    rightOutMsg.header.stamp = msg2->header.stamp;
    pc_right_pub_.publish(rightOutMsg);

    // 步骤 7: 左右点云拼接融合并发布 (frame=velodyne)
    *left_cloud += *right_cloud;
    sensor_msgs::PointCloud2 fusedOutMsg;
    pcl::toROSMsg(*left_cloud, fusedOutMsg);
    fusedOutMsg.header.frame_id = parent_frame_;
    fusedOutMsg.header.stamp = msg1->header.stamp;
    pc_fused_pub_.publish(fusedOutMsg);

    // 步骤 8: 发布最近障碍物距离
    std_msgs::Float32 dis_msg;
    dis_msg.data = final_min_dis;
    min_dis_pub_.publish(dis_msg);
}

// ==================== 单叉臂模式回调 ====================
// 仅接收左雷达数据，执行完整的处理管线
void TipObstacleNode::scanCallbackSingle(const sensor_msgs::LaserScan::ConstPtr &msg) 
{
    ROS_INFO_THROTTLE(1.0, "[scanCallbackSingle] Received scan, ranges_size=%zu", msg->ranges.size());

    // 步骤 1: 投影→过滤→TF变换
    auto left_cloud = filterAndTransformCloud(*msg, true);
    ROS_INFO_THROTTLE(1.0, "[scanCallbackSingle] After filter: cloud_size=%zu", left_cloud->points.size());

    // 步骤 2: 库位走廊裁切 (条件触发)
    applyCarportFilter(left_cloud, msg->header.stamp);

    // 步骤 3: 发布可视化 Marker
    publishCarportMarker();
    publishVisibilityMarker();

    // 步骤 4: 计算最近障碍物距离
    float final_min_dis = calculateMinDisToLidar(left_cloud, true);
    ROS_INFO_THROTTLE(1.0, "[scanCallbackSingle] Final: cloud_size=%zu, min_dis=%.3f", left_cloud->points.size(), final_min_dis);

    // 步骤 5: 发布左雷达点云 (frame=velodyne)
    sensor_msgs::PointCloud2 leftOutMsg;
    pcl::toROSMsg(*left_cloud, leftOutMsg);
    leftOutMsg.header.frame_id = parent_frame_;
    leftOutMsg.header.stamp = msg->header.stamp;
    pc_left_pub_.publish(leftOutMsg);

    // 步骤 6: 单叉臂模式下，融合点云 = 左雷达点云 (直接复制发布)
    sensor_msgs::PointCloud2 fusedOutMsg = leftOutMsg;
    pc_fused_pub_.publish(fusedOutMsg);

    // 步骤 7: 发布最近障碍物距离
    std_msgs::Float32 dis_msg;
    dis_msg.data = final_min_dis;
    min_dis_pub_.publish(dis_msg);
}

// ==================== 主函数 ====================
int main(int argc, char **argv) {
    ros::init(argc, argv, "tip_obstacle");  // 初始化 ROS 节点
    ros::NodeHandle nh;                      // 全局句柄
    ros::NodeHandle pnh("~");                // 私有句柄 (~tip_obstacle_node)

    TipObstacleNode node(nh, pnh);           // 创建节点实例

    ros::spin();  // 进入事件循环，等待回调
    return 0;
}