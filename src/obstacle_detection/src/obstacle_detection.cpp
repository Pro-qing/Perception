/**
 * @file obstacle_detection.cpp
 * @brief 障碍物检测节点主入口 + 构造函数 + 核心回调
 *
 * 本文件包含:
 * - main() 函数
 * - 构造函数(参数加载、订阅/发布初始化)
 * - syncCloudCallback() 双雷达同步回调(核心处理入口)
 * - timerCallback() 定时发布检测状态
 * - checkAndReloadYaml() YAML动态重载
 * - setLogLevel() 日志级别设置
 */

#include "obstacle_detection/IrregularPolygonFilter.hpp"
#include "obstacle_detection/obstacle_detection.hpp"
#include <boost/bind.hpp>
#include <cstdlib>
#include <ros/package.h>

/**
 * @brief 主函数 - 节点入口
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "obstacle_detection");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ObstacleDetection detector(nh, private_nh);
    ros::spin();
    return 0;
}

ObstacleDetection::ObstacleDetection(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
    : nh_(nh), private_nh_(private_nh) {
    setLogLevel();

    // ========== 加载基础参数 ==========
    private_nh_.param<std::string>("target_frame", target_frame_, "base_link");
    private_nh_.param<double>("voxel_leaf_size", voxel_leaf_size_, 0.05);
    private_nh_.param<double>("cluster_tolerance", cluster_tolerance_, 0.15);
    private_nh_.param<int>("min_cluster_size", min_cluster_size_, 5);
    private_nh_.param<int>("max_cluster_size", max_cluster_size_, 1000);
    private_nh_.param<double>("z_axis_min", z_axis_min_, -0.3);
    private_nh_.param<double>("z_axis_max", z_axis_max_, 1.5);
    private_nh_.param<double>("ground_threshold", ground_threshold_, 0.08);
    private_nh_.param<double>("roi_radius", roi_radius_, 10.0);
    private_nh_.param<bool>("use_roi_filter", use_roi_filter_, true);
    private_nh_.param<std::string>("points_mid_topic", points_mid_topic_, "/points_mid");
    private_nh_.param<double>("distance_threshold", distance_threshold_, 0.5);
    private_nh_.param<int>("min_region_points", min_region_points_, 5);
    private_nh_.param<int>("garage_history_size", garage_history_size_, 5);
    private_nh_.param<int>("garage_confirm_threshold", garage_confirm_threshold_, 3);
    private_nh_.param<double>("garage_enable_distance", garage_enable_distance_, 6.0);

    // ========== 创建订阅者(双雷达时间同步) ==========
    mid_cloud_sub_.subscribe(nh_, points_mid_topic_, 5);
    tip_cloud_sub_.subscribe(nh_, "/fused_points_tip", 5);
    sync_.reset(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), mid_cloud_sub_, tip_cloud_sub_));
    sync_->registerCallback(boost::bind(&ObstacleDetection::syncCloudCallback, this, _1, _2));

    // ========== 创建其他订阅者 ==========
    keyPointPath_sub_ = nh_.subscribe<autoware_msgs::KeyPointArray>("/keypoint_path", 1, &ObstacleDetection::keyPointPathCallback, this);
    feedback_status_sub_ = nh_.subscribe<autoware_remove_msgs::State>("/feedback_status", 1, &ObstacleDetection::feedbackStatusCallback, this);
    elevator_info_sub_ = nh_.subscribe<autoware_msgs::ElevatorInfo>("/elevator_info", 1, &ObstacleDetection::elevatorInfoCallback, this);
    lqr_dire_sub_ = nh_.subscribe<std_msgs::Int8>("/lqr_dire", 1, &ObstacleDetection::lqrDireCallback, this);
    floor_set_sub_ = nh_.subscribe<std_msgs::Int8>("/floor_set", 1, &ObstacleDetection::floorSetCallback, this);
    current_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/current_pose", 1, &ObstacleDetection::currentPoseCallback, this);

    // ========== 创建发布者 ==========
    ground_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/ground_points_mid", 1);
    obstacle_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/obstacle_points_mid", 1);
    cluster_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/clustered_points_mid", 1);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/obstacle_detection_markers", 1);
    calibration_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/calibration_points_mid", 1);
    target_region_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/target_region_points", 1);
    obstacle_detection_pub_ = nh_.advertise<std_msgs::UInt32>("/obstacle_detection", 1);
    fused_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/points_fused_detection", 1);

    // ========== 创建定时器 ==========
    publish_timer_ = nh_.createTimer(ros::Duration(1.0/10), &ObstacleDetection::timerCallback, this, false);

    // ========== YAML动态重载初始化 ==========
    private_nh_.param<std::string>("yaml_file_path", yaml_file_path_, "");
    if (yaml_file_path_.empty()) {
        std::string pkg_path = ros::package::getPath("obstacle_detection");
        yaml_file_path_ = pkg_path + "/params/obstacle_detection.yaml";
    }
    struct stat file_stat;
    if (stat(yaml_file_path_.c_str(), &file_stat) == 0) {
        last_yaml_mod_time_ = file_stat.st_mtime;
        ROS_INFO("YAML reload: monitoring file [%s]", yaml_file_path_.c_str());
    } else {
        last_yaml_mod_time_ = 0;
        ROS_WARN("YAML reload: cannot stat file [%s], auto-reload disabled", yaml_file_path_.c_str());
    }
    yaml_reload_timer_ = nh_.createTimer(ros::Duration(1.0), &ObstacleDetection::checkAndReloadYaml, this, false);

    // ========== 初始化默认值 ==========
    target_region_has_noise_ = false;
    floor_set_ = 0;
    targetPoint_.map_pose.pose.position.x = 0;
    targetPoint_.map_pose.pose.position.y = 0;
    targetPoint_.map_pose.pose.position.z = 0;
    targetPoint_.map_pose.pose.orientation.x = 0;
    targetPoint_.map_pose.pose.orientation.y = 0;
    targetPoint_.map_pose.pose.orientation.z = 0;
    targetPoint_.map_pose.pose.orientation.w = 1;

    current_pose_.pose.position.x = 0;
    current_pose_.pose.position.y = 0;
    current_pose_.pose.position.z = 0;
    current_pose_.pose.orientation.x = 0;
    current_pose_.pose.orientation.y = 0;
    current_pose_.pose.orientation.z = 0;
    current_pose_.pose.orientation.w = 1;
    current_pose_.header.frame_id = "map";

    obstacle_detection_.data = 0;
    ROS_INFO("Mid-range LiDAR Obstacle Detection Node Initialized");
    ROS_INFO("Subscribed to: %s", points_mid_topic_.c_str());
    ROS_INFO("Target frame: %s", target_frame_.c_str());
    ROS_INFO("Distance threshold: %.2f m", distance_threshold_);
}

// ========== 定时器回调 - 10Hz发布检测状态 ==========
void ObstacleDetection::timerCallback(const ros::TimerEvent& event)
{
    obstacle_detection_pub_.publish(obstacle_detection_);
}

// ========== 双雷达同步回调 - 核心处理入口 ==========
void ObstacleDetection::syncCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& mid_msg,
                                          const sensor_msgs::PointCloud2::ConstPtr& tip_msg) {
    // 目标点未启用时，清除所有检测标志位并直接返回
    if (!targetPoint_.enable) {
        obstacle_detection_.data &= ~1;
        obstacle_detection_.data &= ~(1 << 1);
        return;
    }

    // 预处理mid点云
    MidProcessResult mid_result;
    if (!preprocessMidCloud(mid_msg, mid_result) || !mid_result.valid) {
        obstacle_detection_.data &= ~1;
        obstacle_detection_.data &= ~(1 << 1);
        return;
    }

    // 动态加载库位检测区域参数
    private_nh_.getParam("/obstacle_detection/carports_min_x", carports_min_x_);
    private_nh_.getParam("/obstacle_detection/carports_max_x", carports_max_x_);
    private_nh_.getParam("/obstacle_detection/carports_min_y", carports_min_y_);
    private_nh_.getParam("/obstacle_detection/carports_max_y", carports_max_y_);
    private_nh_.getParam("/obstacle_detection/carports_min_z", carports_min_z_);
    private_nh_.getParam("/obstacle_detection/carports_max_z", carports_max_z_);

    // 变换tip点云到目标坐标系
    PointCloud::Ptr tip_cloud(new PointCloud);
    pcl::fromROSMsg(*tip_msg, *tip_cloud);
    if (tip_cloud->empty()) {
        ROS_WARN_THROTTLE(1.0, "Received empty fused_points_tip point cloud");
    }

    PointCloud::Ptr tip_transformed(new PointCloud);
    if (target_frame_ != tip_msg->header.frame_id) {
        if (!transformPointCloud(tip_cloud, tip_transformed, tip_msg->header.frame_id, target_frame_)) {
            ROS_WARN("Transform tip cloud failed, using original frame");
            *tip_transformed = *tip_cloud;
        }
    } else {
        *tip_transformed = *tip_cloud;
    }

    // 融合两路点云
    PointCloud::Ptr fused_cloud(new PointCloud);
    *fused_cloud = *(mid_result.filtered_cloud);
    fused_cloud->insert(fused_cloud->end(), tip_transformed->begin(), tip_transformed->end());
    fused_cloud->width = fused_cloud->size();
    fused_cloud->height = 1;

    sensor_msgs::PointCloud2 fused_msg;
    pcl::toROSMsg(*fused_cloud, fused_msg);
    fused_msg.header = mid_result.header;
    fused_cloud_pub_.publish(fused_msg);

    std::vector<pcl::PointIndices> cluster_indices;
    std::vector<ObstacleInfo> obstacles;

    // 目标区域障碍物检测
    checkTargetPointRegion(fused_cloud, fused_msg.header);

    // 根据目标类型设置检测标志位
    if (targetPoint_.type_e == POINT_TYPE_ELEVATOR) {
        if (target_region_has_noise_) {
            obstacle_detection_.data |= 1;
        } else {
            obstacle_detection_.data &= ~1;
        }
        if (fused_cloud->size() > static_cast<size_t>(min_cluster_size_)) {
            performClustering(fused_cloud, cluster_indices);
            processClusters(fused_cloud, cluster_indices, obstacles, fused_msg.header);
        }
        publishPointClouds(mid_result.ground_cloud, fused_cloud, cluster_indices, fused_msg.header);
        publishObstacleInfo(obstacles, fused_msg.header);
    } else if (targetPoint_.type_e == POINT_TYPE_GARAGE) {
        // 库位场景: 滑动窗口防抖
        garage_detection_history_.push_back(target_region_has_noise_);
        while (static_cast<int>(garage_detection_history_.size()) > garage_history_size_) {
            garage_detection_history_.pop_front();
        }
        int noise_count = 0;
        for (const auto& detection : garage_detection_history_) {
            if (detection) noise_count++;
        }
        if (noise_count >= garage_confirm_threshold_) {
            obstacle_detection_.data |= (1 << 1);
            ROS_INFO_THROTTLE(1.0, "Garage obstacle CONFIRMED: %d/%d frames detected noise",
                              noise_count, garage_history_size_);
        } else {
            obstacle_detection_.data &= ~(1 << 1);
            if (noise_count > 0) {
                ROS_INFO_THROTTLE(1.0, "Garage obstacle pending: %d/%d frames (threshold: %d)",
                                  noise_count, garage_history_size_, garage_confirm_threshold_);
            }
        }
    } else {
        obstacle_detection_.data &= ~1;
        obstacle_detection_.data &= ~(1 << 1);
    }
}

// ========== YAML动态重载 ==========
void ObstacleDetection::checkAndReloadYaml(const ros::TimerEvent& event)
{
    struct stat file_stat;
    if (stat(yaml_file_path_.c_str(), &file_stat) != 0) {
        return;
    }
    if (file_stat.st_mtime != last_yaml_mod_time_) {
        std::string cmd = "rosparam load " + yaml_file_path_ + " /obstacle_detection";
        int ret = system(cmd.c_str());
        if (ret == 0) {
            ROS_INFO("YAML reload: file [%s] reloaded successfully", yaml_file_path_.c_str());
        } else {
            ROS_ERROR("YAML reload: failed to reload file [%s], system() returned %d",
                      yaml_file_path_.c_str(), ret);
        }
        last_yaml_mod_time_ = file_stat.st_mtime;
    }
}

// ========== 日志级别设置 ==========
void ObstacleDetection::setLogLevel()
{
    std::string log_level;
    private_nh_.param<std::string>("log_level", log_level, "INFO");

    ros::console::Level level;
    if (log_level == "DEBUG") {
        level = ros::console::levels::Debug;
    } else if (log_level == "INFO") {
        level = ros::console::levels::Info;
    } else if (log_level == "WARN") {
        level = ros::console::levels::Warn;
    } else if (log_level == "ERROR") {
        level = ros::console::levels::Error;
    } else if (log_level == "FATAL") {
        level = ros::console::levels::Fatal;
    } else {
        ROS_WARN("Unknown log level %s, defaulting to INFO", log_level.c_str());
        level = ros::console::levels::Info;
    }

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, level)) {
        ros::console::notifyLoggerLevelsChanged();
        ROS_INFO("Log level set to %s", log_level.c_str());
    }
}