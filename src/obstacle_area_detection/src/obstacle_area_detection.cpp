/**
 * @file obstacle_area_detection.cpp
 * @brief 库区障碍物检测节点主入口 + 构造函数 + 核心回调
 *
 * 本文件包含:
 * - main() 函数
 * - 构造函数(参数加载、订阅/发布初始化)
 * - midCloudCallback() mid360点云回调(核心处理入口)
 * - timerCallback() 定时发布检测状态
 * - checkAndReloadYaml() YAML动态重载
 * - setLogLevel() 日志级别设置
 */

#include "obstacle_area_detection/IrregularPolygonFilter.hpp"
#include "obstacle_area_detection/obstacle_area_detection.hpp"
#include <boost/bind.hpp>
#include <cstdlib>
#include <ros/package.h>

/**
 * @brief 主函数 - 节点入口
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "obstacle_area_detection");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ObstacleAreaDetection detector(nh, private_nh);
    ros::spin();
    return 0;
}

ObstacleAreaDetection::ObstacleAreaDetection(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
    : nh_(nh), private_nh_(private_nh) {
    setLogLevel();

    // ========== 加载基础参数 ==========
    private_nh_.param<std::string>("target_frame", target_frame_, "base_link");
    private_nh_.param<std::string>("points_mid_topic", points_mid_topic_, "/points_mid");
    private_nh_.param<double>("voxel_leaf_size", voxel_leaf_size_, 0.05);
    private_nh_.param<double>("z_axis_min", z_axis_min_, -1.60);
    private_nh_.param<double>("z_axis_max", z_axis_max_, 0.50);
    private_nh_.param<double>("ground_threshold", ground_threshold_, 0.15);
    private_nh_.param<double>("roi_radius", roi_radius_, 10.0);
    private_nh_.param<bool>("use_roi_filter", use_roi_filter_, true);
    private_nh_.param<double>("distance_threshold", distance_threshold_, 0.4);
    private_nh_.param<double>("area_enable_distance", area_enable_distance_, 10.0);

    // ========== 防误判参数 ==========
    private_nh_.param<int>("min_region_points", min_region_points_, 5);
    private_nh_.param<int>("area_history_size", area_history_size_, 5);
    private_nh_.param<int>("area_confirm_threshold", area_confirm_threshold_, 3);
    private_nh_.param<int>("area_clear_threshold", area_clear_threshold_, 1);

    // ========== 外围聚类检测参数(Layer 2) ==========
    private_nh_.param<bool>("enable_proximity_cluster", enable_proximity_cluster_, true);
    private_nh_.param<double>("cluster_tolerance", cluster_tolerance_, 0.2);
    private_nh_.param<int>("cluster_min_points", cluster_min_points_, 8);
    private_nh_.param<int>("cluster_max_points", cluster_max_points_, 1000);
    private_nh_.param<double>("cluster_max_z_range", cluster_max_z_range_, 5.0);
    private_nh_.param<double>("cluster_min_centroid_z", cluster_min_centroid_z_, -1.0);
    private_nh_.param<double>("expand_margin_x", expand_margin_x_, 1.0);
    private_nh_.param<double>("expand_margin_y", expand_margin_y_, 0.5);
    private_nh_.param<double>("proximity_threshold", proximity_threshold_, 0.5);
    private_nh_.param<bool>("proximity_debug", proximity_debug_, false);

    // ========== 创建订阅者(单雷达，无需时间同步) ==========
    mid_cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(points_mid_topic_, 5,
                     &ObstacleAreaDetection::midCloudCallback, this);
    keypoint_path_sub_ = nh_.subscribe<autoware_msgs::KeyPointArray>("/keypoint_path", 1,
                         &ObstacleAreaDetection::keyPointPathCallback, this);
    current_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/current_pose", 1,
                        &ObstacleAreaDetection::currentPoseCallback, this);

    // ========== 创建发布者 ==========
    ground_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/area_ground_points", 1);
    obstacle_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/area_obstacle_points", 1);
    calibration_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/area_calibration_points", 1);
    fused_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/area_preprocessed_points", 1);
    area_status_pub_ = nh_.advertise<std_msgs::UInt32>("/obstacle_area_detection", 1);
    area_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/obstacle_area_markers", 1);
    target_region_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/area_target_region_points", 1);
    proximity_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/area_proximity_cluster_points", 1);
    proximity_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/area_proximity_markers", 1);

    // ========== 创建定时器 ==========
    publish_timer_ = nh_.createTimer(ros::Duration(1.0/10), &ObstacleAreaDetection::timerCallback, this, false);

    // ========== YAML动态重载初始化 ==========
    private_nh_.param<std::string>("yaml_file_path", yaml_file_path_, "");
    if (yaml_file_path_.empty()) {
        std::string pkg_path = ros::package::getPath("obstacle_area_detection");
        yaml_file_path_ = pkg_path + "/params/obstacle_area_detection.yaml";
    }
    struct stat file_stat;
    if (stat(yaml_file_path_.c_str(), &file_stat) == 0) {
        last_yaml_mod_time_ = file_stat.st_mtime;
        ROS_INFO("YAML reload: monitoring file [%s]", yaml_file_path_.c_str());
    } else {
        last_yaml_mod_time_ = 0;
        ROS_WARN("YAML reload: cannot stat file [%s], auto-reload disabled", yaml_file_path_.c_str());
    }
    yaml_reload_timer_ = nh_.createTimer(ros::Duration(1.0), &ObstacleAreaDetection::checkAndReloadYaml, this, false);

    // ========== 初始化默认值 ==========
    current_pose_.pose.position.x = 0;
    current_pose_.pose.position.y = 0;
    current_pose_.pose.position.z = 0;
    current_pose_.pose.orientation.x = 0;
    current_pose_.pose.orientation.y = 0;
    current_pose_.pose.orientation.z = 0;
    current_pose_.pose.orientation.w = 1;
    current_pose_.header.frame_id = "map";

    ROS_INFO("Obstacle Area Detection Node Initialized");
    ROS_INFO("Subscribed to: %s", points_mid_topic_.c_str());
    ROS_INFO("Target frame: %s", target_frame_.c_str());
    ROS_INFO("Area enable distance: %.2f m", area_enable_distance_);
    ROS_INFO("Distance threshold: %.2f m", distance_threshold_);
}

// ========== 定时器回调 - 10Hz发布检测状态 ==========
void ObstacleAreaDetection::timerCallback(const ros::TimerEvent& event)
{
    // 构建位标志: 每个库位占一个bit
    std_msgs::UInt32 status_msg;
    status_msg.data = 0;
    for (size_t i = 0; i < parking_spots_.size(); i++) {
        if (i >= 32) break;  // UInt32最多32个bit
        if (parking_spots_[i].has_obstacle) {
            status_msg.data |= (1 << i);
        }
    }
    area_status_pub_.publish(status_msg);
}

// ========== mid360点云回调 - 核心处理入口 ==========
void ObstacleAreaDetection::midCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    // 没有活跃库位时直接返回
    if (parking_spots_.empty()) {
        return;
    }

    // 检查是否有任何库位启用了检测
    bool any_enabled = false;
    for (const auto& spot : parking_spots_) {
        if (spot.enable) {
            any_enabled = true;
            break;
        }
    }
    if (!any_enabled) {
        return;
    }

    // 预处理mid360点云
    PreprocessResult result;
    if (!preprocessMidCloud(msg, result) || !result.valid) {
        return;
    }

    // 动态加载库位检测区域参数
    private_nh_.getParam("/obstacle_area_detection/area_min_x", area_bounds_.min_x);
    private_nh_.getParam("/obstacle_area_detection/area_max_x", area_bounds_.max_x);
    private_nh_.getParam("/obstacle_area_detection/area_min_y", area_bounds_.min_y);
    private_nh_.getParam("/obstacle_area_detection/area_max_y", area_bounds_.max_y);
    private_nh_.getParam("/obstacle_area_detection/area_min_z", area_bounds_.min_z);
    private_nh_.getParam("/obstacle_area_detection/area_max_z", area_bounds_.max_z);

    // 发布预处理后的点云
    sensor_msgs::PointCloud2 preprocessed_msg;
    pcl::toROSMsg(*(result.filtered_cloud), preprocessed_msg);
    preprocessed_msg.header = result.header;
    fused_cloud_pub_.publish(preprocessed_msg);

    // 发布地面和障碍物点云
    publishPointClouds(result.ground_cloud, result.filtered_cloud, result.header);

    // 遍历所有活跃库位，对每个库位进行检测
    for (auto& spot : parking_spots_) {
        if (!spot.enable) {
            spot.has_obstacle = false;
            spot.raw_detection = false;
            continue;
        }

        // Layer 1: 内部区域检测
        bool detected = checkAreaForSpot(spot, result.filtered_cloud, result.header);
        spot.raw_detection = detected;

        // Layer 2: 外围聚类补充检测(仅当Layer 1未检测到时触发)
        if (!detected && enable_proximity_cluster_) {
            detected = checkProximityCluster(spot, result.filtered_cloud, result.header);
        }

        // 更新防抖状态
        updateSpotDetectionState(spot.id, detected);
    }

    // 发布所有库位的可视化标记
    publishAreaMarkers(result.header);
}

// ========== YAML动态重载 ==========
void ObstacleAreaDetection::checkAndReloadYaml(const ros::TimerEvent& event)
{
    struct stat file_stat;
    if (stat(yaml_file_path_.c_str(), &file_stat) != 0) {
        return;
    }
    if (file_stat.st_mtime != last_yaml_mod_time_) {
        std::string cmd = "rosparam load " + yaml_file_path_ + " /obstacle_area_detection";
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
void ObstacleAreaDetection::setLogLevel()
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

// ========== 生成库位唯一ID ==========
std::string ObstacleAreaDetection::generateSpotId(int index) {
    return "carports_" + std::to_string(index);
}