/**
 * @file obstacle_detection.cpp
 * @brief 障碍物检测节点主实现文件
 *
 * 实现了完整的障碍物检测流水线，包括：
 * - 双雷达点云同步接收与融合
 * - 点云预处理(标定→ROI→坐标变换→降采样→Z轴滤波→地面分割→车体过滤)
 * - 目标区域障碍物检测(电梯/库位场景)
 * - 检测结果发布与RVIZ可视化
 *
 * 检测结果通过位标志(obstacle_detection_)发布：
 *   bit0: 电梯区域是否有障碍物
 *   bit1: 库位区域是否有障碍物
 */

#include "obstacle_detection/IrregularPolygonFilter.hpp"
#include "obstacle_detection/obstacle_detection.hpp"
#include <boost/bind.hpp>
#include <cstdlib>
#include <ros/package.h>

/**
 * @brief 主函数 - 节点入口
 *
 * 初始化ROS节点，创建ObstacleDetection对象并进入spin循环
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "obstacle_detection");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ObstacleDetection detector(nh, private_nh);
    
    ros::spin();
    
    return 0;
}


/*
 * 以下为ObstacleDetection类的实现
 */


/**
 * @brief 构造函数 - 初始化整个障碍物检测节点
 *
 * 初始化流程:
 * 1. 设置日志级别
 * 2. 加载所有参数(从launch文件和yaml)
 * 3. 创建双雷达同步订阅者(使用ApproximateTime策略)
 * 4. 创建其他订阅者(导航路径、反馈状态、电梯信息等)
 * 5. 创建所有发布者
 * 6. 创建10Hz定时器(用于持续发布检测状态)
 * 7. 初始化目标点和当前位姿的默认值
 */
ObstacleDetection::ObstacleDetection(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
    : nh_(nh), private_nh_(private_nh) {
    setLogLevel();
    // 初始化参数
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
    // 方案B: 最小点数阈值 - 目标区域内点数低于此值不判定为有障碍物
    private_nh_.param<int>("min_region_points", min_region_points_, 5);
    // 方案C: 库位检测时间维度防抖参数
    private_nh_.param<int>("garage_history_size", garage_history_size_, 5);
    private_nh_.param<int>("garage_confirm_threshold", garage_confirm_threshold_, 3);
    // 方案E: 库位检测启用距离
    private_nh_.param<double>("garage_enable_distance", garage_enable_distance_, 6.0);
    

    // 订阅补盲雷达话题并同步
    mid_cloud_sub_.subscribe(nh_, points_mid_topic_, 5);
    tip_cloud_sub_.subscribe(nh_, "/fused_points_tip", 5);
    sync_.reset(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), mid_cloud_sub_, tip_cloud_sub_));
    sync_->registerCallback(boost::bind(&ObstacleDetection::syncCloudCallback, this, _1, _2));
    keyPointPath_sub_ = nh_.subscribe<autoware_msgs::KeyPointArray>("/keypoint_path", 1, &ObstacleDetection::keyPointPathCallback, this);
    feedback_status_sub_ = nh_.subscribe<autoware_remove_msgs::State>("/feedback_status", 1, &ObstacleDetection::feedbackStatusCallback, this);
    elevator_info_sub_ = nh_.subscribe<autoware_msgs::ElevatorInfo>("/elevator_info", 1, &ObstacleDetection::elevatorInfoCallback, this);
    lqr_dire_sub_ = nh_.subscribe<std_msgs::Int8>("/lqr_dire", 1, &ObstacleDetection::lqrDireCallback, this); 
    floor_set_sub_ = nh_.subscribe<std_msgs::Int8>("/floor_set", 1, &ObstacleDetection::floorSetCallback, this);
    current_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/current_pose", 1, &ObstacleDetection::currentPoseCallback, this);


    // 发布处理后的点云和障碍物信息
    ground_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/ground_points_mid", 1);
    obstacle_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/obstacle_points_mid", 1);
    cluster_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/clustered_points_mid", 1);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/obstacle_detection_markers", 1);

    calibration_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/calibration_points_mid", 1);

    // 发布目标点区域检测结果
    target_region_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/target_region_points", 1);
    obstacle_detection_pub_ = nh_.advertise<std_msgs::UInt32>("/obstacle_detection", 1);
    fused_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/points_fused_detection", 1);

    publish_timer_ = nh_.createTimer(ros::Duration(1.0/10), &ObstacleDetection::timerCallback, this, false);

    // ========== YAML动态重载初始化 ==========
    // 从launch文件获取yaml配置文件路径，为空则自动推导
    private_nh_.param<std::string>("yaml_file_path", yaml_file_path_, "");
    if (yaml_file_path_.empty()) {
        // 如果launch文件没有显式指定yaml路径，使用功能包下的默认路径
        std::string pkg_path = ros::package::getPath("obstacle_detection");
        yaml_file_path_ = pkg_path + "/params/obstacle_detection.yaml";
    }
    // 记录文件初始修改时间
    struct stat file_stat;
    if (stat(yaml_file_path_.c_str(), &file_stat) == 0) {
        last_yaml_mod_time_ = file_stat.st_mtime;
        ROS_INFO("YAML reload: monitoring file [%s]", yaml_file_path_.c_str());
    } else {
        last_yaml_mod_time_ = 0;
        ROS_WARN("YAML reload: cannot stat file [%s], auto-reload disabled", yaml_file_path_.c_str());
    }
    // 创建1Hz定时器，周期性检查yaml文件是否被修改
    yaml_reload_timer_ = nh_.createTimer(ros::Duration(1.0), &ObstacleDetection::checkAndReloadYaml, this, false);

    target_region_has_noise_ = false;
    floor_set_ = 0;
    // inplace_ = false;

    targetPoint_.map_pose.pose.position.x = 0;
    targetPoint_.map_pose.pose.position.y = 0;
    targetPoint_.map_pose.pose.position.z = 0;

    targetPoint_.map_pose.pose.orientation.x = 0;
    targetPoint_.map_pose.pose.orientation.y = 0;
    targetPoint_.map_pose.pose.orientation.z = 0;
    targetPoint_.map_pose.pose.orientation.w = 1;

    // 初始化current_pose_
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

/**
 * @brief 定时器回调 - 以10Hz频率发布障碍物检测状态
 *
 * 即使没有新的点云输入，也持续发布检测状态，确保下游系统能及时获取最新状态。
 * obstacle_detection_ 是一个位标志：
 *   bit0 (0x01): 电梯区域检测结果
 *   bit1 (0x02): 库位区域检测结果
 */
void ObstacleDetection::timerCallback(const ros::TimerEvent& event)
{
    obstacle_detection_pub_.publish(obstacle_detection_);
}

/**
 * @brief 中距雷达点云预处理流水线
 *
 * 完整处理流程:
 * 1. 从参数服务器动态加载电梯区域和车体轮廓参数(支持运行时修改)
 * 2. ROS消息 → PCL点云转换
 * 3. 雷达外参标定补偿 (points_mid_calibration)
 * 4. ROI区域滤波 (去除远处无效点)
 * 5. 坐标系变换 (源坐标系 → target_frame，通常是velodyne)
 * 6. 体素降采样 (降低点云密度，voxel_leaf_size=0时跳过)
 * 7. Z轴直通滤波 (保留z_axis_min_到z_axis_max_范围内的点)
 * 8. RANSAC地面分割 (分离地面点和障碍物点)
 * 9. 车体轮廓过滤 (去除车辆自身的激光雷达反射点)
 */
bool ObstacleDetection::preprocessMidCloud(const sensor_msgs::PointCloud2::ConstPtr& input_msg,
                                           MidProcessResult& result) {
    ros::Time start_time = ros::Time::now();

    private_nh_.getParam("/obstacle_detection/elevator_min_x", elevator_min_x_);
    private_nh_.getParam("/obstacle_detection/elevator_max_x", elevator_max_x_);
    private_nh_.getParam("/obstacle_detection/elevator_min_y", elevator_min_y_);
    private_nh_.getParam("/obstacle_detection/elevator_max_y", elevator_max_y_);
    private_nh_.getParam("/obstacle_detection/elevator_min_z", elevator_min_z_);
    private_nh_.getParam("/obstacle_detection/elevator_max_z", elevator_max_z_);

    XmlRpc::XmlRpcValue car_rect;
    private_nh_.getParam("/obstacle_detection/car_rect", car_rect);
    private_nh_.getParam("/obstacle_detection/car_min_z", car_min_z_);
    private_nh_.getParam("/obstacle_detection/car_max_z", car_max_z_);
    car_vertices_.clear();
    for (int i = 0; i < car_rect.size(); i++) {
        Eigen::Vector2f point;
        point.x() = static_cast<double>(car_rect[i]["x"]);
        point.y() = static_cast<double>(car_rect[i]["y"]);
        car_vertices_.push_back(point);
    }

    PointCloud::Ptr input_cloud(new PointCloud);
    pcl::fromROSMsg(*input_msg, *input_cloud);
    if (input_cloud->empty()) {
        ROS_WARN("Received empty mid point cloud");
        result.valid = false;
        return false;
    }

    PointCloud::Ptr calibration_cloud = points_mid_calibration(input_cloud, input_msg->header);

    std_msgs::Header header = input_msg->header;

    PointCloud::Ptr roi_cloud(new PointCloud);
    if (use_roi_filter_) {
        applyROIFilter(calibration_cloud, roi_cloud);
    } else {
        *roi_cloud = *calibration_cloud;
    }

    PointCloud::Ptr transformed_cloud(new PointCloud);
    if (target_frame_ != header.frame_id) {
        if (!transformPointCloud(roi_cloud, transformed_cloud, header.frame_id, target_frame_)) {
            ROS_WARN("Transform failed, using original frame");
            *transformed_cloud = *roi_cloud;
        }
    } else {
        *transformed_cloud = *roi_cloud;
    }

    PointCloud::Ptr downsampled_cloud(new PointCloud);
    if (voxel_leaf_size_ != 0) {
        pcl::VoxelGrid<PointT> voxel_filter;
        voxel_filter.setInputCloud(transformed_cloud);
        voxel_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
        voxel_filter.filter(*downsampled_cloud);
    } else {
        *downsampled_cloud = *transformed_cloud;
    }

    PointCloud::Ptr z_filtered_cloud(new PointCloud);
    pcl::PassThrough<PointT> pass_z;
    pass_z.setInputCloud(downsampled_cloud);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(z_axis_min_, z_axis_max_);
    pass_z.filter(*z_filtered_cloud);

    PointCloud::Ptr ground_cloud(new PointCloud);
    PointCloud::Ptr obstacle_cloud(new PointCloud);
    segmentGroundPlane(z_filtered_cloud, ground_cloud, obstacle_cloud);

    IrregularPolygonFilter car_filter;
    car_filter.setPolygonVertices(car_vertices_);
    car_filter.setZRange(car_min_z_, car_max_z_);
    PointCloud::Ptr final_cloud(new PointCloud);
    car_filter.applyFilterNegative(obstacle_cloud, final_cloud);

    result.ground_cloud = ground_cloud;
    result.filtered_cloud = final_cloud;
    result.header = input_msg->header;
    result.header.frame_id = target_frame_;
    result.valid = true;

    ros::Duration processing_time = ros::Time::now() - start_time;
    ROS_DEBUG_THROTTLE(1.0, "Mid preprocessing time: %.3f ms", processing_time.toSec() * 1000);
    return true;
}

/**
 * @brief 双雷达同步回调 - 核心处理入口
 *
 * 这是整个障碍物检测的主处理函数，由message_filters同步器触发。
 * 处理流程:
 * 1. 检查目标点是否启用（不启用则清除检测标志并返回）
 * 2. 预处理mid点云（标定→ROI→变换→降采样→地面分割→车体过滤）
 * 3. 将tip点云变换到目标坐标系
 * 4. 融合mid和tip两路点云
 * 5. 对融合后的点云进行目标区域检测
 * 6. 根据目标类型（电梯/库位）设置对应的检测标志位
 */
void ObstacleDetection::syncCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& mid_msg,
                                          const sensor_msgs::PointCloud2::ConstPtr& tip_msg) {

    // 目标点未启用时，清除所有检测标志位并直接返回
    if (!targetPoint_.enable) {
        obstacle_detection_.data &= ~1;
        obstacle_detection_.data &= ~(1 << 1);
        return;
    }

    MidProcessResult mid_result;
    if (!preprocessMidCloud(mid_msg, mid_result) || !mid_result.valid) {
        obstacle_detection_.data &= ~1;
        obstacle_detection_.data &= ~(1 << 1);
        return;
    }

    private_nh_.getParam("/obstacle_detection/carports_min_x", carports_min_x_);
    private_nh_.getParam("/obstacle_detection/carports_max_x", carports_max_x_);
    private_nh_.getParam("/obstacle_detection/carports_min_y", carports_min_y_);
    private_nh_.getParam("/obstacle_detection/carports_max_y", carports_max_y_);
    private_nh_.getParam("/obstacle_detection/carports_min_z", carports_min_z_);
    private_nh_.getParam("/obstacle_detection/carports_max_z", carports_max_z_);

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

    checkTargetPointRegion(fused_cloud, fused_msg.header);

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
        // 方案C: 时间维度防抖 - 使用滑动窗口对多帧检测结果进行投票
        // 维护最近 garage_history_size_ 帧的检测结果
        // 只有当窗口内有 >= garage_confirm_threshold_ 帧检测到噪声时才确认有障碍物
        // 这样可以过滤掉间歇性的单帧噪声误报
        garage_detection_history_.push_back(target_region_has_noise_);
        
        // 保持滑动窗口大小
        while (static_cast<int>(garage_detection_history_.size()) > garage_history_size_) {
            garage_detection_history_.pop_front();
        }
        
        // 统计窗口内检测到噪声的帧数
        int noise_count = 0;
        for (const auto& detection : garage_detection_history_) {
            if (detection) noise_count++;
        }
        
        // 只有当确认帧数 >= 阈值时才报告有障碍物
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

void ObstacleDetection::keyPointPathCallback(const autoware_msgs::KeyPointArray::ConstPtr& msg){

    if (!msg->path.empty()) {
        for (const auto& type : msg->path.back().types)
        {
            if ((type.type_name == "carports") || 
                (type.type_name == "connects" && robustElevatorCheck(type.data)))
            {
                if (type.type_name == "carports") {
                    targetPoint_.type_e = POINT_TYPE_GARAGE;
                    ROS_INFO("carports");
                }
                else if (type.type_name == "connects") {
                    targetPoint_.type_e = POINT_TYPE_ELEVATOR;
                    ROS_INFO("connects");
                }
                targetPoint_.map_pose = msg->path.back().pose;
                // 目标点切换时清除库位检测历史记录，避免旧数据影响新目标的判断
                garage_detection_history_.clear();
     
                break;
            }
        }
    }  
}
/**
 * @brief 反馈状态回调 - 库位场景启用条件(方案E: 使用可配置的启用距离)
 *
 * 当任务类型为1且距离目标 < garage_enable_distance_ 时启用库位检测
 * 缩短启用距离(从10m改为6m)可以让目标点位置更稳定，减少检测区域偏移
 */
void ObstacleDetection::feedbackStatusCallback(const autoware_remove_msgs::State::ConstPtr& msg) {
    if (targetPoint_.type_e == POINT_TYPE_GARAGE) {
        if (msg->TaskInfo.type == 1 && msg->TaskInfo.site.dis < garage_enable_distance_) {
            targetPoint_.enable = true;
        }
        else {
            targetPoint_.enable = false;
        }
    }
}

void ObstacleDetection::elevatorInfoCallback(const autoware_msgs::ElevatorInfo::ConstPtr& msg) {
    if (targetPoint_.type_e == POINT_TYPE_ELEVATOR) {
        if (msg->door_status[0] == 1 && 
            (elevator_control_flag_ == 4 || elevator_control_flag_ == -4) &&
            floor_set_ == msg->floor
        ) {
            targetPoint_.enable = true;
        }
        else {
            targetPoint_.enable = false;
        }
    }
}

void ObstacleDetection::lqrDireCallback(const std_msgs::Int8::ConstPtr& msg)
{
    elevator_control_flag_ = msg->data;
}

void ObstacleDetection::floorSetCallback(const std_msgs::Int8::ConstPtr& msg)
{
    floor_set_ = msg->data;
}

void ObstacleDetection::currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pose_ = *msg;
    
    // 如果targetPoint未启用，直接返回
    if (!targetPoint_.enable) {
        return;
    }
    
    // 计算当前位置到目标位置的距离（仅考虑x和y，忽略z）
    double dx = current_pose_.pose.position.x - targetPoint_.map_pose.pose.position.x;
    double dy = current_pose_.pose.position.y - targetPoint_.map_pose.pose.position.y;
    double distance = sqrt(dx * dx + dy * dy);
    
    // 如果距离小于阈值，关闭enable
    if (distance < distance_threshold_) {
        targetPoint_.enable = false;
        ROS_INFO("Distance to target point (%.3f m) is less than threshold (%.3f m), disabling targetPoint", 
                 distance, distance_threshold_);
    }
}

bool ObstacleDetection::transformPointCloud(const PointCloud::Ptr& input, PointCloud::Ptr& output,
                        const std::string& source_frame, const std::string& target_frame) {
    try {
        tf::StampedTransform transform;
        tf_listener_.lookupTransform(target_frame, source_frame, ros::Time(0), transform);
        
        Eigen::Affine3f eigen_transform = Eigen::Affine3f::Identity();
        
        // 获取平移
        tf::Vector3 origin = transform.getOrigin();
        eigen_transform.translation() << origin.x(), origin.y(), origin.z();
        
        // 获取旋转
        tf::Quaternion rotation = transform.getRotation();
        Eigen::Quaternionf q(rotation.w(), rotation.x(), rotation.y(), rotation.z());
        eigen_transform.rotate(q);
        pcl::transformPointCloud(*input, *output, eigen_transform);
        
        return true;
    } catch (tf::TransformException &ex) {
        ROS_WARN("TF transform from %s to %s failed: %s", 
                source_frame.c_str(), target_frame.c_str(), ex.what());
        return false;
    }
}

void ObstacleDetection::applyROIFilter(const PointCloud::Ptr& input, PointCloud::Ptr& output) {
    for (const auto& point : *input) {
        double distance = sqrt(point.x * point.x + point.y * point.y);
        if (distance <= roi_radius_) {
            output->push_back(point);
        }
    }
    output->width = output->size();
    output->height = 1;
    output->is_dense = true;
}

void ObstacleDetection::segmentGroundPlane(const PointCloud::Ptr& input,
                        PointCloud::Ptr& ground,
                        PointCloud::Ptr& obstacles) {
    // 使用RANSAC进行平面分割
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(ground_threshold_);
    seg.setMaxIterations(1000);
    seg.setInputCloud(input);
    seg.segment(*inliers, *coefficients);
    
    if (inliers->indices.size() == 0) {
        ROS_WARN("No ground plane found");
        *obstacles = *input;
        return;
    }
    
    // 提取地面点
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(input);
    extract.setIndices(inliers);
    
    extract.setNegative(false);
    extract.filter(*ground);
    
    extract.setNegative(true);
    extract.filter(*obstacles);
    
    ROS_DEBUG_THROTTLE(2.0, "Ground points: %lu, Obstacle points: %lu", 
                        ground->size(), obstacles->size());
}

void ObstacleDetection::performClustering(const PointCloud::Ptr& cloud,
                        std::vector<pcl::PointIndices>& cluster_indices) {
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);
    
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(cluster_tolerance_);
    ec.setMinClusterSize(min_cluster_size_);
    ec.setMaxClusterSize(max_cluster_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);
}

void ObstacleDetection::processClusters(const PointCloud::Ptr& cloud,
                    const std::vector<pcl::PointIndices>& cluster_indices,
                    std::vector<ObstacleInfo>& obstacles,
                    const std_msgs::Header& header) {
    obstacles.clear();
    
    for (size_t i = 0; i < cluster_indices.size(); i++) {
        // 提取聚类点云
        PointCloud cluster_cloud;
        for (const auto& idx : cluster_indices[i].indices) {
            cluster_cloud.points.push_back(cloud->points[idx]);
        }
        cluster_cloud.width = cluster_cloud.points.size();
        cluster_cloud.height = 1;
        cluster_cloud.is_dense = true;
        
        // 计算边界框
        PointT min_pt, max_pt;
        pcl::getMinMax3D(cluster_cloud, min_pt, max_pt);
        
        // 创建障碍物信息
        ObstacleInfo obstacle;
        obstacle.center.x = (min_pt.x + max_pt.x) / 2.0;
        obstacle.center.y = (min_pt.y + max_pt.y) / 2.0;
        obstacle.center.z = (min_pt.z + max_pt.z) / 2.0;
        
        obstacle.dimensions.x = max_pt.x - min_pt.x;
        obstacle.dimensions.y = max_pt.y - min_pt.y;
        obstacle.dimensions.z = max_pt.z - min_pt.z;
        
        obstacle.distance = sqrt(obstacle.center.x * obstacle.center.x + 
                                obstacle.center.y * obstacle.center.y);
        obstacle.point_count = cluster_cloud.size();
        
        obstacles.push_back(obstacle);
    }
}

void ObstacleDetection::publishPointClouds(const PointCloud::Ptr& ground,
                        const PointCloud::Ptr& obstacles,
                        const std::vector<pcl::PointIndices>& cluster_indices,
                        const std_msgs::Header& header) {
    // 发布地面点云
    if (ground->size() > 0) {
        sensor_msgs::PointCloud2 ground_msg;
        pcl::toROSMsg(*ground, ground_msg);
        ground_msg.header = header;
        ground_pub_.publish(ground_msg);
    }
    
    // 发布障碍物点云
    if (obstacles->size() > 0) {
        sensor_msgs::PointCloud2 obstacle_msg;
        pcl::toROSMsg(*obstacles, obstacle_msg);
        obstacle_msg.header = header;
        obstacle_pub_.publish(obstacle_msg);
    }
    
    // 发布彩色聚类点云
    if (!cluster_indices.empty()) {
        publishColoredClusters(obstacles, cluster_indices, header);
    }
}

void ObstacleDetection::publishColoredClusters(const PointCloud::Ptr& cloud,
                            const std::vector<pcl::PointIndices>& cluster_indices,
                            const std_msgs::Header& header) {
    pcl::PointCloud<pcl::PointXYZRGB> colored_cloud;
    
    // 预定义颜色表
    std::vector<std::tuple<uint8_t, uint8_t, uint8_t>> colors = {
        {255, 0, 0},    // 红色
        {0, 255, 0},    // 绿色
        {0, 0, 255},    // 蓝色
        {255, 255, 0},  // 黄色
        {255, 0, 255},  // 品红
        {0, 255, 255},  // 青色
        {255, 165, 0},  // 橙色
        {128, 0, 128},  // 紫色
    };
    
    for (size_t i = 0; i < cluster_indices.size(); i++) {
        auto color = colors[i % colors.size()];
        uint8_t r = std::get<0>(color);
        uint8_t g = std::get<1>(color);
        uint8_t b = std::get<2>(color);
        
        for (const auto& idx : cluster_indices[i].indices) {
            pcl::PointXYZRGB point;
            point.x = cloud->points[idx].x;
            point.y = cloud->points[idx].y;
            point.z = cloud->points[idx].z;
            point.r = r;
            point.g = g;
            point.b = b;
            colored_cloud.push_back(point);
        }
    }
    
    colored_cloud.width = colored_cloud.size();
    colored_cloud.height = 1;
    
    sensor_msgs::PointCloud2 colored_msg;
    pcl::toROSMsg(colored_cloud, colored_msg);
    colored_msg.header = header;
    cluster_pub_.publish(colored_msg);
}

void ObstacleDetection::publishObstacleInfo(const std::vector<ObstacleInfo>& obstacles,
                        const std_msgs::Header& header) {
    visualization_msgs::MarkerArray marker_array;

    
    marker_pub_.publish(marker_array);
}

void ObstacleDetection::setColorByDistance(std_msgs::ColorRGBA& color, double distance) {
    // 颜色渐变：近处红色 -> 远处绿色
    if (distance < 2.0) {
        color.r = 1.0;
        color.g = 0.0;
        color.b = 0.0;
    } else if (distance < 5.0) {
        float ratio = (distance - 2.0) / 3.0;
        color.r = 1.0 - ratio * 0.5;
        color.g = ratio;
        color.b = 0.0;
    } else {
        color.r = 0.5;
        color.g = 1.0;
        color.b = 0.0;
    }
    color.a = 0.5;  // 半透明
}

void ObstacleDetection::setLogLevel()
{
        // 获取日志级别参数
    std::string log_level;
    private_nh_.param<std::string>("log_level", log_level, "INFO");

    // 将字符串转换为ROS日志级别
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

    // 设置日志级别
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, level)) {
        ros::console::notifyLoggerLevelsChanged();
        ROS_INFO("Log level set to %s", log_level.c_str());
    }
}


PointCloud::Ptr ObstacleDetection::points_mid_calibration(PointCloud::Ptr msgPtr, std_msgs::Header header)
{   
    private_nh_.getParam("/obstacle_detection/points_mid/x", mid_x_);
    private_nh_.getParam("/obstacle_detection/points_mid/y", mid_y_);
    private_nh_.getParam("/obstacle_detection/points_mid/z", mid_z_);
    private_nh_.getParam("/obstacle_detection/points_mid/roll", mid_roll_);
    private_nh_.getParam("/obstacle_detection/points_mid/pitch", mid_pitch_);
    private_nh_.getParam("/obstacle_detection/points_mid/yaw", mid_yaw_);

    Eigen::Affine3f calibration_transform = Eigen::Affine3f::Identity();
    
    calibration_transform.translation() = Eigen::Vector3f(
        mid_x_, mid_y_, mid_z_
    );
    
    Eigen::Matrix3f rotation_matrix;
    
    float yaw = mid_yaw_;   // 绕Z轴
    float pitch = mid_pitch_; // 绕Y轴
    float roll = mid_roll_;   // 绕X轴
    
    Eigen::AngleAxisf roll_angle(roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitch_angle(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yaw_angle(yaw, Eigen::Vector3f::UnitZ());
    
    Eigen::Quaternionf q = yaw_angle * pitch_angle * roll_angle;
    calibration_transform.linear() = q.toRotationMatrix();
    

    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    
    pcl::transformPointCloud(*msgPtr, *transformed_cloud, calibration_transform);


    // 转换回ROS消息格式并发布
    sensor_msgs::PointCloud2 output_mid;
    pcl::toROSMsg(*transformed_cloud, output_mid);
    output_mid.header = header;
    calibration_pub_.publish(output_mid);
    

    return transformed_cloud;
}



void ObstacleDetection::checkTargetPointRegion(const PointCloud::Ptr& obstacle_cloud,
                                             const std_msgs::Header& header) {

    // 转换目标点到当前坐标系（如果需要）
    geometry_msgs::Pose transformed_target_pose = transformTargetPoseToVelodyne(targetPoint_.map_pose);
    
    // 提取目标点的位置和方向
    Eigen::Vector3f target_position(
        transformed_target_pose.position.x,
        transformed_target_pose.position.y,
        transformed_target_pose.position.z
    );
    
    Eigen::Quaternionf target_orientation(
        transformed_target_pose.orientation.w,
        transformed_target_pose.orientation.x,
        transformed_target_pose.orientation.y,
        transformed_target_pose.orientation.z
    );
    
    // 归一化四元数（确保它是单位四元数）
    target_orientation.normalize();
    
    // 创建旋转矩阵
    Eigen::Matrix3f rotation_matrix = target_orientation.toRotationMatrix();
    Eigen::Matrix3f inverse_rotation_matrix = rotation_matrix.inverse();  // 存储逆矩阵避免重复计算
    
    // 确保只在正确的类型下进行检测
    if (targetPoint_.type_e != POINT_TYPE_ELEVATOR && targetPoint_.type_e != POINT_TYPE_GARAGE) {
        ROS_DEBUG_THROTTLE(1.0, "Target point type is not ELEVATOR or GARAGE, skipping region check");
        target_region_has_noise_ = false;
        return;
    }
    
    // 获取边界参数（必须先定义这些变量）
    double min_x, max_x, min_y, max_y, min_z, max_z;
    if (targetPoint_.type_e == POINT_TYPE_ELEVATOR) {
        min_x = elevator_min_x_; max_x = elevator_max_x_;
        min_y = elevator_min_y_; max_y = elevator_max_y_;
        min_z = elevator_min_z_; max_z = elevator_max_z_;
    } else {
        min_x = carports_min_x_; max_x = carports_max_x_;
        min_y = carports_min_y_; max_y = carports_max_y_;
        min_z = carports_min_z_; max_z = carports_max_z_;
    }
    
    ROS_INFO_THROTTLE(2.0, "Target position in %s: (%.6f, %.6f, %.6f), orientation: (%.6f, %.6f, %.6f, %.6f), bounds: x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f]",
                       target_frame_.c_str(),
                       target_position.x(), target_position.y(), target_position.z(),
                       target_orientation.w(), target_orientation.x(), target_orientation.y(), target_orientation.z(),
                       min_x, max_x, min_y, max_y, min_z, max_z);
    
    // 收集区域内的点
    PointCloud::Ptr region_points(new PointCloud);
    bool has_noise = false;
    
    // 统计信息用于调试
    size_t total_points = obstacle_cloud->size();
    size_t points_in_region = 0;
    size_t points_outside_region = 0;
    
    // 方案B: 记录区域内的点数，用于后续与最小点数阈值比较
    // 不再在发现单个点时就立即设置has_noise，而是等统计完所有点后再判断
    
    // 输出目标点信息用于调试（检查目标点位置是否稳定）
    static Eigen::Vector3f last_target_position(0, 0, 0);
    static int position_change_count = 0;
    Eigen::Vector3f current_target_position(target_position.x(), target_position.y(), target_position.z());
    if ((current_target_position - last_target_position).norm() > 0.01) {
        position_change_count++;
        ROS_WARN_THROTTLE(1.0, "Target position changed! Count: %d, Map: (%.3f, %.3f, %.3f), %s: (%.3f, %.3f, %.3f), Change: (%.3f, %.3f, %.3f)",
                          position_change_count,
                          targetPoint_.map_pose.pose.position.x, 
                          targetPoint_.map_pose.pose.position.y,
                          targetPoint_.map_pose.pose.position.z,
                          target_frame_.c_str(),
                          target_position.x(), target_position.y(), target_position.z(),
                          (current_target_position - last_target_position).x(),
                          (current_target_position - last_target_position).y(),
                          (current_target_position - last_target_position).z());
        last_target_position = current_target_position;
    }
    
    for (const auto& point : *obstacle_cloud) {
        // 将点转换到目标点的局部坐标系
        // 注意：这里是将全局坐标系的点转换到目标点的局部坐标系
        // 转换公式：p_local = R^(-1) * (p_global - t_target)
        // 其中 R 是目标点的旋转矩阵，t_target 是目标点的位置
        Eigen::Vector3f p(point.x, point.y, point.z);
        Eigen::Vector3f p_local = inverse_rotation_matrix * (p - target_position);
        
        bool point_in_region = false;
        
        // 边界检查：包含边界
        bool x_in_range = (p_local.x() >= min_x) && (p_local.x() <= max_x);
        bool y_in_range = (p_local.y() >= min_y) && (p_local.y() <= max_y);
        bool z_in_range = (p_local.z() >= min_z) && (p_local.z() <= max_z);
        
        if (targetPoint_.type_e == POINT_TYPE_ELEVATOR)
        {
            // 检查是否在区域内（包含边界）
            // 注意：不在这里设置has_noise，而是在统计完所有点后根据min_region_points_阈值判断
            if (x_in_range && y_in_range && z_in_range) {
                point_in_region = true;
                obstacle_detection_.data &= ~1;
            }
        }
        else if (targetPoint_.type_e == POINT_TYPE_GARAGE) {
            // 检查是否在区域内（包含边界）
            // 注意：不在这里设置has_noise，而是在统计完所有点后根据min_region_points_阈值判断
            if (x_in_range && y_in_range && z_in_range) {
                point_in_region = true;
            }
        }
        
        // 只有确认在区域内的点才添加到 region_points
        if (point_in_region) {
            region_points->push_back(point);
            points_in_region++;
        } else {
            points_outside_region++;
        }
    }
    
    // 方案B: 使用最小点数阈值判断是否有障碍物
    // 只有区域内的点数 >= min_region_points_ 时才判定为有障碍物
    if (static_cast<int>(points_in_region) >= min_region_points_) {
        has_noise = true;
    } else {
        has_noise = false;
        ROS_INFO_THROTTLE(1.0, "Region points (%lu) below threshold (%d), not considered noise", 
                          points_in_region, min_region_points_);
    }
    
    // 调试信息：如果发现有框外的点被添加，输出详细信息
    if (points_in_region > 0 && points_in_region != region_points->size()) {
        ROS_WARN("Mismatch: points_in_region=%lu, region_points->size()=%lu", 
                 points_in_region, region_points->size());
    }
    
    // 发布区域内的点云(仅当点数满足阈值时才发布)
    if (static_cast<int>(region_points->size()) >= min_region_points_) {
        // 双重验证：再次检查所有点是否真的在区域内（使用完全相同的逻辑）
        PointCloud::Ptr verified_points(new PointCloud);
        verified_points->reserve(region_points->size());
        
        size_t filtered_out = 0;
        for (const auto& point : *region_points) {
            // 重新计算局部坐标（确保使用相同的转换）
            Eigen::Vector3f p(point.x, point.y, point.z);
            Eigen::Vector3f p_local = inverse_rotation_matrix * (p - target_position);
            
            // 严格边界检查（与第一次检查完全相同的逻辑）
            bool x_in_range = (p_local.x() >= min_x) && (p_local.x() <= max_x);
            bool y_in_range = (p_local.y() >= min_y) && (p_local.y() <= max_y);
            bool z_in_range = (p_local.z() >= min_z) && (p_local.z() <= max_z);
            
            if (x_in_range && y_in_range && z_in_range) {
                verified_points->push_back(point);
            } else {
                filtered_out++;
                // 输出详细信息用于调试
                ROS_WARN_THROTTLE(1.0, "Filtered point outside region! Local: (%.4f, %.4f, %.4f), Global: (%.4f, %.4f, %.4f), bounds: x[%.4f,%.4f] y[%.4f,%.4f] z[%.4f,%.4f]",
                                   p_local.x(), p_local.y(), p_local.z(),
                                   point.x, point.y, point.z,
                                   min_x, max_x, min_y, max_y, min_z, max_z);
            }
        }
        
        if (filtered_out > 0) {
            ROS_WARN("Verification filtered out %lu/%lu points that were outside region", 
                     filtered_out, region_points->size());
        }
        
        // 只发布验证通过的点
        if (verified_points->size() > 0) {
            // 第三次验证：在发布前最后一次检查，并输出详细信息
            PointCloud::Ptr final_points(new PointCloud);
            final_points->reserve(verified_points->size());
            
            size_t final_filtered = 0;
            for (const auto& point : *verified_points) {
                Eigen::Vector3f p(point.x, point.y, point.z);
                Eigen::Vector3f p_local = inverse_rotation_matrix * (p - target_position);
                
                // 严格边界检查
                bool x_in = (p_local.x() >= min_x) && (p_local.x() <= max_x);
                bool y_in = (p_local.y() >= min_y) && (p_local.y() <= max_y);
                bool z_in = (p_local.z() >= min_z) && (p_local.z() <= max_z);
                
                if (x_in && y_in && z_in) {
                    final_points->push_back(point);
                } else {
                    final_filtered++;
                    ROS_ERROR("CRITICAL: Point outside bounds in final check! Local: (%.6f, %.6f, %.6f), bounds: x[%.6f,%.6f] y[%.6f,%.6f] z[%.6f,%.6f]",
                              p_local.x(), p_local.y(), p_local.z(),
                              min_x, max_x, min_y, max_y, min_z, max_z);
                }
            }
            
            if (final_filtered > 0) {
                ROS_ERROR("Final check filtered out %lu points! This should not happen!", final_filtered);
            }
            
            // 输出所有点的局部坐标用于验证（限制输出数量避免日志过多）
            if (final_points->size() <= 10) {
                ROS_INFO("All %lu final points in local coordinates:", final_points->size());
                for (size_t i = 0; i < final_points->size(); ++i) {
                    const auto& point = final_points->points[i];
                    Eigen::Vector3f p(point.x, point.y, point.z);
                    Eigen::Vector3f p_local = inverse_rotation_matrix * (p - target_position);
                    bool x_ok = (p_local.x() >= min_x) && (p_local.x() <= max_x);
                    bool y_ok = (p_local.y() >= min_y) && (p_local.y() <= max_y);
                    bool z_ok = (p_local.z() >= min_z) && (p_local.z() <= max_z);
                    ROS_INFO("  Point %lu: Global(%.6f, %.6f, %.6f) -> Local(%.6f, %.6f, %.6f) [x:%s y:%s z:%s]",
                             i, point.x, point.y, point.z, 
                             p_local.x(), p_local.y(), p_local.z(),
                             x_ok ? "OK" : "OUT", y_ok ? "OK" : "OUT", z_ok ? "OK" : "OUT");
                }
            }
            
            if (final_points->size() > 0) {
                // 输出边界框的8个顶点在全局坐标系中的位置（用于验证）
                ROS_INFO_THROTTLE(2.0, "Boundary box vertices in global coordinates (for verification):");
                Eigen::Matrix3f rotation_matrix = target_orientation.toRotationMatrix();
                std::vector<Eigen::Vector3f> local_vertices = {
                    Eigen::Vector3f(min_x, min_y, min_z),
                    Eigen::Vector3f(max_x, min_y, min_z),
                    Eigen::Vector3f(max_x, max_y, min_z),
                    Eigen::Vector3f(min_x, max_y, min_z),
                    Eigen::Vector3f(min_x, min_y, max_z),
                    Eigen::Vector3f(max_x, min_y, max_z),
                    Eigen::Vector3f(max_x, max_y, max_z),
                    Eigen::Vector3f(min_x, max_y, max_z)
                };
                for (size_t i = 0; i < local_vertices.size(); ++i) {
                    Eigen::Vector3f global_vertex = rotation_matrix * local_vertices[i] + target_position;
                    ROS_INFO_THROTTLE(2.0, "  Vertex %lu: Local(%.3f, %.3f, %.3f) -> Global(%.3f, %.3f, %.3f)",
                                      i, local_vertices[i].x(), local_vertices[i].y(), local_vertices[i].z(),
                                      global_vertex.x(), global_vertex.y(), global_vertex.z());
                }
                
                sensor_msgs::PointCloud2 region_msg;
                pcl::toROSMsg(*final_points, region_msg);
                region_msg.header = header;
                target_region_cloud_pub_.publish(region_msg);
            } else {
                ROS_ERROR("All points filtered out in final check, not publishing!");
            }
        } else {
            ROS_WARN("All %lu points were filtered out during verification, not publishing", region_points->size());
        }
        
        if (targetPoint_.type_e == POINT_TYPE_ELEVATOR) {
            ROS_INFO("Total: %lu, In region: %lu, Outside: %lu, Verified: %lu. Bounds: x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f]. Has noise: %s", 
                     total_points, points_in_region, points_outside_region, verified_points->size(),
                     elevator_min_x_, elevator_max_x_,
                     elevator_min_y_, elevator_max_y_,
                     elevator_min_z_, elevator_max_z_,
                     has_noise ? "YES" : "NO");
        } else if (targetPoint_.type_e == POINT_TYPE_GARAGE) {
            ROS_INFO("Total: %lu, In region: %lu, Outside: %lu, Verified: %lu. Bounds: x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f]. Has noise: %s", 
                     total_points, points_in_region, points_outside_region, verified_points->size(),
                     carports_min_x_, carports_max_x_,
                     carports_min_y_, carports_max_y_,
                     carports_min_z_, carports_max_z_,
                     has_noise ? "YES" : "NO");
        }
    } else {
        ROS_INFO_THROTTLE(1.0, "No points found in target region (checked %lu total points)", total_points);
    }
    
    // 更新区域状态
    target_region_has_noise_ = has_noise;
    publishTargetRegionMarker(header);
}


void ObstacleDetection::publishTargetRegionMarker(const std_msgs::Header& header) {
    visualization_msgs::MarkerArray marker_array;
    
    double temp_min_x, temp_max_x;
    double temp_min_y, temp_max_y;
    double temp_min_z, temp_max_z;
    double l_r, l_g, l_b, l_a;
    if (targetPoint_.type_e == POINT_TYPE_ELEVATOR)
    {
        temp_min_x = elevator_min_x_; temp_max_x = elevator_max_x_;
        temp_min_y = elevator_min_y_; temp_max_y = elevator_max_y_;
        temp_min_z = elevator_min_z_; temp_max_z = elevator_max_z_;
        l_r = 1.0; l_g = 1.0; l_b = 1.0; l_a = 1.0; // 白色
    }
    else if (targetPoint_.type_e == POINT_TYPE_GARAGE) {
        temp_min_x = carports_min_x_; temp_max_x = carports_max_x_;
        temp_min_y = carports_min_y_; temp_max_y = carports_max_y_;
        temp_min_z = carports_min_z_; temp_max_z = carports_max_z_;
        l_r = 0.0; l_g = 1.0; l_b = 1.0; l_a = 1.0; // 青色
    }

    // 创建目标点区域立体框（线框）
    visualization_msgs::Marker region_marker;
    region_marker.header = header;
    region_marker.header.frame_id = target_frame_;
    region_marker.ns = "target_region";
    region_marker.id = 0;
    region_marker.type = visualization_msgs::Marker::LINE_LIST;
    region_marker.action = visualization_msgs::Marker::ADD;
    
    // 设置标记位置和方向（使用targetPoint的位姿）
    region_marker.pose = transformTargetPoseToVelodyne(targetPoint_.map_pose);
    
    // 设置线宽
    region_marker.scale.x = 0.05;  // 线宽
    region_marker.scale.y = 0.0;
    region_marker.scale.z = 0.0;
    
    // 根据是否有噪点设置颜色
    if (target_region_has_noise_) {
        // 有噪点：红色
        region_marker.color.r = 1.0;
        region_marker.color.g = 0.0;
        region_marker.color.b = 0.0;
        region_marker.color.a = 1.0;
    } else {
        // 无噪点：绿色
        region_marker.color.r = l_r;
        region_marker.color.g = l_g;
        region_marker.color.b = l_b;
        region_marker.color.a = l_a;
    }
    
    region_marker.lifetime = ros::Duration(1.0);
    
    // 计算立体框中心相对于目标点的偏移
    // 因为我们的检测区域不是关于原点对称的，所以需要计算中心位置
    double center_x = (temp_max_x + temp_min_x) / 2.0;
    double center_y = (temp_max_y + temp_min_y) / 2.0;
    double center_z = (temp_max_z + temp_min_z) / 2.0;
    
    // 计算半宽、半高、半深
    double half_x = (temp_max_x - temp_min_x) / 2.0;
    double half_y = (temp_max_y - temp_min_y) / 2.0;
    double half_z = (temp_max_z - temp_min_z) / 2.0;
    
    // 定义8个顶点（在局部坐标系中，相对于目标点）
    // 注意：这些顶点坐标必须与checkTargetPointRegion中的检查逻辑一致
    // 在checkTargetPointRegion中，点被转换到目标点的局部坐标系（以目标点为原点）
    // 所以这里的顶点也应该相对于目标点（不是边界框中心）
    std::vector<geometry_msgs::Point> vertices(8);
    
    // 底面四个顶点（相对于目标点的局部坐标）
    vertices[0].x = temp_min_x; vertices[0].y = temp_min_y; vertices[0].z = temp_min_z;  // 前左下
    vertices[1].x = temp_max_x; vertices[1].y = temp_min_y; vertices[1].z = temp_min_z;  // 前右下
    vertices[2].x = temp_max_x; vertices[2].y = temp_max_y; vertices[2].z = temp_min_z;  // 后右下
    vertices[3].x = temp_min_x; vertices[3].y = temp_max_y; vertices[3].z = temp_min_z;  // 后左下
    
    // 顶面四个顶点（相对于目标点的局部坐标）
    vertices[4].x = temp_min_x; vertices[4].y = temp_min_y; vertices[4].z = temp_max_z;  // 前左上
    vertices[5].x = temp_max_x; vertices[5].y = temp_min_y; vertices[5].z = temp_max_z;  // 前右上
    vertices[6].x = temp_max_x; vertices[6].y = temp_max_y; vertices[6].z = temp_max_z;  // 后右上
    vertices[7].x = temp_min_x; vertices[7].y = temp_max_y; vertices[7].z = temp_max_z;  // 后左上
    
    // 重要：marker的pose已经设置为targetPoint的位置和方向
    // 顶点坐标是相对于这个pose的局部坐标，不需要调整marker位置
    // 这样边界框的可视化就与点云检查的坐标系完全一致了
    
    // 定义12条边（每条边由两个顶点组成）
    // 底面
    region_marker.points.push_back(vertices[0]); region_marker.points.push_back(vertices[1]);  // 前边
    region_marker.points.push_back(vertices[1]); region_marker.points.push_back(vertices[2]);  // 右边
    region_marker.points.push_back(vertices[2]); region_marker.points.push_back(vertices[3]);  // 后边
    region_marker.points.push_back(vertices[3]); region_marker.points.push_back(vertices[0]);  // 左边
    
    // 顶面
    region_marker.points.push_back(vertices[4]); region_marker.points.push_back(vertices[5]);  // 前边
    region_marker.points.push_back(vertices[5]); region_marker.points.push_back(vertices[6]);  // 右边
    region_marker.points.push_back(vertices[6]); region_marker.points.push_back(vertices[7]);  // 后边
    region_marker.points.push_back(vertices[7]); region_marker.points.push_back(vertices[4]);  // 左边
    
    // 侧面
    region_marker.points.push_back(vertices[0]); region_marker.points.push_back(vertices[4]);  // 左前竖边
    region_marker.points.push_back(vertices[1]); region_marker.points.push_back(vertices[5]);  // 右前竖边
    region_marker.points.push_back(vertices[2]); region_marker.points.push_back(vertices[6]);  // 右后竖边
    region_marker.points.push_back(vertices[3]); region_marker.points.push_back(vertices[7]);  // 左后竖边
    
    // 将标记添加到数组
    marker_array.markers.push_back(region_marker);
    



    // if (!car_vertices_.empty()) {
    //     visualization_msgs::Marker car_marker;
    //     car_marker.header = header;
    //     car_marker.header.frame_id = target_frame_;  // 通常为 base_link
    //     car_marker.ns = "car_body";
    //     car_marker.id = 100;  // 使用较大ID避免与其他标记冲突
    //     car_marker.type = visualization_msgs::Marker::LINE_LIST;
    //     car_marker.action = visualization_msgs::Marker::ADD;
        
    //     // 车体在base_link坐标系中，位置在原点
    //     car_marker.pose.position.x = 0;
    //     car_marker.pose.position.y = 0;
    //     car_marker.pose.position.z = 0;
    //     car_marker.pose.orientation.x = 0;
    //     car_marker.pose.orientation.y = 0;
    //     car_marker.pose.orientation.z = 0;
    //     car_marker.pose.orientation.w = 1;
        
    //     // 设置线宽和颜色
    //     car_marker.scale.x = 0.03;  // 线宽
    //     car_marker.color.r = 1.0;   // 黄色
    //     car_marker.color.g = 1.0;
    //     car_marker.color.b = 0.0;
    //     car_marker.color.a = 0.8;   // 半透明
        
    //     car_marker.lifetime = ros::Duration(1.0);
        
    //     // 将多边形顶点转换为3D点（底面）
    //     std::vector<geometry_msgs::Point> bottom_vertices;
    //     for (const auto& vertex : car_vertices_) {
    //         geometry_msgs::Point p;
    //         p.x = vertex.x();
    //         p.y = vertex.y();
    //         p.z = car_min_z_;  // 底面
    //         bottom_vertices.push_back(p);
    //     }
        
    //     // 创建顶面顶点（Z轴为car_max_z_）
    //     std::vector<geometry_msgs::Point> top_vertices;
    //     for (const auto& vertex : car_vertices_) {
    //         geometry_msgs::Point p;
    //         p.x = vertex.x();
    //         p.y = vertex.y();
    //         p.z = car_max_z_;  // 顶面
    //         top_vertices.push_back(p);
    //     }
        
    //     // 绘制底面多边形
    //     size_t n = bottom_vertices.size();
    //     for (size_t i = 0; i < n; i++) {
    //         car_marker.points.push_back(bottom_vertices[i]);
    //         car_marker.points.push_back(bottom_vertices[(i + 1) % n]);
    //     }
        
    //     // 绘制顶面多边形
    //     for (size_t i = 0; i < n; i++) {
    //         car_marker.points.push_back(top_vertices[i]);
    //         car_marker.points.push_back(top_vertices[(i + 1) % n]);
    //     }
        
    //     // 绘制侧面连接线
    //     for (size_t i = 0; i < n; i++) {
    //         car_marker.points.push_back(bottom_vertices[i]);
    //         car_marker.points.push_back(top_vertices[i]);
    //     }
        
    //     marker_array.markers.push_back(car_marker);
    // }



    // 发布标记
    marker_pub_.publish(marker_array);
}


// 解析电梯信息
bool ObstacleDetection::robustElevatorCheck(const std::string& data) {
    ROS_INFO("robustElevatorCheck with data: %s", data.c_str());

    // 检查数据是否为空
    if (data.empty()) {
        ROS_WARN("Empty data received");
        return false;
    }

    // 查找最外层的单引号对
    size_t first_quote = data.find('\'');
    if (first_quote == std::string::npos) {
        ROS_WARN("No opening quote found");
        return false;
    }
    
    // 查找第二个单引号（结束引号）
    size_t second_quote = data.find('\'', first_quote + 1);
    if (second_quote == std::string::npos) {
        ROS_WARN("No closing quote found");
        return false;
    }
    
    // 查找可能的第三个单引号（如果有嵌套引号）
    size_t third_quote = data.find('\'', second_quote + 1);
    size_t fourth_quote = std::string::npos;
    
    // 如果有四个单引号，说明是嵌套情况：''A1,1''
    if (third_quote != std::string::npos) {
        fourth_quote = data.find('\'', third_quote + 1);
    }
    
    std::string quoted_content;
    
    if (fourth_quote != std::string::npos) {
        // 处理嵌套引号情况：''A1,1''
        quoted_content = data.substr(second_quote + 1, third_quote - second_quote - 1);
    } else {
        // 处理普通引号情况：'A1,1'
        quoted_content = data.substr(first_quote + 1, second_quote - first_quote - 1);
    }
    
    ROS_INFO("Quoted content: %s", quoted_content.c_str());
    
    // 分割引号内容
    size_t comma_in_quoted = quoted_content.find(',');
    if (comma_in_quoted == std::string::npos) {
        ROS_WARN("No comma found in quoted content");
        return false;
    }
    
    std::string second_in_quotes = quoted_content.substr(comma_in_quoted + 1);
    
    // 找最后一个逗号
    size_t last_comma = data.find_last_of(',');
    if (last_comma == std::string::npos) {
        ROS_WARN("No comma found in data");
        return false;
    }
    
    std::string last_value = data.substr(last_comma + 1);
    
    ROS_INFO("Second in quotes: %s, Last value: %s", 
             second_in_quotes.c_str(), last_value.c_str());
    
    // 条件检查
    bool result = (second_in_quotes == "1" && last_value == "0");
    ROS_INFO("Check result: %s", result ? "true" : "false");
    
    return result;
}


/**
 * @brief yaml文件变更检查回调 - 以1Hz频率检查配置文件是否被修改
 *
 * 工作原理:
 * 1. 使用stat()获取yaml文件的st_mtime(最后修改时间)
 * 2. 与上次记录的last_yaml_mod_time_比较
 * 3. 如果时间戳不同，说明文件被修改，执行重载:
 *    a. 调用system("rosparam load ...")将新配置加载到参数服务器
 *    b. 更新last_yaml_mod_time_为最新时间戳
 *    c. 输出日志通知
 * 4. 下一帧的getParam()调用会自动获取参数服务器上的新值
 *
 * 注意: 此函数使用system()调用rosparam命令，这是ROS1中最可靠的
 * yaml重载方式。替代方案是使用yaml-cpp直接解析文件，但需要
 * 手动处理参数服务器的更新逻辑，更复杂。
 */
void ObstacleDetection::checkAndReloadYaml(const ros::TimerEvent& event)
{
    struct stat file_stat;
    if (stat(yaml_file_path_.c_str(), &file_stat) != 0) {
        // 文件不存在或无法访问，跳过检查
        return;
    }

    // 比较文件修改时间
    if (file_stat.st_mtime != last_yaml_mod_time_) {
        // 文件被修改，重新加载到参数服务器
        std::string cmd = "rosparam load " + yaml_file_path_ + " /obstacle_detection";
        int ret = system(cmd.c_str());
        if (ret == 0) {
            ROS_INFO("YAML reload: file [%s] reloaded successfully", yaml_file_path_.c_str());
        } else {
            ROS_ERROR("YAML reload: failed to reload file [%s], system() returned %d", 
                      yaml_file_path_.c_str(), ret);
        }
        // 更新时间戳(无论成功与否，避免反复重试失败的加载)
        last_yaml_mod_time_ = file_stat.st_mtime;
    }
}

geometry_msgs::Pose ObstacleDetection::transformTargetPoseToVelodyne(const geometry_msgs::PoseStamped& input_pose)
{
    // 将目标点从map坐标系转换到target_frame_坐标系（通常是velodyne或base_link）
    try {
        // 获取当前时间
        ros::Time now = ros::Time::now();
        
        // 等待坐标变换可用
        if (tf_listener_.waitForTransform(target_frame_, "map", now, ros::Duration(1.0))) {
            geometry_msgs::PoseStamped pose_in_map = input_pose;
            pose_in_map.header.stamp = now;
            pose_in_map.header.frame_id = "map";
            // 转换坐标系到target_frame_
            geometry_msgs::PoseStamped pose_in_target_frame;
            tf_listener_.transformPose(target_frame_, pose_in_map, pose_in_target_frame);
            
            // 存储转换后的目标点
            return pose_in_target_frame.pose;
        } else {
            ROS_WARN("TF transform from map to %s not available, using original pose", target_frame_.c_str());
        }
    } catch (tf::TransformException &ex) {
        ROS_WARN("Failed to transform target point from map to %s: %s", target_frame_.c_str(), ex.what());
    }
    return input_pose.pose;
}

