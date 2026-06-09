/**
 * @file pointcloud_preprocessing.cpp
 * @brief 点云预处理模块
 *
 * 本文件包含点云预处理流水线的实现:
 * - preprocessMidCloud()      mid360预处理(标定→ROI→变换→降采样→Z滤波→地面分割→车体过滤)
 * - pointsMidCalibration()    6DOF外参标定补偿
 * - applyROIFilter()          ROI半径滤波
 * - transformPointCloud()     TF坐标变换
 * - segmentGroundPlane()      RANSAC地面分割
 */

#include "obstacle_area_detection/IrregularPolygonFilter.hpp"
#include "obstacle_area_detection/obstacle_area_detection.hpp"

/**
 * @brief mid360点云预处理流水线
 *
 * 处理流程:
 * 1. 从参数服务器动态加载检测区域和车体轮廓参数(支持运行时修改)
 * 2. ROS消息 → PCL点云转换
 * 3. 雷达外参标定补偿 (pointsMidCalibration)
 * 4. ROI区域滤波 (去除远处无效点)
 * 5. 坐标系变换 (源坐标系 → target_frame，通常是velodyne)
 * 6. 体素降采样 (降低点云密度，voxel_leaf_size=0时跳过)
 * 7. Z轴直通滤波 (保留z_axis_min_到z_axis_max_范围内的点)
 * 8. RANSAC地面分割 (分离地面点和障碍物点)
 * 9. 车体轮廓过滤 (去除车辆自身的激光雷达反射点)
 */
bool ObstacleAreaDetection::preprocessMidCloud(const sensor_msgs::PointCloud2::ConstPtr& input_msg,
                                           PreprocessResult& result) {
    ros::Time start_time = ros::Time::now();

    // 动态加载参数(支持运行时修改)
    private_nh_.getParam("/obstacle_area_detection/area_min_x", area_bounds_.min_x);
    private_nh_.getParam("/obstacle_area_detection/area_max_x", area_bounds_.max_x);
    private_nh_.getParam("/obstacle_area_detection/area_min_y", area_bounds_.min_y);
    private_nh_.getParam("/obstacle_area_detection/area_max_y", area_bounds_.max_y);
    private_nh_.getParam("/obstacle_area_detection/area_min_z", area_bounds_.min_z);
    private_nh_.getParam("/obstacle_area_detection/area_max_z", area_bounds_.max_z);

    XmlRpc::XmlRpcValue car_rect;
    private_nh_.getParam("/obstacle_area_detection/car_rect", car_rect);
    private_nh_.getParam("/obstacle_area_detection/car_min_z", car_min_z_);
    private_nh_.getParam("/obstacle_area_detection/car_max_z", car_max_z_);
    car_vertices_.clear();
    for (int i = 0; i < car_rect.size(); i++) {
        Eigen::Vector2f point;
        point.x() = static_cast<double>(car_rect[i]["x"]);
        point.y() = static_cast<double>(car_rect[i]["y"]);
        car_vertices_.push_back(point);
    }

    // 步骤1: ROS消息 → PCL点云
    PointCloud::Ptr input_cloud(new PointCloud);
    pcl::fromROSMsg(*input_msg, *input_cloud);
    if (input_cloud->empty()) {
        ROS_WARN("Received empty mid point cloud");
        result.valid = false;
        return false;
    }

    // 步骤2: 6DOF外参标定补偿
    PointCloud::Ptr calibration_cloud = pointsMidCalibration(input_cloud, input_msg->header);

    std_msgs::Header header = input_msg->header;

    // 步骤3: ROI区域滤波
    PointCloud::Ptr roi_cloud(new PointCloud);
    if (use_roi_filter_) {
        applyROIFilter(calibration_cloud, roi_cloud);
    } else {
        *roi_cloud = *calibration_cloud;
    }

    // 步骤4: 坐标系变换
    PointCloud::Ptr transformed_cloud(new PointCloud);
    if (target_frame_ != header.frame_id) {
        if (!transformPointCloud(roi_cloud, transformed_cloud, header.frame_id, target_frame_)) {
            ROS_WARN("Transform failed, using original frame");
            *transformed_cloud = *roi_cloud;
        }
    } else {
        *transformed_cloud = *roi_cloud;
    }

    // 步骤5: 体素降采样
    PointCloud::Ptr downsampled_cloud(new PointCloud);
    if (voxel_leaf_size_ != 0) {
        pcl::VoxelGrid<PointT> voxel_filter;
        voxel_filter.setInputCloud(transformed_cloud);
        voxel_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
        voxel_filter.filter(*downsampled_cloud);
    } else {
        *downsampled_cloud = *transformed_cloud;
    }

    // 步骤6: Z轴直通滤波
    PointCloud::Ptr z_filtered_cloud(new PointCloud);
    pcl::PassThrough<PointT> pass_z;
    pass_z.setInputCloud(downsampled_cloud);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(z_axis_min_, z_axis_max_);
    pass_z.filter(*z_filtered_cloud);

    // 步骤7: RANSAC地面分割
    PointCloud::Ptr ground_cloud(new PointCloud);
    PointCloud::Ptr obstacle_cloud(new PointCloud);
    segmentGroundPlane(z_filtered_cloud, ground_cloud, obstacle_cloud);

    // 步骤8: 车体轮廓过滤
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
 * @brief 中距雷达标定变换
 *
 * 根据yaml配置的6DOF参数(平移+旋转)对中距雷达进行外参标定补偿
 */
PointCloud::Ptr ObstacleAreaDetection::pointsMidCalibration(PointCloud::Ptr msgPtr, std_msgs::Header header)
{
    private_nh_.getParam("/obstacle_area_detection/points_mid/x", mid_x_);
    private_nh_.getParam("/obstacle_area_detection/points_mid/y", mid_y_);
    private_nh_.getParam("/obstacle_area_detection/points_mid/z", mid_z_);
    private_nh_.getParam("/obstacle_area_detection/points_mid/roll", mid_roll_);
    private_nh_.getParam("/obstacle_area_detection/points_mid/pitch", mid_pitch_);
    private_nh_.getParam("/obstacle_area_detection/points_mid/yaw", mid_yaw_);

    Eigen::Affine3f calibration_transform = Eigen::Affine3f::Identity();
    calibration_transform.translation() = Eigen::Vector3f(mid_x_, mid_y_, mid_z_);

    float yaw = mid_yaw_;
    float pitch = mid_pitch_;
    float roll = mid_roll_;

    Eigen::AngleAxisf roll_angle(roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitch_angle(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yaw_angle(yaw, Eigen::Vector3f::UnitZ());

    Eigen::Quaternionf q = yaw_angle * pitch_angle * roll_angle;
    calibration_transform.linear() = q.toRotationMatrix();

    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*msgPtr, *transformed_cloud, calibration_transform);

    // 发布标定后点云供调试
    sensor_msgs::PointCloud2 output_mid;
    pcl::toROSMsg(*transformed_cloud, output_mid);
    output_mid.header = header;
    calibration_pub_.publish(output_mid);

    return transformed_cloud;
}

/**
 * @brief ROI区域滤波
 *
 * 基于XY平面距离进行滤波，仅保留ROI半径内的点
 */
void ObstacleAreaDetection::applyROIFilter(const PointCloud::Ptr& input, PointCloud::Ptr& output) {
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

/**
 * @brief 点云坐标系变换
 *
 * 使用TF查找变换矩阵，然后用PCL进行点云变换
 */
bool ObstacleAreaDetection::transformPointCloud(const PointCloud::Ptr& input, PointCloud::Ptr& output,
                        const std::string& source_frame, const std::string& target_frame) {
    try {
        tf::StampedTransform transform;
        tf_listener_.lookupTransform(target_frame, source_frame, ros::Time(0), transform);

        Eigen::Affine3f eigen_transform = Eigen::Affine3f::Identity();
        tf::Vector3 origin = transform.getOrigin();
        eigen_transform.translation() << origin.x(), origin.y(), origin.z();

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

/**
 * @brief RANSAC地面平面分割
 *
 * 使用RANSAC算法拟合平面模型，将点云分为地面和非地面两部分
 */
void ObstacleAreaDetection::segmentGroundPlane(const PointCloud::Ptr& input,
                        PointCloud::Ptr& ground,
                        PointCloud::Ptr& obstacles) {
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