/**
 * @file area_detection.cpp
 * @brief 库区障碍物检测模块
 *
 * 本文件包含:
 * - checkAreaForSpot()            单库位区域检测(坐标变换+边界框判断)
 * - checkProximityCluster()       外围聚类补充检测(Layer 2)
 * - updateSpotDetectionState()    滑动窗口防抖+施密特触发器
 * - transformTargetPoseToVelodyne() 目标点坐标系变换(map→velodyne)
 */

#include "obstacle_area_detection/obstacle_area_detection.hpp"

/**
 * @brief 检测单个库位区域内的障碍物
 *
 * 将目标点从map坐标系变换到velodyne坐标系，然后将每个障碍物点变换到
 * 目标点的局部坐标系中，检查是否落在预定义的边界框内。
 */
bool ObstacleAreaDetection::checkAreaForSpot(const ParkingSpot& spot,
                                             const PointCloud::Ptr& obstacle_cloud,
                                             const std_msgs::Header& header) {
    // 转换目标点到当前坐标系
    geometry_msgs::Pose transformed_target_pose = transformTargetPoseToVelodyne(spot.map_pose);

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
    target_orientation.normalize();

    Eigen::Matrix3f rotation_matrix = target_orientation.toRotationMatrix();
    Eigen::Matrix3f inverse_rotation_matrix = rotation_matrix.inverse();

    // 边界参数
    double min_x = area_bounds_.min_x, max_x = area_bounds_.max_x;
    double min_y = area_bounds_.min_y, max_y = area_bounds_.max_y;
    double min_z = area_bounds_.min_z, max_z = area_bounds_.max_z;

    ROS_INFO_THROTTLE(2.0, "[%s] Target position in %s: (%.3f, %.3f, %.3f), bounds: x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f]",
                      spot.id.c_str(), target_frame_.c_str(),
                      target_position.x(), target_position.y(), target_position.z(),
                      min_x, max_x, min_y, max_y, min_z, max_z);

    // 收集区域内的点
    PointCloud::Ptr region_points(new PointCloud);
    size_t total_points = obstacle_cloud->size();
    size_t points_in_region = 0;

    // 遍历所有点，变换到目标点局部坐标系后判断是否在区域内
    for (const auto& point : *obstacle_cloud) {
        Eigen::Vector3f p(point.x, point.y, point.z);
        Eigen::Vector3f p_local = inverse_rotation_matrix * (p - target_position);

        bool x_in_range = (p_local.x() >= min_x) && (p_local.x() <= max_x);
        bool y_in_range = (p_local.y() >= min_y) && (p_local.y() <= max_y);
        bool z_in_range = (p_local.z() >= min_z) && (p_local.z() <= max_z);

        if (x_in_range && y_in_range && z_in_range) {
            region_points->push_back(point);
            points_in_region++;
        }
    }

    // 使用最小点数阈值判断是否有障碍物
    bool has_noise = (static_cast<int>(points_in_region) >= min_region_points_);
    if (!has_noise) {
        ROS_DEBUG_THROTTLE(1.0, "[%s] Region points (%lu) below threshold (%d), not considered noise",
                           spot.id.c_str(), points_in_region, min_region_points_);
    }

    // 发布区域内的点云(仅当点数满足阈值时才发布)
    if (static_cast<int>(region_points->size()) >= min_region_points_) {
        // 双重验证
        PointCloud::Ptr verified_points(new PointCloud);
        verified_points->reserve(region_points->size());

        for (const auto& point : *region_points) {
            Eigen::Vector3f p(point.x, point.y, point.z);
            Eigen::Vector3f p_local = inverse_rotation_matrix * (p - target_position);

            bool x_in = (p_local.x() >= min_x) && (p_local.x() <= max_x);
            bool y_in = (p_local.y() >= min_y) && (p_local.y() <= max_y);
            bool z_in = (p_local.z() >= min_z) && (p_local.z() <= max_z);

            if (x_in && y_in && z_in) {
                verified_points->push_back(point);
            }
        }

        if (verified_points->size() > 0) {
            sensor_msgs::PointCloud2 region_msg;
            pcl::toROSMsg(*verified_points, region_msg);
            region_msg.header = header;
            target_region_cloud_pub_.publish(region_msg);
        }

        ROS_INFO("[%s] Total: %lu, In region: %lu, Verified: %lu. Has noise: %s",
                 spot.id.c_str(), total_points, points_in_region, verified_points->size(),
                 has_noise ? "YES" : "NO");
    } else {
        ROS_DEBUG_THROTTLE(1.0, "[%s] No points found in target region (checked %lu total points)",
                           spot.id.c_str(), total_points);
    }

    return has_noise;
}

/**
 * @brief 库位外围聚类检测 - Layer 2补充检测
 *
 * 当Layer 1未检测到障碍物时触发。
 * 对预处理后的点云做欧几里得聚类，检查是否有聚类"贴着"库位边界框外侧。
 */
bool ObstacleAreaDetection::checkProximityCluster(const ParkingSpot& spot,
                                                  const PointCloud::Ptr& mid_cloud,
                                                  const std_msgs::Header& header) {
    if (!mid_cloud || mid_cloud->empty()) {
        return false;
    }

    // 获取目标点在velodyne坐标系下的位姿
    geometry_msgs::Pose transformed_target_pose = transformTargetPoseToVelodyne(spot.map_pose);

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
    target_orientation.normalize();

    Eigen::Matrix3f rotation_matrix = target_orientation.toRotationMatrix();
    Eigen::Matrix3f inverse_rotation_matrix = rotation_matrix.inverse();

    // 库位边界参数
    double min_x = area_bounds_.min_x, max_x = area_bounds_.max_x;
    double min_y = area_bounds_.min_y, max_y = area_bounds_.max_y;
    double min_z = area_bounds_.min_z, max_z = area_bounds_.max_z;

    // ========== 步骤1: 欧几里得聚类 ==========
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(mid_cloud);

    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(cluster_tolerance_);
    ec.setMinClusterSize(cluster_min_points_);
    ec.setMaxClusterSize(cluster_max_points_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(mid_cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    ec.extract(cluster_indices);

    if (proximity_debug_) {
        ROS_INFO("[%s][Proximity] Found %lu clusters from %lu points",
                 spot.id.c_str(), cluster_indices.size(), mid_cloud->size());
    }

    if (cluster_indices.empty()) {
        return false;
    }

    // ========== 步骤2-4: 逐个cluster过滤+邻近判断 ==========
    for (size_t ci = 0; ci < cluster_indices.size(); ci++) {
        const auto& indices = cluster_indices[ci].indices;

        // 提取聚类点云并计算边界
        PointT min_pt, max_pt;
        min_pt.x = min_pt.y = min_pt.z = std::numeric_limits<float>::max();
        max_pt.x = max_pt.y = max_pt.z = std::numeric_limits<float>::lowest();

        Eigen::Vector3f centroid(0.0f, 0.0f, 0.0f);
        for (const auto& idx : indices) {
            const auto& pt = mid_cloud->points[idx];
            centroid.x() += pt.x;
            centroid.y() += pt.y;
            centroid.z() += pt.z;
            if (pt.x < min_pt.x) min_pt.x = pt.x;
            if (pt.y < min_pt.y) min_pt.y = pt.y;
            if (pt.z < min_pt.z) min_pt.z = pt.z;
            if (pt.x > max_pt.x) max_pt.x = pt.x;
            if (pt.y > max_pt.y) max_pt.y = pt.y;
            if (pt.z > max_pt.z) max_pt.z = pt.z;
        }
        centroid /= static_cast<float>(indices.size());

        // 过滤条件: z范围检查(过滤地面合并的巨型cluster)
        double z_range = max_pt.z - min_pt.z;
        if (z_range > cluster_max_z_range_) {
            if (proximity_debug_) {
                ROS_INFO("[%s][Proximity] Cluster %lu REJECTED: z_range=%.2f > max=%.2f, points=%lu",
                         spot.id.c_str(), ci, z_range, cluster_max_z_range_, indices.size());
            }
            continue;
        }

        // 过滤条件: 重心z检查(过滤残余地面)
        if (centroid.z() < cluster_min_centroid_z_) {
            if (proximity_debug_) {
                ROS_INFO("[%s][Proximity] Cluster %lu REJECTED: centroid_z=%.2f < min=%.2f, points=%lu",
                         spot.id.c_str(), ci, centroid.z(), cluster_min_centroid_z_, indices.size());
            }
            continue;
        }

        // ========== 步骤3: 将聚类重心变换到目标点局部坐标系 ==========
        Eigen::Vector3f centroid_local = inverse_rotation_matrix * (centroid - target_position);

        // 计算重心到库位边界框的最近距离(AABB最近点距离)
        double dx = std::max(0.0, std::max(min_x - centroid_local.x(), centroid_local.x() - max_x));
        double dy = std::max(0.0, std::max(min_y - centroid_local.y(), centroid_local.y() - max_y));
        double distance_to_box = std::sqrt(dx * dx + dy * dy);

        // 检查聚类是否有部分点进入扩展边界框
        bool any_point_in_expanded = false;
        double expand_min_x = min_x - expand_margin_x_;
        double expand_max_x = max_x + expand_margin_x_;
        double expand_min_y = min_y - expand_margin_y_;
        double expand_max_y = max_y + expand_margin_y_;

        for (const auto& idx : indices) {
            const auto& pt = mid_cloud->points[idx];
            Eigen::Vector3f p(pt.x, pt.y, pt.z);
            Eigen::Vector3f p_local = inverse_rotation_matrix * (p - target_position);

            if (p_local.x() >= expand_min_x && p_local.x() <= expand_max_x &&
                p_local.y() >= expand_min_y && p_local.y() <= expand_max_y &&
                p_local.z() >= min_z && p_local.z() <= max_z) {
                any_point_in_expanded = true;
                break;
            }
        }

        if (proximity_debug_) {
            ROS_INFO("[%s][Proximity] Cluster %lu: points=%lu, z_range=%.2f, centroid_z=%.2f, "
                     "centroid_local=(%.2f,%.2f,%.2f), dist_to_box=%.2f, in_expanded=%s",
                     spot.id.c_str(), ci, indices.size(), z_range, centroid.z(),
                     centroid_local.x(), centroid_local.y(), centroid_local.z(),
                     distance_to_box, any_point_in_expanded ? "YES" : "NO");
        }

        // ========== 步骤5: 邻近判断 ==========
        if (distance_to_box < proximity_threshold_ || any_point_in_expanded) {
            ROS_INFO("[%s][Proximity] DETECTED: Cluster %lu with %lu points, dist=%.2f",
                     spot.id.c_str(), ci, indices.size(), distance_to_box);

            // 可视化
            PointCloud::Ptr detected_cluster(new PointCloud);
            for (const auto& idx : indices) {
                detected_cluster->push_back(mid_cloud->points[idx]);
            }
            geometry_msgs::Point centroid_msg;
            centroid_msg.x = centroid.x();
            centroid_msg.y = centroid.y();
            centroid_msg.z = centroid.z();
            publishProximityInfo(spot, detected_cluster, centroid_msg, distance_to_box,
                                indices.size(), header);

            return true;
        }
    }

    return false;
}

/**
 * @brief 更新单个库位的防抖状态
 *
 * 使用滑动窗口 + 施密特触发器:
 *   确认: noise_count >= confirm_threshold
 *   清除: noise_count <= clear_threshold
 *   中间状态: 保持当前状态不变
 */
void ObstacleAreaDetection::updateSpotDetectionState(const std::string& spot_id, bool raw_detection) {
    auto& history = spot_detection_history_[spot_id];

    // 添加当前帧检测结果到滑动窗口
    history.push_back(raw_detection);
    while (static_cast<int>(history.size()) > area_history_size_) {
        history.pop_front();
    }

    // 统计窗口内检测到障碍物的帧数
    int noise_count = 0;
    for (const auto& detection : history) {
        if (detection) noise_count++;
    }

    // 施密特触发器逻辑
    auto& confirmed = spot_confirmed_[spot_id];
    if (noise_count >= area_confirm_threshold_) {
        confirmed = true;
        ROS_DEBUG_THROTTLE(1.0, "[%s] Obstacle CONFIRMED: %d/%d frames detected",
                           spot_id.c_str(), noise_count, area_history_size_);
    } else if (noise_count <= area_clear_threshold_) {
        confirmed = false;
        if (noise_count > 0) {
            ROS_DEBUG_THROTTLE(1.0, "[%s] Obstacle pending: %d/%d frames (threshold: %d)",
                               spot_id.c_str(), noise_count, area_history_size_, area_confirm_threshold_);
        }
    }
    // else: 中间状态，保持 confirmed 不变，避免状态跳变

    // 回写到parking_spots_
    for (auto& spot : parking_spots_) {
        if (spot.id == spot_id) {
            spot.has_obstacle = confirmed;
            break;
        }
    }
}

/**
 * @brief 将目标点位姿从map坐标系变换到velodyne坐标系
 */
geometry_msgs::Pose ObstacleAreaDetection::transformTargetPoseToVelodyne(const geometry_msgs::PoseStamped& input_pose)
{
    try {
        ros::Time now = ros::Time::now();

        if (tf_listener_.waitForTransform(target_frame_, "map", now, ros::Duration(1.0))) {
            geometry_msgs::PoseStamped pose_in_map = input_pose;
            pose_in_map.header.stamp = now;
            pose_in_map.header.frame_id = "map";

            geometry_msgs::PoseStamped pose_in_target_frame;
            tf_listener_.transformPose(target_frame_, pose_in_map, pose_in_target_frame);

            return pose_in_target_frame.pose;
        } else {
            ROS_WARN("TF transform from map to %s not available, using original pose", target_frame_.c_str());
        }
    } catch (tf::TransformException &ex) {
        ROS_WARN("Failed to transform target point from map to %s: %s", target_frame_.c_str(), ex.what());
    }
    return input_pose.pose;
}