/**
 * @file target_region_detection.cpp
 * @brief 目标区域障碍物检测模块
 *
 * 本文件包含:
 * - checkTargetPointRegion()         核心检测逻辑(坐标变换+边界框判断)
 * - transformTargetPoseToVelodyne()  目标点坐标系变换(map→velodyne)
 * - robustElevatorCheck()            电梯数据健壮性解析
 */

#include "obstacle_detection/obstacle_detection.hpp"

/**
 * @brief 检测目标区域内的障碍物 - 核心检测逻辑
 *
 * 将目标点从map坐标系变换到velodyne坐标系，然后将每个障碍物点变换到
 * 目标点的局部坐标系中，检查是否落在预定义的边界框内。
 */
void ObstacleDetection::checkTargetPointRegion(const PointCloud::Ptr& obstacle_cloud,
                                             const std_msgs::Header& header) {
    // 转换目标点到当前坐标系
    geometry_msgs::Pose transformed_target_pose = transformTargetPoseToVelodyne(targetPoint_.map_pose);

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

    // 确保只在正确的类型下进行检测
    if (targetPoint_.type_e != POINT_TYPE_ELEVATOR && targetPoint_.type_e != POINT_TYPE_GARAGE) {
        ROS_DEBUG_THROTTLE(1.0, "Target point type is not ELEVATOR or GARAGE, skipping region check");
        target_region_has_noise_ = false;
        return;
    }

    // 获取边界参数
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
    size_t total_points = obstacle_cloud->size();
    size_t points_in_region = 0;
    size_t points_outside_region = 0;

    // 调试: 检测目标点位置变化
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
            if (targetPoint_.type_e == POINT_TYPE_ELEVATOR) {
                obstacle_detection_.data &= ~1;
            }
        } else {
            points_outside_region++;
        }
    }

    // 使用最小点数阈值判断是否有障碍物
    bool has_noise = (static_cast<int>(points_in_region) >= min_region_points_);
    if (!has_noise) {
        ROS_INFO_THROTTLE(1.0, "Region points (%lu) below threshold (%d), not considered noise",
                          points_in_region, min_region_points_);
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

    target_region_has_noise_ = has_noise;
    publishTargetRegionMarker(header);
}

/**
 * @brief 将目标点位姿从map坐标系变换到velodyne坐标系
 */
geometry_msgs::Pose ObstacleDetection::transformTargetPoseToVelodyne(const geometry_msgs::PoseStamped& input_pose)
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

/**
 * @brief 电梯数据健壮性检查
 *
 * 解析格式如 "'A1,1'" 或 "''A1,1''" 的电梯标识数据，
 * 验证第二个引号内值为"1"且最后值为"0"
 */
bool ObstacleDetection::robustElevatorCheck(const std::string& data) {
    ROS_INFO("robustElevatorCheck with data: %s", data.c_str());

    if (data.empty()) {
        ROS_WARN("Empty data received");
        return false;
    }

    size_t first_quote = data.find('\'');
    if (first_quote == std::string::npos) {
        ROS_WARN("No opening quote found");
        return false;
    }

    size_t second_quote = data.find('\'', first_quote + 1);
    if (second_quote == std::string::npos) {
        ROS_WARN("No closing quote found");
        return false;
    }

    size_t third_quote = data.find('\'', second_quote + 1);
    size_t fourth_quote = std::string::npos;

    if (third_quote != std::string::npos) {
        fourth_quote = data.find('\'', third_quote + 1);
    }

    std::string quoted_content;
    if (fourth_quote != std::string::npos) {
        quoted_content = data.substr(second_quote + 1, third_quote - second_quote - 1);
    } else {
        quoted_content = data.substr(first_quote + 1, second_quote - first_quote - 1);
    }

    ROS_INFO("Quoted content: %s", quoted_content.c_str());

    size_t comma_in_quoted = quoted_content.find(',');
    if (comma_in_quoted == std::string::npos) {
        ROS_WARN("No comma found in quoted content");
        return false;
    }

    std::string second_in_quotes = quoted_content.substr(comma_in_quoted + 1);

    size_t last_comma = data.find_last_of(',');
    if (last_comma == std::string::npos) {
        ROS_WARN("No comma found in data");
        return false;
    }

    std::string last_value = data.substr(last_comma + 1);

    ROS_INFO("Second in quotes: %s, Last value: %s",
             second_in_quotes.c_str(), last_value.c_str());

    bool result = (second_in_quotes == "1" && last_value == "0");
    ROS_INFO("Check result: %s", result ? "true" : "false");

    return result;
}