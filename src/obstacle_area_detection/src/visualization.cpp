/**
 * @file visualization.cpp
 * @brief 可视化模块
 *
 * 本文件包含所有ROS可视化相关的发布函数:
 * - publishAreaMarkers()       发布所有库位的3D边界框+状态文本
 * - publishPointClouds()       发布地面/障碍物点云
 * - publishProximityInfo()     发布外围聚类检测的可视化信息
 */

#include "obstacle_area_detection/obstacle_area_detection.hpp"

/**
 * @brief 发布所有库位的可视化标记
 *
 * 在RVIZ中绘制每个库位的3D边界框(线框) + 状态文本
 * 有障碍物时显示红色，无障碍物时显示青色
 * 未启用的库位显示为灰色
 */
void ObstacleAreaDetection::publishAreaMarkers(const std_msgs::Header& header) {
    visualization_msgs::MarkerArray marker_array;

    for (size_t i = 0; i < parking_spots_.size(); i++) {
        const auto& spot = parking_spots_[i];
        int id = static_cast<int>(i);

        // 获取目标点在velodyne坐标系下的位姿
        geometry_msgs::Pose target_pose = transformTargetPoseToVelodyne(spot.map_pose);

        // 边界参数
        double min_x = area_bounds_.min_x, max_x = area_bounds_.max_x;
        double min_y = area_bounds_.min_y, max_y = area_bounds_.max_y;
        double min_z = area_bounds_.min_z, max_z = area_bounds_.max_z;

        // ========== 边界框线框标记 ==========
        visualization_msgs::Marker box_marker;
        box_marker.header = header;
        box_marker.header.frame_id = target_frame_;
        box_marker.ns = "area_box";
        box_marker.id = id;
        box_marker.type = visualization_msgs::Marker::LINE_LIST;
        box_marker.action = visualization_msgs::Marker::ADD;
        box_marker.pose = target_pose;
        box_marker.scale.x = 0.05;
        box_marker.scale.y = 0.0;
        box_marker.scale.z = 0.0;

        // 颜色: 红色=有障碍物, 青色=无障碍物, 灰色=未启用
        if (!spot.enable) {
            box_marker.color.r = 0.5;
            box_marker.color.g = 0.5;
            box_marker.color.b = 0.5;
            box_marker.color.a = 0.5;
        } else if (spot.has_obstacle) {
            box_marker.color.r = 1.0;
            box_marker.color.g = 0.0;
            box_marker.color.b = 0.0;
            box_marker.color.a = 1.0;
        } else {
            box_marker.color.r = 0.0;
            box_marker.color.g = 1.0;
            box_marker.color.b = 1.0;
            box_marker.color.a = 1.0;
        }

        box_marker.lifetime = ros::Duration(1.0);

        // 定义8个顶点(在目标点局部坐标系下)
        std::vector<geometry_msgs::Point> vertices(8);

        // 底面
        vertices[0].x = min_x; vertices[0].y = min_y; vertices[0].z = min_z;
        vertices[1].x = max_x; vertices[1].y = min_y; vertices[1].z = min_z;
        vertices[2].x = max_x; vertices[2].y = max_y; vertices[2].z = min_z;
        vertices[3].x = min_x; vertices[3].y = max_y; vertices[3].z = min_z;

        // 顶面
        vertices[4].x = min_x; vertices[4].y = min_y; vertices[4].z = max_z;
        vertices[5].x = max_x; vertices[5].y = min_y; vertices[5].z = max_z;
        vertices[6].x = max_x; vertices[6].y = max_y; vertices[6].z = max_z;
        vertices[7].x = min_x; vertices[7].y = max_y; vertices[7].z = max_z;

        // 底面4条边
        box_marker.points.push_back(vertices[0]); box_marker.points.push_back(vertices[1]);
        box_marker.points.push_back(vertices[1]); box_marker.points.push_back(vertices[2]);
        box_marker.points.push_back(vertices[2]); box_marker.points.push_back(vertices[3]);
        box_marker.points.push_back(vertices[3]); box_marker.points.push_back(vertices[0]);

        // 顶面4条边
        box_marker.points.push_back(vertices[4]); box_marker.points.push_back(vertices[5]);
        box_marker.points.push_back(vertices[5]); box_marker.points.push_back(vertices[6]);
        box_marker.points.push_back(vertices[6]); box_marker.points.push_back(vertices[7]);
        box_marker.points.push_back(vertices[7]); box_marker.points.push_back(vertices[4]);

        // 侧面4条竖边
        box_marker.points.push_back(vertices[0]); box_marker.points.push_back(vertices[4]);
        box_marker.points.push_back(vertices[1]); box_marker.points.push_back(vertices[5]);
        box_marker.points.push_back(vertices[2]); box_marker.points.push_back(vertices[6]);
        box_marker.points.push_back(vertices[3]); box_marker.points.push_back(vertices[7]);

        marker_array.markers.push_back(box_marker);

        // ========== 状态文本标记 ==========
        visualization_msgs::Marker text_marker;
        text_marker.header = header;
        text_marker.header.frame_id = target_frame_;
        text_marker.ns = "area_text";
        text_marker.id = id;
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;
        text_marker.pose = target_pose;
        text_marker.pose.position.z += max_z + 0.2;  // 文本显示在边界框上方
        text_marker.scale.z = 0.2;

        if (!spot.enable) {
            text_marker.color.r = 0.5;
            text_marker.color.g = 0.5;
            text_marker.color.b = 0.5;
            text_marker.color.a = 0.5;
            text_marker.text = spot.id + " [DISABLED]";
        } else if (spot.has_obstacle) {
            text_marker.color.r = 1.0;
            text_marker.color.g = 0.0;
            text_marker.color.b = 0.0;
            text_marker.color.a = 1.0;
            text_marker.text = spot.id + " [OCCUPIED]";
        } else {
            text_marker.color.r = 0.0;
            text_marker.color.g = 1.0;
            text_marker.color.b = 1.0;
            text_marker.color.a = 1.0;
            text_marker.text = spot.id + " [CLEAR]";
        }

        text_marker.lifetime = ros::Duration(1.0);
        marker_array.markers.push_back(text_marker);
    }

    area_marker_pub_.publish(marker_array);
}

/**
 * @brief 发布各类点云(地面、障碍物)
 */
void ObstacleAreaDetection::publishPointClouds(const PointCloud::Ptr& ground,
                        const PointCloud::Ptr& obstacles,
                        const std_msgs::Header& header) {
    if (ground->size() > 0) {
        sensor_msgs::PointCloud2 ground_msg;
        pcl::toROSMsg(*ground, ground_msg);
        ground_msg.header = header;
        ground_pub_.publish(ground_msg);
    }

    if (obstacles->size() > 0) {
        sensor_msgs::PointCloud2 obstacle_msg;
        pcl::toROSMsg(*obstacles, obstacle_msg);
        obstacle_msg.header = header;
        obstacle_pub_.publish(obstacle_msg);
    }
}

/**
 * @brief 发布外围聚类检测的可视化信息
 *
 * 发布检测到的cluster点云 + 文本标记 + 边界框线框
 */
void ObstacleAreaDetection::publishProximityInfo(const ParkingSpot& spot,
                                                 const PointCloud::Ptr& cluster_cloud,
                                                 const geometry_msgs::Point& centroid,
                                                 double distance_to_box,
                                                 size_t point_count,
                                                 const std_msgs::Header& header) {
    // 发布cluster点云
    if (proximity_cloud_pub_.getNumSubscribers() > 0 && cluster_cloud && !cluster_cloud->empty()) {
        sensor_msgs::PointCloud2 cluster_msg;
        pcl::toROSMsg(*cluster_cloud, cluster_msg);
        cluster_msg.header = header;
        proximity_cloud_pub_.publish(cluster_msg);
    }

    // 发布标记
    if (proximity_marker_pub_.getNumSubscribers() > 0) {
        visualization_msgs::MarkerArray marker_array;

        // 文本标记
        visualization_msgs::Marker text_marker;
        text_marker.header = header;
        text_marker.header.frame_id = target_frame_;
        text_marker.ns = "area_proximity";
        text_marker.id = 0;
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;
        text_marker.pose.position = centroid;
        text_marker.pose.position.z += 0.3;
        text_marker.pose.orientation.w = 1.0;
        text_marker.scale.z = 0.15;
        text_marker.color.r = 1.0;
        text_marker.color.g = 0.5;
        text_marker.color.b = 0.0;
        text_marker.color.a = 1.0;
        text_marker.text = spot.id + " PROXIMITY[" + std::to_string(point_count) + "pts,"
                           + "d=" + std::to_string(distance_to_box).substr(0, 4) + "m]";
        text_marker.lifetime = ros::Duration(1.0);
        marker_array.markers.push_back(text_marker);

        proximity_marker_pub_.publish(marker_array);
    }
}