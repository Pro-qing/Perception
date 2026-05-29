/**
 * @file visualization.cpp
 * @brief 可视化模块
 *
 * 本文件包含所有ROS可视化相关的发布函数:
 * - publishPointClouds()          发布地面/障碍物/聚类点云
 * - publishColoredClusters()      发布彩色聚类点云
 * - publishObstacleInfo()         发布障碍物信息可视化标记
 * - publishTargetRegionMarker()   发布目标区域3D线框
 * - setColorByDistance()          根据距离设置颜色
 */

#include "obstacle_detection/obstacle_detection.hpp"

/**
 * @brief 发布各类点云(地面、障碍物、聚类)
 */
void ObstacleDetection::publishPointClouds(const PointCloud::Ptr& ground,
                        const PointCloud::Ptr& obstacles,
                        const std::vector<pcl::PointIndices>& cluster_indices,
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

    if (!cluster_indices.empty()) {
        publishColoredClusters(obstacles, cluster_indices, header);
    }
}

/**
 * @brief 发布彩色聚类点云
 *
 * 为每个聚类分配不同颜色，便于在RVIZ中可视化区分不同障碍物
 */
void ObstacleDetection::publishColoredClusters(const PointCloud::Ptr& cloud,
                            const std::vector<pcl::PointIndices>& cluster_indices,
                            const std_msgs::Header& header) {
    pcl::PointCloud<pcl::PointXYZRGB> colored_cloud;

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

/**
 * @brief 发布障碍物信息可视化标记
 */
void ObstacleDetection::publishObstacleInfo(const std::vector<ObstacleInfo>& obstacles,
                        const std_msgs::Header& header) {
    visualization_msgs::MarkerArray marker_array;
    marker_pub_.publish(marker_array);
}

/**
 * @brief 发布目标区域可视化标记
 *
 * 在RVIZ中绘制目标区域的3D边界框(线框)
 * 有噪点时显示红色，无噪点时显示白色(电梯)或青色(库位)
 */
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

    visualization_msgs::Marker region_marker;
    region_marker.header = header;
    region_marker.header.frame_id = target_frame_;
    region_marker.ns = "target_region";
    region_marker.id = 0;
    region_marker.type = visualization_msgs::Marker::LINE_LIST;
    region_marker.action = visualization_msgs::Marker::ADD;

    region_marker.pose = transformTargetPoseToVelodyne(targetPoint_.map_pose);

    region_marker.scale.x = 0.05;
    region_marker.scale.y = 0.0;
    region_marker.scale.z = 0.0;

    if (target_region_has_noise_) {
        region_marker.color.r = 1.0;
        region_marker.color.g = 0.0;
        region_marker.color.b = 0.0;
        region_marker.color.a = 1.0;
    } else {
        region_marker.color.r = l_r;
        region_marker.color.g = l_g;
        region_marker.color.b = l_b;
        region_marker.color.a = l_a;
    }

    region_marker.lifetime = ros::Duration(1.0);

    // 定义8个顶点(在目标点局部坐标系下)
    std::vector<geometry_msgs::Point> vertices(8);

    // 底面
    vertices[0].x = temp_min_x; vertices[0].y = temp_min_y; vertices[0].z = temp_min_z;
    vertices[1].x = temp_max_x; vertices[1].y = temp_min_y; vertices[1].z = temp_min_z;
    vertices[2].x = temp_max_x; vertices[2].y = temp_max_y; vertices[2].z = temp_min_z;
    vertices[3].x = temp_min_x; vertices[3].y = temp_max_y; vertices[3].z = temp_min_z;

    // 顶面
    vertices[4].x = temp_min_x; vertices[4].y = temp_min_y; vertices[4].z = temp_max_z;
    vertices[5].x = temp_max_x; vertices[5].y = temp_min_y; vertices[5].z = temp_max_z;
    vertices[6].x = temp_max_x; vertices[6].y = temp_max_y; vertices[6].z = temp_max_z;
    vertices[7].x = temp_min_x; vertices[7].y = temp_max_y; vertices[7].z = temp_max_z;

    // 底面4条边
    region_marker.points.push_back(vertices[0]); region_marker.points.push_back(vertices[1]);
    region_marker.points.push_back(vertices[1]); region_marker.points.push_back(vertices[2]);
    region_marker.points.push_back(vertices[2]); region_marker.points.push_back(vertices[3]);
    region_marker.points.push_back(vertices[3]); region_marker.points.push_back(vertices[0]);

    // 顶面4条边
    region_marker.points.push_back(vertices[4]); region_marker.points.push_back(vertices[5]);
    region_marker.points.push_back(vertices[5]); region_marker.points.push_back(vertices[6]);
    region_marker.points.push_back(vertices[6]); region_marker.points.push_back(vertices[7]);
    region_marker.points.push_back(vertices[7]); region_marker.points.push_back(vertices[4]);

    // 侧面4条竖边
    region_marker.points.push_back(vertices[0]); region_marker.points.push_back(vertices[4]);
    region_marker.points.push_back(vertices[1]); region_marker.points.push_back(vertices[5]);
    region_marker.points.push_back(vertices[2]); region_marker.points.push_back(vertices[6]);
    region_marker.points.push_back(vertices[3]); region_marker.points.push_back(vertices[7]);

    marker_array.markers.push_back(region_marker);
    marker_pub_.publish(marker_array);
}

/**
 * @brief 根据距离设置颜色(近红远绿)
 */
void ObstacleDetection::setColorByDistance(std_msgs::ColorRGBA& color, double distance) {
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
    color.a = 0.5;
}