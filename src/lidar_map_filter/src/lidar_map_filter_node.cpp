/**
 * @file lidar_map_filter_node.cpp
 * @brief lidar_map_filter 节点入口
 */

#include <ros/ros.h>
#include "lidar_map_filter/lidar_map_filter.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_map_filter_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    LidarMapFilterNode node(nh, pnh);

    ros::spin();

    return 0;
}