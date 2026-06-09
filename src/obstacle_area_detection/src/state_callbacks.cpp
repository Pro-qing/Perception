/**
 * @file state_callbacks.cpp
 * @brief 状态回调模块
 *
 * 本文件包含所有ROS订阅者的回调函数:
 * - keyPointPathCallback()       导航关键点路径(获取库位列表)
 * - currentPoseCallback()        当前车辆位姿(距离判断)
 */

#include "obstacle_area_detection/obstacle_area_detection.hpp"

/**
 * @brief 关键点路径回调 - 获取库位列表
 *
 * 从导航路径中提取所有 carports 类型的关键点，
 * 更新 parking_spots_ 列表。
 * 使用map_pose.position作为唯一标识来判断库位是否已存在。
 */
void ObstacleAreaDetection::keyPointPathCallback(const autoware_msgs::KeyPointArray::ConstPtr& msg) {
    if (msg->path.empty()) {
        return;
    }

    // 收集所有carports类型的关键点
    std::vector<ParkingSpot> new_spots;
    int spot_index = 0;
    for (const auto& keypoint : msg->path) {
        for (const auto& type : keypoint.types) {
            if (type.type_name == "carports") {
                ParkingSpot spot;
                spot.id = generateSpotId(spot_index);
                spot.map_pose = keypoint.pose;
                spot.has_obstacle = false;
                spot.raw_detection = false;
                spot.enable = false;  // 将由currentPoseCallback根据距离设置
                new_spots.push_back(spot);
                spot_index++;
                break;  // 每个keypoint只取第一个carports类型
            }
        }
    }

    // 检查是否与现有库位列表一致(通过位置比较)
    bool changed = false;
    if (new_spots.size() != parking_spots_.size()) {
        changed = true;
    } else {
        for (size_t i = 0; i < new_spots.size(); i++) {
            double dx = new_spots[i].map_pose.pose.position.x - parking_spots_[i].map_pose.pose.position.x;
            double dy = new_spots[i].map_pose.pose.position.y - parking_spots_[i].map_pose.pose.position.y;
            if (std::sqrt(dx * dx + dy * dy) > 0.01) {
                changed = true;
                break;
            }
        }
    }

    if (changed) {
        // 保留已有库位的检测历史(如果位置未变)
        std::map<std::string, std::deque<bool>> preserved_history;
        std::map<std::string, bool> preserved_confirmed;

        for (const auto& new_spot : new_spots) {
            // 在旧列表中查找位置相近的库位
            for (const auto& old_spot : parking_spots_) {
                double dx = new_spot.map_pose.pose.position.x - old_spot.map_pose.pose.position.x;
                double dy = new_spot.map_pose.pose.position.y - old_spot.map_pose.pose.position.y;
                if (std::sqrt(dx * dx + dy * dy) < 0.01) {
                    // 位置未变，保留历史
                    if (spot_detection_history_.count(old_spot.id)) {
                        preserved_history[new_spot.id] = spot_detection_history_[old_spot.id];
                    }
                    if (spot_confirmed_.count(old_spot.id)) {
                        preserved_confirmed[new_spot.id] = spot_confirmed_[old_spot.id];
                    }
                    break;
                }
            }
        }

        // 更新库位列表和历史记录
        parking_spots_ = new_spots;
        spot_detection_history_ = preserved_history;
        spot_confirmed_ = preserved_confirmed;

        ROS_INFO("Updated parking spots: %lu spots from keypoint path", parking_spots_.size());
        for (const auto& spot : parking_spots_) {
            ROS_INFO("  [%s] map_pose: (%.2f, %.2f, %.2f)",
                     spot.id.c_str(),
                     spot.map_pose.pose.position.x,
                     spot.map_pose.pose.position.y,
                     spot.map_pose.pose.position.z);
        }
    }
}

/**
 * @brief 当前位姿回调 - 距离判断
 *
 * 更新当前车辆位姿，并根据距离更新每个库位的enable状态。
 * 启用条件: distance < area_enable_distance 且 distance >= distance_threshold
 */
void ObstacleAreaDetection::currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pose_ = *msg;

    for (auto& spot : parking_spots_) {
        double dx = current_pose_.pose.position.x - spot.map_pose.pose.position.x;
        double dy = current_pose_.pose.position.y - spot.map_pose.pose.position.y;
        double distance = std::sqrt(dx * dx + dy * dy);

        if (distance < distance_threshold_) {
            // 太近，禁用(避免车辆自身被误检)
            if (spot.enable) {
                ROS_INFO("[%s] Distance to spot (%.3f m) < threshold (%.3f m), disabling",
                         spot.id.c_str(), distance, distance_threshold_);
            }
            spot.enable = false;
        } else if (distance <= area_enable_distance_) {
            // 在启用范围内
            if (!spot.enable) {
                ROS_INFO("[%s] Enabling detection, distance: %.3f m", spot.id.c_str(), distance);
            }
            spot.enable = true;
        } else {
            // 太远，禁用
            if (spot.enable) {
                ROS_DEBUG("[%s] Distance to spot (%.3f m) > enable distance (%.3f m), disabling",
                          spot.id.c_str(), distance, area_enable_distance_);
            }
            spot.enable = false;
        }
    }
}