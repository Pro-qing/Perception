/**
 * @file state_callbacks.cpp
 * @brief 状态回调模块
 *
 * 本文件包含所有ROS订阅者的回调函数:
 * - keyPointPathCallback()       导航关键点路径(获取目标点位姿和类型)
 * - feedbackStatusCallback()     反馈状态(库位场景启用条件)
 * - elevatorInfoCallback()       电梯信息(电梯场景启用条件)
 * - currentPoseCallback()        当前车辆位姿(距离判断)
 * - lqrDireCallback()            电梯控制方向标志
 * - floorSetCallback()           目标楼层设置
 */

#include "obstacle_detection/obstacle_detection.hpp"

/**
 * @brief 关键点路径回调 - 获取目标点信息
 *
 * 从导航路径的最后一个关键点中提取目标类型(电梯/库位)和位姿
 * 电梯通过robustElevatorCheck验证connects类型数据
 */
void ObstacleDetection::keyPointPathCallback(const autoware_msgs::KeyPointArray::ConstPtr& msg) {
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
                // 目标点切换时清除库位检测历史记录
                garage_detection_history_.clear();
                break;
            }
        }
    }
}

/**
 * @brief 反馈状态回调 - 库位场景启用条件
 *
 * 当任务类型为1且距离目标 < garage_enable_distance_ 时启用库位检测
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

/**
 * @brief 电梯信息回调 - 电梯场景启用条件
 *
 * 启用条件: 电梯门打开 + 控制标志为±4 + 楼层匹配
 */
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

/**
 * @brief 当前位姿回调 - 距离判断
 *
 * 当车辆距离目标点过近(< distance_threshold)时禁用检测，避免车辆自身被误检
 */
void ObstacleDetection::currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pose_ = *msg;

    if (!targetPoint_.enable) {
        return;
    }

    double dx = current_pose_.pose.position.x - targetPoint_.map_pose.pose.position.x;
    double dy = current_pose_.pose.position.y - targetPoint_.map_pose.pose.position.y;
    double distance = sqrt(dx * dx + dy * dy);

    if (distance < distance_threshold_) {
        targetPoint_.enable = false;
        ROS_INFO("Distance to target point (%.3f m) is less than threshold (%.3f m), disabling targetPoint",
                 distance, distance_threshold_);
    }
}

/** @brief 电梯控制方向标志回调 */
void ObstacleDetection::lqrDireCallback(const std_msgs::Int8::ConstPtr& msg)
{
    elevator_control_flag_ = msg->data;
}

/** @brief 目标楼层设置回调 */
void ObstacleDetection::floorSetCallback(const std_msgs::Int8::ConstPtr& msg)
{
    floor_set_ = msg->data;
}