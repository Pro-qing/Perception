#ifndef __SAFE_TIP_LIFT_HPP
#define __SAFE_TIP_LIFT_HPP

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt16MultiArray.h>
#include <autoware_can_msgs/CANInfo.h>
#include <autoware_msgs/VehicleCmd.h>
#include <autoware_remove_msgs/State.h>
#include <cmath>
#include <algorithm>

// 新增依赖
#include <yaml-cpp/yaml.h>
#include <thread>
#include <atomic>
#include <string>
#include <sys/inotify.h>
#include <unistd.h>

class SafeTipLift{
public:
    SafeTipLift();
    ~SafeTipLift();

private:
    ros::Subscriber can_info_sub_, vehicle_cmd_sub_;
    ros::Subscriber arrived_flag_sub_;
    ros::Subscriber feedback_status_sub_;
    ros::Publisher safetiplift_pub_;

    // 线程安全的原子参数
    std::atomic<uint16_t> base_def_safelift_;
    std::atomic<uint16_t> base_pallet_safelift_;
    std::atomic<double> max_angle_;
    std::atomic<double> min_adjustment_;
    std::atomic<double> max_adjustment_;
    std::atomic<double> smoothness_factor_;
    
    std::atomic<uint16_t> min_safe_lift_threshold_; // 叉尖阈值下限
    std::atomic<double> collision_disable_distance_; // 防撞失效距离
    std::atomic<double> sigmoid_steepness_;          // S形曲线陡峭程度
    std::atomic<double> reciprocal_k_;               // 倒数函数衰减系数

    std::string config_file_;
    std::thread config_watch_thread_;
    void loadConfig();
    void watchConfig();

    int8_t arrived_flag_;
    double current_angle_;
    uint16_t actual_lift_;
    bool inplace_;
    double current_speed_;
    
    autoware_remove_msgs::State::ConstPtr state_ptr_;

    void canInfoCallback(const autoware_can_msgs::CANInfo::ConstPtr& msg);
    void vehicleCmdCallback(const autoware_msgs::VehicleCmd::ConstPtr& msg);
    void arrivedFlagCallback(const std_msgs::Int8::ConstPtr& msg);
    void feedbackStatusCallback(const autoware_remove_msgs::State::ConstPtr& msg);
    void publishSafeTipLift();
    
    uint16_t calculateInverseSafeLift(uint16_t base_safelift, double angle_rad);
    uint16_t calculateInverseSigmoidSafeLift(uint16_t base_safelift, double angle_rad);
    uint16_t calculateReciprocalSafeLift(uint16_t base_safelift, double angle_rad);
};

#endif