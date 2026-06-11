#include "pallet_identification/safe_tip_lift.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "safe_tip_lift");
    SafeTipLift safetiplift;
    ros::spin();
    return 0;
}

SafeTipLift::SafeTipLift() {
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    if (!private_nh.getParam("config_file", config_file_)) {
        ROS_FATAL("Missing 'config_file' parameter");
        ros::shutdown();
        return;
    }

    // 初始加载配置
    loadConfig();

    // 启动监听线程
    config_watch_thread_ = std::thread(&SafeTipLift::watchConfig, this);
    
    can_info_sub_ = nh.subscribe<autoware_can_msgs::CANInfo>("/can_info", 10, &SafeTipLift::canInfoCallback, this);
    vehicle_cmd_sub_ = nh.subscribe<autoware_msgs::VehicleCmd>("/vehicle_cmd_smooth", 10, &SafeTipLift::vehicleCmdCallback, this);
    arrived_flag_sub_ = nh.subscribe<std_msgs::Int8>("/arrived_flag", 10, &SafeTipLift::arrivedFlagCallback, this);
    feedback_status_sub_ = nh.subscribe<autoware_remove_msgs::State>("/feedback_status", 10, &SafeTipLift::feedbackStatusCallback, this);

    safetiplift_pub_ = nh.advertise<std_msgs::UInt16MultiArray>("/safe_tip_lift", 10);
    
    arrived_flag_ = 1;
    current_angle_ = 0.0;
    actual_lift_ = 0;
    current_speed_ = 0;
    state_ptr_ = nullptr;
}

SafeTipLift::~SafeTipLift() {
    if (config_watch_thread_.joinable()) {
        config_watch_thread_.join();
    }
}

void SafeTipLift::loadConfig() {
    try {
        YAML::Node config = YAML::LoadFile(config_file_);
        if (config["safe_tip_lift"]) {
            auto node = config["safe_tip_lift"];
            
            // 参数
            base_def_safelift_.store(node["base_def_safelift"].as<uint16_t>(30));
            base_pallet_safelift_.store(node["base_pallet_safelift"].as<uint16_t>(20));
            max_angle_.store(node["max_angle"].as<double>(1.5708));
            min_adjustment_.store(node["min_adjustment"].as<double>(0.5));
            max_adjustment_.store(node["max_adjustment"].as<double>(1.0));
            smoothness_factor_.store(node["smoothness_factor"].as<double>(2.0));
            min_safe_lift_threshold_.store(node["min_safe_lift_threshold"].as<uint16_t>(15));
            collision_disable_distance_.store(node["collision_disable_distance"].as<double>(0.05));
            sigmoid_steepness_.store(node["sigmoid_steepness"].as<double>(10.0));
            reciprocal_k_.store(node["reciprocal_k"].as<double>(0.5));
            
            ROS_INFO("SafeTipLift params reloaded: def=%d, pal=%d, min_threshold=%d, col_dist=%.2f", 
                     base_def_safelift_.load(), 
                     base_pallet_safelift_.load(), 
                     min_safe_lift_threshold_.load(), 
                     collision_disable_distance_.load());
        }
    } catch (const YAML::Exception& e) {
        ROS_WARN("Failed to load YAML config: %s", e.what());
    }
}

void SafeTipLift::watchConfig() {
    int fd = inotify_init();
    if (fd < 0) { ROS_ERROR("inotify_init failed"); return; }

    size_t last_slash = config_file_.find_last_of('/');
    std::string dir_path = config_file_.substr(0, last_slash);
    std::string file_name = config_file_.substr(last_slash + 1);

    int wd = inotify_add_watch(fd, dir_path.c_str(), IN_CLOSE_WRITE | IN_MOVED_TO);
    if (wd < 0) { ROS_ERROR("inotify_add_watch failed"); return; }

    char buffer[4096];
    while (ros::ok()) {
        fd_set set;
        FD_ZERO(&set);
        FD_SET(fd, &set);
        struct timeval timeout;
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;

        int rv = select(fd + 1, &set, NULL, NULL, &timeout);
        if (rv > 0 && FD_ISSET(fd, &set)) {
            int length = read(fd, buffer, sizeof(buffer));
            int i = 0;
            while (i < length) {
                struct inotify_event *event = (struct inotify_event *)&buffer[i];
                if (event->len > 0) {
                    if (std::string(event->name) == file_name) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(50));
                        loadConfig();
                    }
                }
                i += sizeof(struct inotify_event) + event->len;
            }
        }
    }
    close(fd);
}

void SafeTipLift::canInfoCallback(const autoware_can_msgs::CANInfo::ConstPtr& msg) {
}

void SafeTipLift::vehicleCmdCallback(const autoware_msgs::VehicleCmd::ConstPtr& msg) {
    current_angle_ = msg->ctrl_cmd.steering_angle;
    publishSafeTipLift();
}

void SafeTipLift::arrivedFlagCallback(const std_msgs::Int8::ConstPtr& msg) {
    arrived_flag_ = msg->data;
    publishSafeTipLift();
}

void SafeTipLift::feedbackStatusCallback(const autoware_remove_msgs::State::ConstPtr& msg) {
    state_ptr_ = msg;
    actual_lift_ = msg->VehicleInfo.lift;
    inplace_ = msg->VehicleInfo.inplace;
    current_speed_ = msg->VehicleCmd.speed;
    publishSafeTipLift();
}

void SafeTipLift::publishSafeTipLift() {
    std_msgs::UInt16MultiArray safelift_msg;
    
    // 使用 .load() 读取原子变量
    uint16_t base_safelift = (arrived_flag_ == 1) ? base_pallet_safelift_.load() : base_def_safelift_.load();
    uint16_t adjusted_safelift = calculateInverseSafeLift(base_safelift, current_angle_);

    // 读取最小限制阈值（取代之前的常量 15）
    uint16_t current_min_threshold = min_safe_lift_threshold_.load();
    if (adjusted_safelift < current_min_threshold) {
        adjusted_safelift = current_min_threshold;
    }
    
    uint16_t below_threshold = (actual_lift_ < adjusted_safelift && (state_ptr_!=nullptr && state_ptr_->TaskInfo.state != 0)) ? 1 : 0;
    
    // 读取防撞机制失效的距离阈值（取代之前的常量 0.05）
    double current_col_disable_dist = collision_disable_distance_.load();
    if (state_ptr_!=nullptr &&
        state_ptr_->VehicleInfo.param[0] < 0 &&     
        state_ptr_->TaskInfo.type == 1 &&      
        state_ptr_->TaskInfo.state != 0 &&      
        state_ptr_->TaskInfo.site.dis < current_col_disable_dist)   
    {
        below_threshold = 0;
    }

    safelift_msg.data.resize(3);
    safelift_msg.data[0] = adjusted_safelift;  
    safelift_msg.data[1] = actual_lift_;       
    safelift_msg.data[2] = below_threshold;    
    
    safetiplift_pub_.publish(safelift_msg);
}

uint16_t SafeTipLift::calculateInverseSafeLift(uint16_t base_safelift, double angle_rad) {
    double abs_angle_rad = std::abs(angle_rad);
    // 使用 .load() 读取原子变量参与计算
    double current_max_angle = max_angle_.load();
    double current_min_adj = min_adjustment_.load();
    double current_max_adj = max_adjustment_.load();
    double current_smoothness = smoothness_factor_.load();

    double clamped_angle = std::min(abs_angle_rad, current_max_angle);
    double normalized_angle = clamped_angle / current_max_angle;
    
    double adjustment_factor = current_max_adj - 
                               (current_max_adj - current_min_adj) * 
                               std::pow(normalized_angle, current_smoothness);
    
    uint16_t adjusted_value = static_cast<uint16_t>(base_safelift * adjustment_factor);
    uint16_t min_safe_value = static_cast<uint16_t>(base_safelift * current_min_adj);
    return std::max(adjusted_value, min_safe_value);
}

uint16_t SafeTipLift::calculateInverseSigmoidSafeLift(uint16_t base_safelift, double angle_rad) {
    double abs_angle_rad = std::abs(angle_rad);
    double current_max_angle = max_angle_.load();
    double clamped_angle = std::min(abs_angle_rad, current_max_angle);
    double normalized_angle = clamped_angle / current_max_angle;
    
    // 取代常量 10.0
    double current_steepness = sigmoid_steepness_.load();
    double sigmoid = 1.0 / (1.0 + std::exp(current_steepness * (normalized_angle - 0.5)));
    
    double current_min_adj = min_adjustment_.load();
    double current_max_adj = max_adjustment_.load();

    double adjustment_factor = current_min_adj + (current_max_adj - current_min_adj) * sigmoid;
    uint16_t adjusted_value = static_cast<uint16_t>(base_safelift * adjustment_factor);
    uint16_t min_safe_value = static_cast<uint16_t>(base_safelift * current_min_adj);
    return std::max(adjusted_value, min_safe_value);
}

uint16_t SafeTipLift::calculateReciprocalSafeLift(uint16_t base_safelift, double angle_rad) {
    double abs_angle_rad = std::abs(angle_rad);
    
    // 取代常量 0.5
    double current_k = reciprocal_k_.load(); 
    
    double current_min_adj = min_adjustment_.load();
    double current_max_adj = max_adjustment_.load();

    double adjustment_factor = current_max_adj / (1.0 + current_k * abs_angle_rad);
    adjustment_factor = std::max(adjustment_factor, current_min_adj);
    
    uint16_t adjusted_value = static_cast<uint16_t>(base_safelift * adjustment_factor);
    return adjusted_value;
}