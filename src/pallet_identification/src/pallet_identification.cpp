#include "pallet_identification/pallet_identification.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pallet_identification");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    PalletIdentification pi(nh, private_nh);
    ros::spin();
    return 0;
}

PalletIdentification::PalletIdentification(ros::NodeHandle& nh, ros::NodeHandle& private_nh) : nh_(nh), private_nh_(private_nh)
{
    // 获取配置文件路径
    if (!private_nh.getParam("config_file", config_file_)) {
        ROS_FATAL("Missing 'config_file' parameter");
        ros::shutdown();
        return;
    }

    // 初始化加载参数
    loadConfig();

    // 启动后台监听线程
    config_watch_thread_ = std::thread(&PalletIdentification::watchConfig, this);
    
    // 初始化订阅者和发布者
    pallet_pose_sub_ = nh_.subscribe("/pallet_pose", 10, &PalletIdentification::palletPoseCallback, this);
    ndt_pose_sub_ = nh_.subscribe("/ndt_pose", 10, &PalletIdentification::ndtPoseCallback, this);
    arrived_pub_ = nh_.advertise<std_msgs::Int8>("/arrived_flag", 10);
}

PalletIdentification::~PalletIdentification()
{
    if (config_watch_thread_.joinable()) {
        config_watch_thread_.join();
    }
}

void PalletIdentification::loadConfig()
{
    try {
        YAML::Node config = YAML::LoadFile(config_file_);
        if (config["pallet_identification"]) {
            auto node = config["pallet_identification"];
            distance_threshold_.store(node["distance_threshold"].as<double>(0.5));
            angle_threshold_.store(node["angle_threshold"].as<double>(0.2));
            ROS_INFO("PalletIdentification params reloaded: distance=%.2f, angle=%.2f", 
                     distance_threshold_.load(), angle_threshold_.load());
        }
    } catch (const YAML::Exception& e) {
        ROS_WARN("Failed to load YAML config: %s", e.what());
    }
}

void PalletIdentification::watchConfig()
{
    int fd = inotify_init();
    if (fd < 0) { ROS_ERROR("inotify_init failed"); return; }

    // 获取文件所在的目录和文件名 (监听目录更安全，可以防范Vim等编辑器产生的临时文件覆盖行为)
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
        timeout.tv_sec = 1; // 1秒超时，用于定期检查 ros::ok()
        timeout.tv_usec = 0;

        int rv = select(fd + 1, &set, NULL, NULL, &timeout);
        if (rv > 0 && FD_ISSET(fd, &set)) {
            int length = read(fd, buffer, sizeof(buffer));
            int i = 0;
            while (i < length) {
                struct inotify_event *event = (struct inotify_event *)&buffer[i];
                if (event->len > 0) {
                    if (std::string(event->name) == file_name) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 延迟以确保文件写入完成
                        loadConfig();
                    }
                }
                i += sizeof(struct inotify_event) + event->len;
            }
        }
    }
    close(fd);
}

void PalletIdentification::palletPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pallet_pose_ = *msg;
}

void PalletIdentification::ndtPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_ndt_pose_ = *msg;
    
    std_msgs::Int8 arrived_msg;
    arrived_msg.data = 0;
    
    if (!isPoseValid(current_pallet_pose_)) {
        arrived_pub_.publish(arrived_msg);
        return;
    }
    
    double dx = current_ndt_pose_.pose.position.x - current_pallet_pose_.pose.position.x;
    double dy = current_ndt_pose_.pose.position.y - current_pallet_pose_.pose.position.y;
    double dz = current_ndt_pose_.pose.position.z - current_pallet_pose_.pose.position.z;
    double distance = sqrt(dx*dx + dy*dy + dz*dz);
    
    double ndt_yaw = getYawFromQuaternion(current_ndt_pose_.pose.orientation);
    double pallet_yaw = getYawFromQuaternion(current_pallet_pose_.pose.orientation);
    double angle_diff = fabs(ndt_yaw - pallet_yaw);
    if (angle_diff > M_PI) {
        angle_diff = 2 * M_PI - angle_diff;
    }
    
    // 使用 .load() 读取原子变量
    if (distance < distance_threshold_.load() && angle_diff < angle_threshold_.load()) {
        arrived_msg.data = 1;
    }

    ROS_INFO("distance:%.4f, angle_diff:%.4f (%.4f:%.4f)", 
        distance, angle_diff, distance_threshold_.load(), angle_threshold_.load());
    
    arrived_pub_.publish(arrived_msg);
}

bool PalletIdentification::isPoseValid(const geometry_msgs::PoseStamped& pose)
{
    return !(pose.pose.position.x == 0 && pose.pose.position.y == 0 && pose.pose.position.z == 0 &&
             pose.pose.orientation.x == 0 && pose.pose.orientation.y == 0 && 
             pose.pose.orientation.z == 0 && pose.pose.orientation.w == 0);
}

double PalletIdentification::getYawFromQuaternion(const geometry_msgs::Quaternion& quat)
{
    tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}