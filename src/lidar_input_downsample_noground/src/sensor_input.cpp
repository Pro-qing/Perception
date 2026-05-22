#include <lidar_input_downsample_noground/sensor_input.h>

SensorInputProcessor::SensorInputProcessor(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh)
{
    // ---- 从 YAML 读取基础参数 ----
    pnh_.param<std::string>("parent_frame", parent_frame_, "base_link");

    // ---- 加载标定参数并计算变换矩阵 ----
    loadCalibrationParams();

    // ---- 加载单线雷达距离过滤半径 ----
    pnh_.param<double>("filter_radius/left",  filter_radius_left_,  0.0);
    pnh_.param<double>("filter_radius/right", filter_radius_right_, 0.0);
    if (filter_radius_left_ > 0.0) {
        ROS_INFO("\033[1;34m[Sensor Input] Left filter radius: %.2f m\033[0m", filter_radius_left_);
    }
    if (filter_radius_right_ > 0.0) {
        ROS_INFO("\033[1;34m[Sensor Input] Right filter radius: %.2f m\033[0m", filter_radius_right_);
    }

    // ---- 加载行为模式配置 ----
    loadBehaviorConfigs();

    // ---- 初始化默认行为：全部启用 ----
    sensor_enabled_["main"]  = true;
    sensor_enabled_["mid"]   = true;
    sensor_enabled_["left"]  = true;
    sensor_enabled_["right"] = true;

    // ---- 发布 TF 静态变换 ----
    publishSensorTF();
    pub_main_calib_  = nh_.advertise<sensor_msgs::PointCloud2>("/points_main_calibration", 10);
    pub_mid_calib_   = nh_.advertise<sensor_msgs::PointCloud2>("/points_mid_calibration", 10);
    pub_left_calib_  = nh_.advertise<sensor_msgs::PointCloud2>("/points_left_calibration", 10);
    pub_right_calib_ = nh_.advertise<sensor_msgs::PointCloud2>("/points_right_calibration", 10);

    ROS_INFO("\033[1;32m[Sensor Input] Processor initialized. All points will be in velodyne frame.\033[0m");
}

SensorInputProcessor::~SensorInputProcessor() {}

void SensorInputProcessor::setSensorEnabled(const std::map<std::string, bool>& sensor_enabled) {
    sensor_enabled_ = sensor_enabled;
}

std::map<std::string, bool> SensorInputProcessor::getSensorEnabled() const {
    return sensor_enabled_;
}

bool SensorInputProcessor::processInput(
    const sensor_msgs::PointCloud2::ConstPtr& msg_16,
    const sensor_msgs::PointCloud2::ConstPtr& msg_mid,
    const sensor_msgs::LaserScan::ConstPtr& msg_left,
    const sensor_msgs::LaserScan::ConstPtr& msg_right,
    std::mutex& mutex,
    pcl::PointCloud<pcl::PointXYZI>::Ptr& merged_cloud,
    ros::Time& stamp)
{
    merged_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());

    // 安全检查：消息不能为空
    if (!msg_16) {
        ROS_WARN_THROTTLE(5.0, "[Sensor Input] msg_16 is null, skipping.");
        return false;
    }

    stamp = msg_16->header.stamp;

    // 预分配：假设合并后点云大小约为各路之和的上限
    // 避免频繁 realloc
    merged_cloud->reserve(100000);

    // 读取当前行为模式 (需要外部锁保护)
    std::map<std::string, bool> enabled;
    {
        std::lock_guard<std::mutex> lock(mutex);
        enabled = sensor_enabled_;
    }

    if (enabled["main"]) {
        try {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_main(new pcl::PointCloud<pcl::PointXYZI>());
            cloud_main->reserve(40000);
            processCloud(msg_16, trans_main_, cloud_main);
            *merged_cloud += *cloud_main;
            publishCalibratedCloud(cloud_main, msg_16->header.stamp, pub_main_calib_);
        } catch (const std::exception& e) {
            ROS_WARN_THROTTLE(5.0, "[Sensor Input] Exception processing main: %s", e.what());
        } catch (...) {
            ROS_WARN_THROTTLE(5.0, "[Sensor Input] Unknown exception processing main.");
        }
    }

    if (enabled["mid"] && msg_mid) {
        try {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_mid(new pcl::PointCloud<pcl::PointXYZI>());
            cloud_mid->reserve(10000);
            processCloud(msg_mid, trans_mid_, cloud_mid);
            *merged_cloud += *cloud_mid;
            publishCalibratedCloud(cloud_mid, msg_mid->header.stamp, pub_mid_calib_);
        } catch (const std::exception& e) {
            ROS_WARN_THROTTLE(5.0, "[Sensor Input] Exception processing mid: %s", e.what());
        } catch (...) {
            ROS_WARN_THROTTLE(5.0, "[Sensor Input] Unknown exception processing mid.");
        }
    }

    if (enabled["left"] && msg_left) {
        try {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_left(new pcl::PointCloud<pcl::PointXYZI>());
            cloud_left->reserve(5000);
            processScan(msg_left, trans_left_, cloud_left);
            publishCalibratedCloud(cloud_left, msg_left->header.stamp, pub_left_calib_);
            if (filter_radius_left_ > 0.0) {
                filterByRadius(cloud_left, filter_radius_left_);
            }
            *merged_cloud += *cloud_left;
        } catch (const std::exception& e) {
            ROS_WARN_THROTTLE(5.0, "[Sensor Input] Exception processing left: %s", e.what());
        } catch (...) {
            ROS_WARN_THROTTLE(5.0, "[Sensor Input] Unknown exception processing left.");
        }
    }

    if (enabled["right"] && msg_right) {
        try {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_right(new pcl::PointCloud<pcl::PointXYZI>());
            cloud_right->reserve(5000);
            processScan(msg_right, trans_right_, cloud_right);
            publishCalibratedCloud(cloud_right, msg_right->header.stamp, pub_right_calib_);
            if (filter_radius_right_ > 0.0) {
                filterByRadius(cloud_right, filter_radius_right_);
            }
            *merged_cloud += *cloud_right;
        } catch (const std::exception& e) {
            ROS_WARN_THROTTLE(5.0, "[Sensor Input] Exception processing right: %s", e.what());
        } catch (...) {
            ROS_WARN_THROTTLE(5.0, "[Sensor Input] Unknown exception processing right.");
        }
    }

    // 设置点云属性
    merged_cloud->width  = merged_cloud->points.size();
    merged_cloud->height = 1;
    merged_cloud->is_dense = true;

    return !merged_cloud->empty();
}

// ============== 加载标定参数 ==============
void SensorInputProcessor::loadCalibrationParams() {
    Eigen::Affine3f trans_main_to_base  = getTransformFromParam("calibration/main");
    Eigen::Affine3f trans_mid_to_velo   = getTransformFromParam("calibration/mid");
    Eigen::Affine3f trans_left_to_velo  = getTransformFromParam("calibration/left");
    Eigen::Affine3f trans_right_to_velo = getTransformFromParam("calibration/right");

    trans_main_  = Eigen::Affine3f::Identity();
    trans_mid_   = trans_mid_to_velo;
    trans_left_  = trans_left_to_velo;
    trans_right_ = trans_right_to_velo;

    base_to_velo_ = trans_main_to_base;

    ROS_INFO("\033[1;34m[Sensor Input] Calibration matrices loaded.\033[0m");
}

// ============== 发布静态 TF ==============
void SensorInputProcessor::publishSensorTF() {
    std::vector<geometry_msgs::TransformStamped> transforms;

    // base_link → velodyne
    {
        geometry_msgs::TransformStamped ts;
        ts.header.stamp = ros::Time::now();
        ts.header.frame_id = "base_link";
        ts.child_frame_id = "velodyne";

        Eigen::Vector3f pos = base_to_velo_.translation();
        ts.transform.translation.x = pos.x();
        ts.transform.translation.y = pos.y();
        ts.transform.translation.z = pos.z();

        Eigen::Matrix3f rot = base_to_velo_.rotation();
        Eigen::Quaternionf quat(rot);
        ts.transform.rotation.x = quat.x();
        ts.transform.rotation.y = quat.y();
        ts.transform.rotation.z = quat.z();
        ts.transform.rotation.w = quat.w();

        transforms.push_back(ts);

        ROS_INFO("\033[1;36m[Sensor Input] TF: base_link -> velodyne [%+.2f, %+.2f, %+.2f]\033[0m",
                 pos.x(), pos.y(), pos.z());
    }

    std::vector<std::pair<std::string, Eigen::Affine3f>> sensor_transforms = {
        {"lidar_main",  trans_main_},
        {"lidar_mid",   trans_mid_},
        {"lidar_left",  trans_left_},
        {"lidar_right", trans_right_}
    };

    for (const auto& kv : sensor_transforms) {
        geometry_msgs::TransformStamped ts;
        ts.header.stamp = ros::Time::now();
        ts.header.frame_id = "velodyne";
        ts.child_frame_id = kv.first;

        Eigen::Vector3f pos = kv.second.translation();
        ts.transform.translation.x = pos.x();
        ts.transform.translation.y = pos.y();
        ts.transform.translation.z = pos.z();

        Eigen::Matrix3f rot = kv.second.rotation();
        Eigen::Quaternionf quat(rot);
        ts.transform.rotation.x = quat.x();
        ts.transform.rotation.y = quat.y();
        ts.transform.rotation.z = quat.z();
        ts.transform.rotation.w = quat.w();

        transforms.push_back(ts);

        ROS_INFO("\033[1;36m[Sensor Input] TF: velodyne -> %s [%+.2f, %+.2f, %+.2f]\033[0m",
                 kv.first.c_str(), pos.x(), pos.y(), pos.z());
    }

    static_broadcaster_.sendTransform(transforms);
}

Eigen::Affine3f SensorInputProcessor::getTransformFromParam(const std::string& param_ns) {
    double x = 0, y = 0, z = 0, roll = 0, pitch = 0, yaw = 0;
    pnh_.param(param_ns + "/x", x, 0.0);
    pnh_.param(param_ns + "/y", y, 0.0);
    pnh_.param(param_ns + "/z", z, 0.0);
    pnh_.param(param_ns + "/roll", roll, 0.0);
    pnh_.param(param_ns + "/pitch", pitch, 0.0);
    pnh_.param(param_ns + "/yaw", yaw, 0.0);

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << x, y, z;
    transform.rotate(Eigen::AngleAxisf(yaw,   Eigen::Vector3f::UnitZ()));
    transform.rotate(Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()));
    transform.rotate(Eigen::AngleAxisf(roll,  Eigen::Vector3f::UnitX()));
    return transform;
}

// ============== 加载行为模式配置 ==============
void SensorInputProcessor::loadBehaviorConfigs() {
    XmlRpc::XmlRpcValue behaviors;
    if (!pnh_.getParam("behaviors", behaviors)) {
        ROS_WARN("[Sensor Input] No 'behaviors' config found. All sensors always enabled.");
        return;
    }

    if (behaviors.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        ROS_WARN("[Sensor Input] 'behaviors' is not a struct. Skipping.");
        return;
    }

    for (auto it = behaviors.begin(); it != behaviors.end(); ++it) {
        int behavior_id = std::stoi(it->first);
        XmlRpc::XmlRpcValue& sensors = it->second;

        std::map<std::string, bool> config;
        if (sensors.hasMember("main"))  config["main"]  = static_cast<bool>(sensors["main"]);
        if (sensors.hasMember("mid"))   config["mid"]   = static_cast<bool>(sensors["mid"]);
        if (sensors.hasMember("left"))  config["left"]  = static_cast<bool>(sensors["left"]);
        if (sensors.hasMember("right")) config["right"] = static_cast<bool>(sensors["right"]);

        behavior_configs_[behavior_id] = config;
        ROS_INFO("[Sensor Input] Behavior %d loaded: main=%d, mid=%d, left=%d, right=%d",
                 behavior_id, config["main"], config["mid"], config["left"], config["right"]);
    }
}

// ============== 点云处理辅助函数 ==============
void SensorInputProcessor::processScan(const sensor_msgs::LaserScan::ConstPtr& scan_msg,
                                       const Eigen::Affine3f& transform,
                                       pcl::PointCloud<pcl::PointXYZI>::Ptr& out_cloud) {
    if (!scan_msg || scan_msg->ranges.empty()) {
        return;
    }
    sensor_msgs::PointCloud2 temp_cloud2;
    try { projector_.projectLaser(*scan_msg, temp_cloud2); } catch (...) { return; }
    if (temp_cloud2.data.empty()) {
        return;
    }
    pcl::PointCloud<pcl::PointXYZI> raw_pcl;
    pcl::fromROSMsg(temp_cloud2, raw_pcl);
    if (raw_pcl.empty()) {
        return;
    }
    pcl::transformPointCloud(raw_pcl, *out_cloud, transform);
}

void SensorInputProcessor::filterByRadius(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, double max_radius) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());
    filtered->reserve(cloud->size());
    double max_radius_sq = max_radius * max_radius;
    for (const auto& pt : cloud->points) {
        if (pt.x * pt.x + pt.y * pt.y + pt.z * pt.z <= max_radius_sq) {
            filtered->points.push_back(pt);
        }
    }
    filtered->width = filtered->points.size();
    filtered->height = 1;
    filtered->is_dense = true;
    cloud = filtered;
}

void SensorInputProcessor::processCloud(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
                                        const Eigen::Affine3f& transform,
                                        pcl::PointCloud<pcl::PointXYZI>::Ptr& out_cloud) {
    if (!cloud_msg || cloud_msg->data.empty()) {
        return;
    }
    pcl::PointCloud<pcl::PointXYZI> raw_pcl;
    pcl::fromROSMsg(*cloud_msg, raw_pcl);
    if (raw_pcl.empty()) {
        return;
    }
    pcl::transformPointCloud(raw_pcl, *out_cloud, transform);
}

void SensorInputProcessor::publishCalibratedCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                                                  const ros::Time& stamp, const ros::Publisher& pub) {
    if (cloud->empty() || pub.getNumSubscribers() == 0) {
        return;
    }
    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(*cloud, output_msg);
    output_msg.header.stamp    = stamp;
    output_msg.header.frame_id = parent_frame_;
    pub.publish(output_msg);
}