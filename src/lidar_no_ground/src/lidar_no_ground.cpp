#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>
#include <cmath>
#include <algorithm>
#include <memory>

// ============================================================
//  Autoware-style Ray Ground Filter
//
//  算法原理:
//    1. 将点云按水平角 (azimuth) 划分为若干扇区 (sector)
//    2. 在每个扇区内按径向距离排序
//    3. 从近到远逐点比较高度差与坡度，判断是否为地面
//    4. 使用 local_max_slope (近处) 和 general_max_slope (远处) 两级阈值
// ============================================================

class LidarNoGroundNode {
public:
    LidarNoGroundNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh), pnh_(pnh)
    {
        // ---- 从 YAML 读取话题参数 ----
        pnh_.param<std::string>("input_topic",  input_topic_,  "/points_downsampled");
        pnh_.param<std::string>("no_ground_topic", no_ground_topic_, "/lidar_no_ground");
        pnh_.param<std::string>("ground_topic",    ground_topic_,    "/lidar_ground");

        // ---- 从 YAML 读取 Ray Ground Filter 参数 ----
        pnh_.param<double>("sensor_height",            sensor_height_,            1.8);
        pnh_.param<double>("clipping_height",          clipping_height_,          0.2);
        pnh_.param<double>("min_point_distance",       min_point_distance_,       0.5);
        pnh_.param<double>("radial_divider_angle",     radial_divider_angle_,     0.1);   // 度
        pnh_.param<double>("concentric_divider_distance", concentric_divider_distance_, 0.0);
        pnh_.param<double>("local_max_slope",          local_max_slope_,          8.0);   // 度
        pnh_.param<double>("general_max_slope",        general_max_slope_,        5.0);   // 度
        pnh_.param<double>("min_height_threshold",     min_height_threshold_,     0.2);   // 米
        pnh_.param<double>("reclass_distance_threshold", reclass_distance_threshold_, 0.5); // 米

        // ---- 从 YAML 读取预过滤参数 ----
        pnh_.param<bool>("pre_filter/enable",          pre_filter_enable_,        false);
        pnh_.param<double>("pre_filter/min_z",         pre_filter_min_z_,        -3.0);
        pnh_.param<double>("pre_filter/max_z",         pre_filter_max_z_,         1.0);

        // 计算扇区数量 (360度 / radial_divider_angle)
        if (radial_divider_angle_ <= 0.0) {
            ROS_WARN("[Lidar No Ground] radial_divider_angle <= 0, using 0.1 degree.");
            radial_divider_angle_ = 0.1;
        }
        num_sectors_ = static_cast<int>(std::ceil(360.0 / radial_divider_angle_));

        // 将角度阈值转为弧度
        local_max_slope_rad_   = local_max_slope_   * M_PI / 180.0;
        general_max_slope_rad_ = general_max_slope_ * M_PI / 180.0;

        // ---- 发布者 ----
        pub_no_ground_ = nh_.advertise<sensor_msgs::PointCloud2>(no_ground_topic_, 10);
        pub_ground_    = nh_.advertise<sensor_msgs::PointCloud2>(ground_topic_,    10);

        // ---- 订阅者 ----
        sub_points_ = nh_.subscribe(input_topic_, 10, &LidarNoGroundNode::pointCloudCallback, this);

        ROS_INFO("\033[1;32m[Lidar No Ground] Node initialized (Ray Ground Filter).\033[0m");
        ROS_INFO("  input_topic:              %s", input_topic_.c_str());
        ROS_INFO("  no_ground_topic:          %s", no_ground_topic_.c_str());
        ROS_INFO("  ground_topic:             %s", ground_topic_.c_str());
        ROS_INFO("  sensor_height:            %.3f m", sensor_height_);
        ROS_INFO("  clipping_height:          %.3f m", clipping_height_);
        ROS_INFO("  min_point_distance:       %.3f m", min_point_distance_);
        ROS_INFO("  radial_divider_angle:     %.3f deg (%d sectors)", radial_divider_angle_, num_sectors_);
        ROS_INFO("  concentric_divider_distance: %.3f m", concentric_divider_distance_);
        ROS_INFO("  local_max_slope:          %.3f deg", local_max_slope_);
        ROS_INFO("  general_max_slope:        %.3f deg", general_max_slope_);
        ROS_INFO("  min_height_threshold:     %.3f m", min_height_threshold_);
        ROS_INFO("  reclass_distance_threshold: %.3f m", reclass_distance_threshold_);
        ROS_INFO("  pre_filter:               %s", pre_filter_enable_ ? "true" : "false");
        if (pre_filter_enable_) {
            ROS_INFO("  pre_filter z range:       [%.2f, %.2f]", pre_filter_min_z_, pre_filter_max_z_);
        }
    }

private:
    ros::NodeHandle nh_, pnh_;
    ros::Publisher  pub_no_ground_;
    ros::Publisher  pub_ground_;
    ros::Subscriber sub_points_;

    // 话题参数
    std::string input_topic_;
    std::string no_ground_topic_;
    std::string ground_topic_;

    // Ray Ground Filter 参数
    double sensor_height_;
    double clipping_height_;
    double min_point_distance_;
    double radial_divider_angle_;
    double concentric_divider_distance_;
    double local_max_slope_;
    double general_max_slope_;
    double min_height_threshold_;
    double reclass_distance_threshold_;

    int    num_sectors_;
    double local_max_slope_rad_;
    double general_max_slope_rad_;

    // 预过滤参数
    bool   pre_filter_enable_;
    double pre_filter_min_z_;
    double pre_filter_max_z_;

    // ============== 扇区内点的结构体 ==============
    struct PointData {
        pcl::PointXYZI point;
        double radius;       // sqrt(x^2 + y^2)
        double theta;        // atan2(y, x), [0, 2*PI)
        int    sector_idx;   // 所属扇区索引
    };

    // ============== 获取扇区索引 ==============
    int getSectorIndex(double theta_deg) const {
        // theta_deg 范围: [0, 360)
        int idx = static_cast<int>(theta_deg / radial_divider_angle_);
        if (idx >= num_sectors_) idx = num_sectors_ - 1;
        if (idx < 0) idx = 0;
        return idx;
    }

    // ============== 主回调 ==============
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        // 至少一个下游订阅者时才处理
        bool has_no_ground_sub = (pub_no_ground_.getNumSubscribers() > 0);
        bool has_ground_sub    = (pub_ground_.getNumSubscribers() > 0);
        if (!has_no_ground_sub && !has_ground_sub) {
            return;
        }

        // 转换为 PCL 点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->empty()) {
            ROS_WARN_THROTTLE(5.0, "[Lidar No Ground] Received empty cloud, skipping.");
            return;
        }

        // ---- 可选: 预过滤 (简单高度裁剪) ----
        if (pre_filter_enable_) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());
            filtered->reserve(cloud->size());
            for (const auto& pt : cloud->points) {
                if (pt.z >= pre_filter_min_z_ && pt.z <= pre_filter_max_z_) {
                    filtered->points.push_back(pt);
                }
            }
            filtered->width    = filtered->points.size();
            filtered->height   = 1;
            filtered->is_dense = true;
            cloud = filtered;

            if (cloud->empty()) {
                ROS_WARN_THROTTLE(5.0, "[Lidar No Ground] Cloud empty after pre_filter, skipping.");
                return;
            }
        }

        // ---- Step 1: 按扇区组织点云 ----
        std::vector<std::vector<PointData>> sectors(num_sectors_);

        for (const auto& pt : cloud->points) {
            double radius = std::sqrt(pt.x * pt.x + pt.y * pt.y);

            // 跳过距离过近的点 (传感器原点附近的噪声)
            if (radius < min_point_distance_) {
                continue;
            }

            // 计算水平角度 [0, 360)
            double theta = std::atan2(pt.y, pt.x) * 180.0 / M_PI;
            if (theta < 0.0) theta += 360.0;

            int sector = getSectorIndex(theta);

            PointData pd;
            pd.point      = pt;
            pd.radius     = radius;
            pd.theta      = theta * M_PI / 180.0;  // 转回弧度存储
            pd.sector_idx = sector;

            sectors[sector].push_back(pd);
        }

        // ---- Step 2: 在每个扇区内按径向距离排序 ----
        for (auto& sector : sectors) {
            std::sort(sector.begin(), sector.end(),
                [](const PointData& a, const PointData& b) {
                    return a.radius < b.radius;
                });
        }

        // ---- Step 3: 逐扇区进行地面判断 ----
        pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_cloud(new pcl::PointCloud<pcl::PointXYZI>());

        for (const auto& sector : sectors) {
            if (sector.empty()) continue;

            // 第一个点: 基于传感器高度判断
            // 如果该点高于 clipping_height 则为非地面
            for (size_t i = 0; i < sector.size(); ++i) {
                const auto& pd = sector[i];

                // 计算该点相对于传感器的相对高度
                // 假设点云已在 base_link 坐标系下, sensor_height_ 是传感器到地面的高度
                // 点的地面高度参考: -sensor_height_ (即 base_link 原点到地面的距离)
                double point_height = pd.point.z - (-sensor_height_);

                if (i == 0) {
                    // 第一个点: 如果高度超过 clipping_height 则非地面
                    if (std::abs(pd.point.z - (-sensor_height_)) > clipping_height_) {
                        no_ground_cloud->points.push_back(pd.point);
                    } else {
                        ground_cloud->points.push_back(pd.point);
                    }
                    continue;
                }

                // 非第一个点: 与前一个点比较坡度
                const auto& prev_pd = sector[i - 1];
                double height_diff = pd.point.z - prev_pd.point.z;
                double dist_diff   = pd.radius  - prev_pd.radius;

                // 计算坡度角
                double slope_angle = 0.0;
                if (dist_diff > 1e-6) {
                    slope_angle = std::atan2(std::abs(height_diff), dist_diff);
                }

                // 根据径向距离选择不同的坡度阈值
                // 距离在 concentric_divider_distance_ 以内用 local_max_slope_
                // 否则用 general_max_slope_
                double max_slope = general_max_slope_rad_;
                if (concentric_divider_distance_ > 0.0 && pd.radius < concentric_divider_distance_) {
                    max_slope = local_max_slope_rad_;
                }

                // 地面判断
                bool is_ground = false;

                if (slope_angle <= max_slope) {
                    // 坡度在阈值内 → 可能是地面
                    if (std::abs(height_diff) < min_height_threshold_) {
                        is_ground = true;
                    }
                }

                if (is_ground) {
                    ground_cloud->points.push_back(pd.point);
                } else {
                    no_ground_cloud->points.push_back(pd.point);
                }
            }
        }

        // ---- Step 4: 重分类 (Re-classification) ----
        // 对于被标记为非地面的点，检查是否靠近已确认的地面点
        // 如果距离足够近，则重新分类为地面
        if (reclass_distance_threshold_ > 0.0 && !ground_cloud->empty()) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr reclass_ground(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::PointCloud<pcl::PointXYZI>::Ptr reclass_no_ground(new pcl::PointCloud<pcl::PointXYZI>());

            // 建立地面点的简易索引 (按扇区)
            std::vector<std::vector<PointData>> ground_sectors(num_sectors_);
            for (const auto& pt : ground_cloud->points) {
                double theta = std::atan2(pt.y, pt.x) * 180.0 / M_PI;
                if (theta < 0.0) theta += 360.0;
                int sector = getSectorIndex(theta);

                PointData pd;
                pd.point      = pt;
                pd.radius     = std::sqrt(pt.x * pt.x + pt.y * pt.y);
                pd.theta      = theta * M_PI / 180.0;
                pd.sector_idx = sector;
                ground_sectors[sector].push_back(pd);
            }

            for (const auto& pt : no_ground_cloud->points) {
                double theta = std::atan2(pt.y, pt.x) * 180.0 / M_PI;
                if (theta < 0.0) theta += 360.0;
                int sector = getSectorIndex(theta);

                double radius = std::sqrt(pt.x * pt.x + pt.y * pt.y);
                bool reclassified = false;

                // 检查同一扇区内的地面点
                for (const auto& gpt : ground_sectors[sector]) {
                    double dist = std::sqrt(
                        (pt.x - gpt.point.x) * (pt.x - gpt.point.x) +
                        (pt.y - gpt.point.y) * (pt.y - gpt.point.y) +
                        (pt.z - gpt.point.z) * (pt.z - gpt.point.z));
                    if (dist < reclass_distance_threshold_) {
                        reclassified = true;
                        break;
                    }
                }

                if (reclassified) {
                    reclass_ground->points.push_back(pt);
                } else {
                    reclass_no_ground->points.push_back(pt);
                }
            }

            // 合并重分类后的地面点
            *ground_cloud    += *reclass_ground;
            no_ground_cloud   = reclass_no_ground;
        }

        // 设置点云属性
        ground_cloud->width    = ground_cloud->points.size();
        ground_cloud->height   = 1;
        ground_cloud->is_dense = true;

        no_ground_cloud->width    = no_ground_cloud->points.size();
        no_ground_cloud->height   = 1;
        no_ground_cloud->is_dense = true;

        // ---- 发布结果 ----
        if (has_ground_sub) {
            sensor_msgs::PointCloud2 ground_msg;
            pcl::toROSMsg(*ground_cloud, ground_msg);
            ground_msg.header.stamp    = msg->header.stamp;
            ground_msg.header.frame_id = msg->header.frame_id;
            pub_ground_.publish(ground_msg);
        }

        if (has_no_ground_sub) {
            sensor_msgs::PointCloud2 no_ground_msg;
            pcl::toROSMsg(*no_ground_cloud, no_ground_msg);
            no_ground_msg.header.stamp    = msg->header.stamp;
            no_ground_msg.header.frame_id = msg->header.frame_id;
            pub_no_ground_.publish(no_ground_msg);
        }

        ROS_INFO_THROTTLE(2.0, "[Lidar No Ground] ground=%lu, no_ground=%lu",
                          ground_cloud->size(), no_ground_cloud->size());
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_no_ground_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    LidarNoGroundNode node(nh, pnh);

    ros::spin();

    return 0;
}