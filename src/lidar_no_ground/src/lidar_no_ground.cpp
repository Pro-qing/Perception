#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>

#include <lidar_pipeline_monitor/PipelineMetrics.h>

#include <vector>
#include <cmath>
#include <algorithm>

// ============================================================
//  PCL RANSAC Ground Plane Segmentation
//
//  算法原理:
//    1. 可选预过滤 (高度裁剪)
//    2. 使用 PCL SACSegmentation + SACMODEL_PERPENDICULAR_PLANE
//       拟合地面平面 (垂直于 Z 轴的平面: ax + by + cz + d = 0)
//    3. RANSAC 内点 (inliers) = 地面点
//    4. RANSAC 外点 (outliers) = 障碍物点
//    5. 可选多次迭代拟合 (多层地面)
// ============================================================

class LidarNoGroundNode {
public:
    LidarNoGroundNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh), pnh_(pnh)
    {
        // ---- 从 YAML 读取话题参数 ----
        pnh_.param<std::string>("input_topic",      input_topic_,      "/points_downsampled");
        pnh_.param<std::string>("no_ground_topic",   no_ground_topic_,  "/lidar_no_ground");
        pnh_.param<std::string>("ground_topic",      ground_topic_,     "/lidar_ground");

        // ---- 从 YAML 读取 RANSAC 参数 ----
        pnh_.param<int>   ("ransac/max_iterations",   ransac_max_iterations_,   100);
        pnh_.param<double>("ransac/distance_threshold", ransac_distance_threshold_, 0.1);
        pnh_.param<double>("ransac/probability",       ransac_probability_,     0.99);
        pnh_.param<double>("ransac/eps_angle",         ransac_eps_angle_,       0.1);     // 度
        pnh_.param<bool>  ("ransac/optimize_coefficients", ransac_optimize_coeff_, true);
        pnh_.param<bool>  ("ransac/use_perpendicular", use_perpendicular_,      true);

        // ---- 从 YAML 读取地面约束参数 ----
        pnh_.param<double>("ground/max_height",       ground_max_height_,      0.5);    // 米
        pnh_.param<double>("ground/min_height",       ground_min_height_,     -2.0);    // 米

        // ---- 从 YAML 读取迭代拟合参数 ----
        pnh_.param<bool>  ("iterative/enable",        iterative_enable_,       false);
        pnh_.param<int>   ("iterative/max_iterations", iterative_max_iters_,   3);
        pnh_.param<double>("iterative/height_threshold", iterative_height_thresh_, 0.05);

        // ---- 从 YAML 读取预过滤参数 ----
        pnh_.param<bool>  ("pre_filter/enable",       pre_filter_enable_,      false);
        pnh_.param<double>("pre_filter/min_z",        pre_filter_min_z_,      -3.0);
        pnh_.param<double>("pre_filter/max_z",        pre_filter_max_z_,       1.0);

        // 将角度转为弧度
        ransac_eps_angle_rad_ = ransac_eps_angle_ * M_PI / 180.0;

        // ---- 发布者 ----
        pub_no_ground_ = nh_.advertise<sensor_msgs::PointCloud2>(no_ground_topic_, 10);
        pub_ground_    = nh_.advertise<sensor_msgs::PointCloud2>(ground_topic_,    10);

        pub_metrics_ = nh_.advertise<lidar_pipeline_monitor::PipelineMetrics>("/pipeline/metrics", 100);

        // ---- 订阅者 ----
        sub_points_ = nh_.subscribe(input_topic_, 10, &LidarNoGroundNode::pointCloudCallback, this);

        ROS_INFO("\033[1;32m[Lidar No Ground] Node initialized (PCL RANSAC Ground Plane Segmentation).\033[0m");
        ROS_INFO("  input_topic:              %s", input_topic_.c_str());
        ROS_INFO("  no_ground_topic:          %s", no_ground_topic_.c_str());
        ROS_INFO("  ground_topic:             %s", ground_topic_.c_str());
        ROS_INFO("  ransac max_iterations:    %d", ransac_max_iterations_);
        ROS_INFO("  ransac distance_threshold:%.3f m", ransac_distance_threshold_);
        ROS_INFO("  ransac probability:       %.3f", ransac_probability_);
        ROS_INFO("  ransac eps_angle:         %.3f deg", ransac_eps_angle_);
        ROS_INFO("  ransac optimize_coeff:    %s", ransac_optimize_coeff_ ? "true" : "false");
        ROS_INFO("  use_perpendicular:        %s", use_perpendicular_ ? "true" : "false");
        ROS_INFO("  ground height range:      [%.2f, %.2f] m", ground_min_height_, ground_max_height_);
        ROS_INFO("  iterative enable:         %s", iterative_enable_ ? "true" : "false");
        if (iterative_enable_) {
            ROS_INFO("  iterative max_iters:      %d", iterative_max_iters_);
            ROS_INFO("  iterative height_thresh:  %.3f m", iterative_height_thresh_);
        }
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

    ros::Publisher pub_metrics_; 

    // 话题参数
    std::string input_topic_;
    std::string no_ground_topic_;
    std::string ground_topic_;

    // RANSAC 参数
    int    ransac_max_iterations_;
    double ransac_distance_threshold_;
    double ransac_probability_;
    double ransac_eps_angle_;
    double ransac_eps_angle_rad_;
    bool   ransac_optimize_coeff_;
    bool   use_perpendicular_;

    // 地面约束参数
    double ground_max_height_;
    double ground_min_height_;

    // 迭代拟合参数
    bool   iterative_enable_;
    int    iterative_max_iters_;
    double iterative_height_thresh_;

    // 预过滤参数
    bool   pre_filter_enable_;
    double pre_filter_min_z_;
    double pre_filter_max_z_;

    // ============== 使用 RANSAC 拟合地面平面 ==============
    // 返回地面平面的模型系数 (ax + by + cz + d = 0)
    bool segmentGroundPlane(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
        pcl::PointIndices::Ptr& inlier_indices,
        pcl::ModelCoefficients::Ptr& coefficients)
    {
        if (cloud->size() < 3) {
            return false;
        }

        pcl::SACSegmentation<pcl::PointXYZI> seg;
        seg.setOptimizeCoefficients(ransac_optimize_coeff_);

        if (use_perpendicular_) {
            // 垂直于 Z 轴的平面 (地面)
            seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
            seg.setAxis(Eigen::Vector3f(0.0, 0.0, 1.0));
            seg.setEpsAngle(ransac_eps_angle_rad_);
        } else {
            // 普通平面模型
            seg.setModelType(pcl::SACMODEL_PLANE);
        }

        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(ransac_max_iterations_);
        seg.setDistanceThreshold(ransac_distance_threshold_);
        seg.setProbability(ransac_probability_);
        seg.setInputCloud(cloud);

        inlier_indices.reset(new pcl::PointIndices());
        coefficients.reset(new pcl::ModelCoefficients());
        seg.segment(*inlier_indices, *coefficients);

        if (inlier_indices->indices.empty()) {
            ROS_WARN_THROTTLE(5.0, "[Lidar No Ground] RANSAC found no inliers.");
            return false;
        }

        // 验证拟合出的平面是否合理 (法向量应该大致指向 Z 轴)
        if (coefficients->values.size() == 4) {
            double a = coefficients->values[0];
            double b = coefficients->values[1];
            double c = coefficients->values[2];
            // d = coefficients->values[3];

            // 归一化法向量
            double norm = std::sqrt(a * a + b * b + c * c);
            if (norm > 1e-6) {
                double nz = c / norm;
                // 法向量的 Z 分量应该接近 1 (地面法向量指向 Z 轴)
                if (std::abs(nz) < 0.5) {
                    ROS_WARN_THROTTLE(5.0,
                        "[Lidar No Ground] RANSAC plane normal Z component too small (%.3f), "
                        "may not be ground plane.", nz);
                }
            }
        }

        return true;
    }

    // ============== 对地面内点进行高度约束过滤 ==============
    void filterGroundByHeight(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
        const pcl::PointIndices::Ptr& inlier_indices,
        pcl::PointIndices::Ptr& true_ground_indices,
        pcl::PointIndices::Ptr& remaining_indices)
    {
        true_ground_indices.reset(new pcl::PointIndices());
        remaining_indices.reset(new pcl::PointIndices());

        for (const auto& idx : inlier_indices->indices) {
            double z = cloud->points[idx].z;
            if (z >= ground_min_height_ && z <= ground_max_height_) {
                true_ground_indices->indices.push_back(idx);
            } else {
                remaining_indices->indices.push_back(idx);
            }
        }
    }

    // ============== 主回调 ==============
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {

        ros::Time cb_start = ros::Time::now(); // 【新增头】
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

        // ---- 地面分割 ----
        pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_cloud(new pcl::PointCloud<pcl::PointXYZI>());

        if (iterative_enable_) {
            // ---- 多次迭代拟合 ----
            // 每次找到地面后移除地面内点, 再对剩余点继续拟合
            pcl::PointCloud<pcl::PointXYZI>::Ptr remaining_cloud(new pcl::PointCloud<pcl::PointXYZI>());
            *remaining_cloud = *cloud;

            for (int iter = 0; iter < iterative_max_iters_; ++iter) {
                if (remaining_cloud->size() < 3) break;

                pcl::PointIndices::Ptr inlier_indices;
                pcl::ModelCoefficients::Ptr coefficients;

                if (!segmentGroundPlane(remaining_cloud, inlier_indices, coefficients)) {
                    break;
                }

                // 对 RANSAC 内点进行高度约束
                pcl::PointIndices::Ptr true_ground_idx;
                pcl::PointIndices::Ptr remaining_inlier_idx;
                filterGroundByHeight(remaining_cloud, inlier_indices,
                                     true_ground_idx, remaining_inlier_idx);

                // 提取真正的地面点
                pcl::PointCloud<pcl::PointXYZI>::Ptr iter_ground(new pcl::PointCloud<pcl::PointXYZI>());
                for (const auto& idx : true_ground_idx->indices) {
                    iter_ground->points.push_back(remaining_cloud->points[idx]);
                }
                *ground_cloud += *iter_ground;

                // 从剩余点中移除 RANSAC 内点 (包括非高度约束内的)
                pcl::ExtractIndices<pcl::PointXYZI> extract;
                extract.setInputCloud(remaining_cloud);
                extract.setIndices(inlier_indices);
                extract.setNegative(true);  // 保留外点
                pcl::PointCloud<pcl::PointXYZI>::Ptr outliers(new pcl::PointCloud<pcl::PointXYZI>());
                extract.filter(*outliers);

                // 如果高度阈值启用, 对剩余点做进一步高度过滤
                if (iterative_height_thresh_ > 0.0 && iter < iterative_max_iters_ - 1) {
                    // 获取地面平面系数
                    double a = coefficients->values[0];
                    double b = coefficients->values[1];
                    double c = coefficients->values[2];
                    double d = coefficients->values[3];
                    double norm = std::sqrt(a * a + b * b + c * c);

                    pcl::PointCloud<pcl::PointXYZI>::Ptr height_filtered(new pcl::PointCloud<pcl::PointXYZI>());
                    for (const auto& pt : outliers->points) {
                        // 计算点到平面的距离
                        double dist = std::abs(a * pt.x + b * pt.y + c * pt.z + d) / norm;
                        if (dist > iterative_height_thresh_) {
                            height_filtered->points.push_back(pt);
                        }
                    }
                    remaining_cloud = height_filtered;
                } else {
                    remaining_cloud = outliers;
                }
            }

            // 最终剩余的点都是非地面点
            no_ground_cloud = remaining_cloud;

        } else {
            // ---- 单次 RANSAC 拟合 ----
            pcl::PointIndices::Ptr inlier_indices;
            pcl::ModelCoefficients::Ptr coefficients;

            if (segmentGroundPlane(cloud, inlier_indices, coefficients)) {
                // 对 RANSAC 内点进行高度约束
                pcl::PointIndices::Ptr true_ground_idx;
                pcl::PointIndices::Ptr remaining_inlier_idx;
                filterGroundByHeight(cloud, inlier_indices,
                                     true_ground_idx, remaining_inlier_idx);

                // 提取地面点 (内点中满足高度约束的)
                pcl::ExtractIndices<pcl::PointXYZI> extract;

                // 地面点
                extract.setInputCloud(cloud);
                extract.setIndices(true_ground_idx);
                extract.setNegative(false);
                extract.filter(*ground_cloud);

                // 非地面点 = 外点 + 内点中不满足高度约束的
                // 合并外点索引和剩余内点索引
                pcl::PointIndices::Ptr outlier_indices(new pcl::PointIndices());

                // 获取外点 (不在 inlier 中的点)
                // 使用 setNegative 取反
                extract.setInputCloud(cloud);
                extract.setIndices(inlier_indices);
                extract.setNegative(true);  // 保留外点
                pcl::PointCloud<pcl::PointXYZI>::Ptr outliers(new pcl::PointCloud<pcl::PointXYZI>());
                extract.filter(*outliers);
                *no_ground_cloud += *outliers;

                // 加入内点中不满足高度约束的
                extract.setInputCloud(cloud);
                extract.setIndices(remaining_inlier_idx);
                extract.setNegative(false);
                pcl::PointCloud<pcl::PointXYZI>::Ptr remain_inliers(new pcl::PointCloud<pcl::PointXYZI>());
                extract.filter(*remain_inliers);
                *no_ground_cloud += *remain_inliers;

                // 打印拟合的平面方程
                if (coefficients->values.size() == 4) {
                    ROS_INFO_THROTTLE(5.0,
                        "[Lidar No Ground] Ground plane: %.3fx + %.3fy + %.3fz + %.3f = 0, "
                        "inliers=%lu, ground=%lu, no_ground=%lu",
                        coefficients->values[0], coefficients->values[1],
                        coefficients->values[2], coefficients->values[3],
                        inlier_indices->indices.size(),
                        ground_cloud->size(), no_ground_cloud->size());
                }
            } else {
                // RANSAC 失败, 将所有点作为非地面
                *no_ground_cloud = *cloud;
                ROS_WARN_THROTTLE(5.0, "[Lidar No Ground] RANSAC failed, all points treated as non-ground.");
            }
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

        // 【新增尾】
        ros::Time cb_end = ros::Time::now();
        lidar_pipeline_monitor::PipelineMetrics metric;
        metric.header.stamp = msg->header.stamp; 
        metric.node_name = "3_no_ground";
        metric.transmission_delay = (cb_start - msg->header.stamp).toSec() * 1000.0;
        metric.processing_time = (cb_end - cb_start).toSec() * 1000.0;
        metric.total_latency = (cb_end - msg->header.stamp).toSec() * 1000.0;
        pub_metrics_.publish(metric);
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