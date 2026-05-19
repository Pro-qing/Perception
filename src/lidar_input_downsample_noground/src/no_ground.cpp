#include <lidar_input_downsample_noground/no_ground.h>

NoGroundProcessor::NoGroundProcessor(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh)
{
    // ---- 从 YAML 读取 RANSAC 参数 ----
    pnh_.param<int>   ("ransac/max_iterations",   ransac_max_iterations_,   100);
    pnh_.param<double>("ransac/distance_threshold", ransac_distance_threshold_, 0.1);
    pnh_.param<double>("ransac/probability",       ransac_probability_,     0.99);
    pnh_.param<double>("ransac/eps_angle",         ransac_eps_angle_,       0.1);
    pnh_.param<bool>  ("ransac/optimize_coefficients", ransac_optimize_coeff_, true);
    pnh_.param<bool>  ("ransac/use_perpendicular", use_perpendicular_,      true);

    // ---- 从 YAML 读取地面约束参数 ----
    pnh_.param<double>("ground/max_height",       ground_max_height_,      0.5);
    pnh_.param<double>("ground/min_height",       ground_min_height_,     -2.0);

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

    ROS_INFO("\033[1;32m[No Ground] Processor initialized (PCL RANSAC Ground Plane Segmentation).\033[0m");
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

void NoGroundProcessor::processNoGround(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    pcl::PointCloud<pcl::PointXYZI>::Ptr& ground_cloud,
    pcl::PointCloud<pcl::PointXYZI>::Ptr& no_ground_cloud)
{
    ground_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
    no_ground_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());

    if (cloud->empty()) {
        return;
    }

    // 预分配
    ground_cloud->reserve(cloud->size() / 3);
    no_ground_cloud->reserve(cloud->size());

    pcl::PointCloud<pcl::PointXYZI>::Ptr working_cloud(new pcl::PointCloud<pcl::PointXYZI>());

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
        working_cloud = filtered;

        if (working_cloud->empty()) {
            return;
        }
    } else {
        working_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>(*cloud));
    }

    // ---- 地面分割 ----
    if (iterative_enable_) {
        // ---- 多次迭代拟合 ----
        pcl::PointCloud<pcl::PointXYZI>::Ptr remaining_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        *remaining_cloud = *working_cloud;

        for (int iter = 0; iter < iterative_max_iters_; ++iter) {
            if (remaining_cloud->size() < 3) break;

            pcl::PointIndices::Ptr inlier_indices;
            pcl::ModelCoefficients::Ptr coefficients;

            if (!segmentGroundPlane(remaining_cloud, inlier_indices, coefficients)) {
                break;
            }

            pcl::PointIndices::Ptr true_ground_idx;
            pcl::PointIndices::Ptr remaining_inlier_idx;
            filterGroundByHeight(remaining_cloud, inlier_indices,
                                 true_ground_idx, remaining_inlier_idx);

            pcl::PointCloud<pcl::PointXYZI>::Ptr iter_ground(new pcl::PointCloud<pcl::PointXYZI>());
            for (const auto& idx : true_ground_idx->indices) {
                iter_ground->points.push_back(remaining_cloud->points[idx]);
            }
            *ground_cloud += *iter_ground;

            pcl::ExtractIndices<pcl::PointXYZI> extract;
            extract.setInputCloud(remaining_cloud);
            extract.setIndices(inlier_indices);
            extract.setNegative(true);
            pcl::PointCloud<pcl::PointXYZI>::Ptr outliers(new pcl::PointCloud<pcl::PointXYZI>());
            extract.filter(*outliers);

            if (iterative_height_thresh_ > 0.0 && iter < iterative_max_iters_ - 1) {
                double a = coefficients->values[0];
                double b = coefficients->values[1];
                double c = coefficients->values[2];
                double d = coefficients->values[3];
                double norm = std::sqrt(a * a + b * b + c * c);

                pcl::PointCloud<pcl::PointXYZI>::Ptr height_filtered(new pcl::PointCloud<pcl::PointXYZI>());
                for (const auto& pt : outliers->points) {
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

        no_ground_cloud = remaining_cloud;

    } else {
        // ---- 单次 RANSAC 拟合 ----
        pcl::PointIndices::Ptr inlier_indices;
        pcl::ModelCoefficients::Ptr coefficients;

        if (segmentGroundPlane(working_cloud, inlier_indices, coefficients)) {
            pcl::PointIndices::Ptr true_ground_idx;
            pcl::PointIndices::Ptr remaining_inlier_idx;
            filterGroundByHeight(working_cloud, inlier_indices,
                                 true_ground_idx, remaining_inlier_idx);

            pcl::ExtractIndices<pcl::PointXYZI> extract;

            // 地面点
            extract.setInputCloud(working_cloud);
            extract.setIndices(true_ground_idx);
            extract.setNegative(false);
            extract.filter(*ground_cloud);

            // 外点
            extract.setInputCloud(working_cloud);
            extract.setIndices(inlier_indices);
            extract.setNegative(true);
            pcl::PointCloud<pcl::PointXYZI>::Ptr outliers(new pcl::PointCloud<pcl::PointXYZI>());
            extract.filter(*outliers);
            *no_ground_cloud += *outliers;

            // 内点中不满足高度约束的
            extract.setInputCloud(working_cloud);
            extract.setIndices(remaining_inlier_idx);
            extract.setNegative(false);
            pcl::PointCloud<pcl::PointXYZI>::Ptr remain_inliers(new pcl::PointCloud<pcl::PointXYZI>());
            extract.filter(*remain_inliers);
            *no_ground_cloud += *remain_inliers;

            if (coefficients->values.size() == 4) {
                ROS_INFO_THROTTLE(5.0,
                    "[No Ground] Ground plane: %.3fx + %.3fy + %.3fz + %.3f = 0, "
                    "inliers=%lu, ground=%lu, no_ground=%lu",
                    coefficients->values[0], coefficients->values[1],
                    coefficients->values[2], coefficients->values[3],
                    inlier_indices->indices.size(),
                    ground_cloud->size(), no_ground_cloud->size());
            }
        } else {
            *no_ground_cloud = *working_cloud;
            ROS_WARN_THROTTLE(5.0, "[No Ground] RANSAC failed, all points treated as non-ground.");
        }
    }

    // 设置点云属性
    ground_cloud->width    = ground_cloud->points.size();
    ground_cloud->height   = 1;
    ground_cloud->is_dense = true;

    no_ground_cloud->width    = no_ground_cloud->points.size();
    no_ground_cloud->height   = 1;
    no_ground_cloud->is_dense = true;
}

// ============== 使用 RANSAC 拟合地面平面 ==============
bool NoGroundProcessor::segmentGroundPlane(
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
        seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
        seg.setAxis(Eigen::Vector3f(0.0, 0.0, 1.0));
        seg.setEpsAngle(ransac_eps_angle_rad_);
    } else {
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
        ROS_WARN_THROTTLE(5.0, "[No Ground] RANSAC found no inliers.");
        return false;
    }

    // 验证拟合出的平面是否合理
    if (coefficients->values.size() == 4) {
        double a = coefficients->values[0];
        double b = coefficients->values[1];
        double c = coefficients->values[2];

        double norm = std::sqrt(a * a + b * b + c * c);
        if (norm > 1e-6) {
            double nz = c / norm;
            if (std::abs(nz) < 0.5) {
                ROS_WARN_THROTTLE(5.0,
                    "[No Ground] RANSAC plane normal Z component too small (%.3f), "
                    "may not be ground plane.", nz);
            }
        }
    }

    return true;
}

// ============== 对地面内点进行高度约束过滤 ==============
void NoGroundProcessor::filterGroundByHeight(
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