#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/common/common.h>
#include <pcl/features/moment_of_inertia_estimation.h>

#include <autoware_msgs/DetectedObject.h>
#include <autoware_msgs/DetectedObjectArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

#include <tf/tf.h>

#include <vector>
#include <cmath>
#include <string>

// ============================================================
//  Lidar Shape Estimation (OBB)
//
//  算法原理:
//    1. 订阅 /detection/lidar_detector/objects (DetectedObjectArray)
//    2. 对每个 DetectedObject 提取其点云
//    3. 使用 MomentOfInertiaEstimation 计算 OBB (Oriented Bounding Box)
//    4. 更新物体的 pose (位置+朝向) 和 dimensions (长宽高)
//    5. 发布更新后的 DetectedObjectArray
// ============================================================

class LidarShapeEstimationNode {
public:
    LidarShapeEstimationNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh), pnh_(pnh)
    {
        // ---- 从 YAML 读取话题参数 ----
        pnh_.param<std::string>("input_topic",  input_topic_,  "/detection/lidar_detector/objects");
        pnh_.param<std::string>("output_topic", output_topic_, "/detection/lidar_detector/objects_obb");

        // ---- 从 YAML 读取 OBB 参数 ----
        pnh_.param<int>("min_cluster_size_for_obb", min_cluster_size_for_obb_, 4);
        pnh_.param<bool>("use_pca", use_pca_, true);

        // ---- 发布者 ----
        pub_objects_ = nh_.advertise<autoware_msgs::DetectedObjectArray>(output_topic_, 10);

        // ---- 订阅者 ----
        sub_objects_ = nh_.subscribe(input_topic_, 10,
            &LidarShapeEstimationNode::objectsCallback, this);

        ROS_INFO("\033[1;32m[Lidar Shape Estimation] Node initialized.\033[0m");
        ROS_INFO("  input_topic:  %s", input_topic_.c_str());
        ROS_INFO("  output_topic: %s", output_topic_.c_str());
        ROS_INFO("  min_cluster_size_for_obb: %d", min_cluster_size_for_obb_);
        ROS_INFO("  use_pca: %s", use_pca_ ? "true" : "false");
    }

private:
    ros::NodeHandle nh_, pnh_;
    ros::Publisher  pub_objects_;
    ros::Subscriber sub_objects_;

    // 话题参数
    std::string input_topic_;
    std::string output_topic_;

    // OBB 参数
    int  min_cluster_size_for_obb_;
    bool use_pca_;

    // ============== 使用 PCA 计算 OBB ==============
    bool computeOBB_PCA(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                        Eigen::Vector3f& obb_center,
                        Eigen::Vector3f& obb_dimensions,
                        Eigen::Matrix3f& obb_rotation)
    {
        if (cloud->size() < static_cast<size_t>(min_cluster_size_for_obb_)) {
            return false;
        }

        // 1. 计算质心
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud, centroid);

        // 2. 使用 PCL PCA 分析
        pcl::PCA<pcl::PointXYZ> pca;
        pca.setInputCloud(cloud);

        // 特征向量 (主方向)
        Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();

        // 3. 将所有点投影到主轴坐标系
        //    投影坐标 = eigen_vectors^T * (point - centroid)
        Eigen::Vector3f min_proj( std::numeric_limits<float>::max(),
                                  std::numeric_limits<float>::max(),
                                  std::numeric_limits<float>::max());
        Eigen::Vector3f max_proj(-std::numeric_limits<float>::max(),
                                 -std::numeric_limits<float>::max(),
                                 -std::numeric_limits<float>::max());

        for (const auto& pt : cloud->points) {
            Eigen::Vector3f p(pt.x - centroid[0], pt.y - centroid[1], pt.z - centroid[2]);
            Eigen::Vector3f proj = eigen_vectors.transpose() * p;

            for (int i = 0; i < 3; ++i) {
                if (proj[i] < min_proj[i]) min_proj[i] = proj[i];
                if (proj[i] > max_proj[i]) max_proj[i] = proj[i];
            }
        }

        // 4. 计算 OBB 尺寸 (长、宽、高)
        Eigen::Vector3f extent = max_proj - min_proj;
        obb_dimensions = extent;

        // 5. 计算 OBB 中心 (在主轴坐标系中的中心, 再转回世界坐标)
        Eigen::Vector3f center_proj = (min_proj + max_proj) / 2.0f;
        obb_center = eigen_vectors * center_proj + Eigen::Vector3f(centroid[0], centroid[1], centroid[2]);

        // 6. 旋转矩阵
        obb_rotation = eigen_vectors;

        return true;
    }

    // ============== 使用 MomentOfInertiaEstimation 计算 OBB ==============
    bool computeOBB_MomentOfInertia(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                     Eigen::Vector3f& obb_center,
                                     Eigen::Vector3f& obb_dimensions,
                                     Eigen::Matrix3f& obb_rotation)
    {
        if (cloud->size() < static_cast<size_t>(min_cluster_size_for_obb_)) {
            return false;
        }

        pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
        feature_extractor.setInputCloud(cloud);
        feature_extractor.compute();

        pcl::PointXYZ min_point_OBB;
        pcl::PointXYZ max_point_OBB;
        pcl::PointXYZ position_OBB;
        Eigen::Matrix3f rotational_matrix_OBB;

        feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);

        // OBB 中心
        obb_center = Eigen::Vector3f(position_OBB.x, position_OBB.y, position_OBB.z);

        // OBB 尺寸
        obb_dimensions = Eigen::Vector3f(
            max_point_OBB.x - min_point_OBB.x,
            max_point_OBB.y - min_point_OBB.y,
            max_point_OBB.z - min_point_OBB.z);

        // 旋转矩阵
        obb_rotation = rotational_matrix_OBB;

        return true;
    }

    // ============== 从旋转矩阵提取四元数 ==============
    geometry_msgs::Quaternion rotationMatrixToQuaternion(const Eigen::Matrix3f& R) {
        Eigen::Quaternionf q(R);
        geometry_msgs::Quaternion quat;
        quat.x = q.x();
        quat.y = q.y();
        quat.z = q.z();
        quat.w = q.w();
        return quat;
    }

    // ============== 主回调 ==============
    void objectsCallback(const autoware_msgs::DetectedObjectArray::ConstPtr& msg) {
        if (pub_objects_.getNumSubscribers() <= 0) {
            return;
        }

        autoware_msgs::DetectedObjectArray output_array;
        output_array.header = msg->header;

        int obb_count = 0;

        for (const auto& obj : msg->objects) {
            autoware_msgs::DetectedObject new_obj = obj;

            // 如果点云为空, 保留原始数据
            if (obj.pointcloud.data.empty()) {
                output_array.objects.push_back(new_obj);
                continue;
            }

            // 将点云转换为 PCL
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::fromROSMsg(obj.pointcloud, *cloud);

            if (cloud->size() < static_cast<size_t>(min_cluster_size_for_obb_)) {
                output_array.objects.push_back(new_obj);
                continue;
            }

            // 计算 OBB
            Eigen::Vector3f obb_center;
            Eigen::Vector3f obb_dimensions;
            Eigen::Matrix3f obb_rotation;

            bool success = false;
            if (use_pca_) {
                success = computeOBB_PCA(cloud, obb_center, obb_dimensions, obb_rotation);
            } else {
                success = computeOBB_MomentOfInertia(cloud, obb_center, obb_dimensions, obb_rotation);
            }

            if (!success) {
                output_array.objects.push_back(new_obj);
                continue;
            }

            // 确保 dimensions 为正值
            obb_dimensions = obb_dimensions.cwiseAbs();

            // 更新位姿 (OBB 中心)
            new_obj.pose.position.x = obb_center[0];
            new_obj.pose.position.y = obb_center[1];
            new_obj.pose.position.z = obb_center[2];

            // 更新朝向 (从旋转矩阵提取四元数)
            new_obj.pose.orientation = rotationMatrixToQuaternion(obb_rotation);

            // 更新尺寸
            // 确保 dimensions.x >= dimensions.y (长 >= 宽), 否则交换并旋转
            double length = obb_dimensions[0];  // 沿第一主轴
            double width  = obb_dimensions[1];  // 沿第二主轴
            double height = obb_dimensions[2];  // 沿第三主轴 (Z)

            // 标准化: length(x) >= width(y)
            if (width > length) {
                std::swap(length, width);
                // 需要绕 Z 轴旋转 90 度
                tf::Quaternion q_orig;
                tf::quaternionMsgToTF(new_obj.pose.orientation, q_orig);
                tf::Quaternion q_rot(tf::Vector3(0, 0, 1), M_PI / 2.0);
                tf::Quaternion q_new = q_orig * q_rot;
                tf::quaternionTFToMsg(q_new, new_obj.pose.orientation);
            }

            new_obj.dimensions.x = length;  // 长度
            new_obj.dimensions.y = width;   // 宽度
            new_obj.dimensions.z = height;  // 高度

            obb_count++;
            output_array.objects.push_back(new_obj);
        }

        pub_objects_.publish(output_array);

        ROS_INFO_THROTTLE(2.0,
            "[Shape Estimation] Input objects: %lu, OBB computed: %d",
            msg->objects.size(), obb_count);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_shape_estimation_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    LidarShapeEstimationNode node(nh, pnh);

    ros::spin();

    return 0;
}