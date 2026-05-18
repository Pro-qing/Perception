#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
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

#include <lidar_pipeline_monitor/PipelineMetrics.h>

#include <tf/tf.h>

#include <vector>
#include <cmath>
#include <string>
#include <sstream>

// ============================================================
//  Lidar Shape Estimation (OBB)
//
//  算法原理:
//    1. 订阅 /detection/lidar_detector/objects (DetectedObjectArray)
//    2. 对每个 DetectedObject 提取其点云
//    3. 使用 PCA / MomentOfInertiaEstimation 计算 OBB
//    4. 更新物体的 pose (位置+朝向) 和 dimensions (长宽高)
//    5. 根据 OBB 尺寸分类标签 (person, car, truck, bicycle, box 等)
//    6. 发布更新后的 DetectedObjectArray
//    7. (debug 模式) 发布 MarkerArray 在 rviz 中可视化 OBB 和标签
// ============================================================

// 分类阈值结构
struct SizeThreshold {
    double min_length, max_length;
    double min_width,  max_width;
    double min_height, max_height;
    std::string label;
};

class LidarShapeEstimationNode {
public:
    LidarShapeEstimationNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh), pnh_(pnh)
    {
        // ---- 从 YAML 读取话题参数 ----
        pnh_.param<std::string>("input_topic",  input_topic_,  "/detection/lidar_detector/objects");
        pnh_.param<std::string>("output_topic", output_topic_, "/detection/lidar_detector/objects_obb");

        // ---- 从 YAML 读取 debug 参数 ----
        pnh_.param<bool>("debug", debug_, false);

        // ---- 从 YAML 读取 OBB 参数 ----
        pnh_.param<int>("min_cluster_size_for_obb", min_cluster_size_for_obb_, 4);
        pnh_.param<bool>("use_pca", use_pca_, true);

        // ---- 从 YAML 读取分类参数 ----
        loadClassificationParams();

        // ---- 发布者 ----
        pub_objects_ = nh_.advertise<autoware_msgs::DetectedObjectArray>(output_topic_, 10);
        if (debug_) {
            pub_markers_ = nh_.advertise<visualization_msgs::MarkerArray>(
                "/detection/lidar_detector/obb_markers", 10);
        }

        pub_metrics_ = nh_.advertise<lidar_pipeline_monitor::PipelineMetrics>("/pipeline/metrics", 100);

        // ---- 订阅者 ----
        sub_objects_ = nh_.subscribe(input_topic_, 10,
            &LidarShapeEstimationNode::objectsCallback, this);

        ROS_INFO("\033[1;32m[Lidar Shape Estimation] Node initialized.\033[0m");
        ROS_INFO("  input_topic:  %s", input_topic_.c_str());
        ROS_INFO("  output_topic: %s", output_topic_.c_str());
        ROS_INFO("  debug:        %s", debug_ ? "true" : "false");
        ROS_INFO("  min_cluster_size_for_obb: %d", min_cluster_size_for_obb_);
        ROS_INFO("  use_pca: %s", use_pca_ ? "true" : "false");
        ROS_INFO("  classification rules: %lu", thresholds_.size());
        for (const auto& t : thresholds_) {
            ROS_INFO("    [%s] L:[%.1f,%.1f] W:[%.1f,%.1f] H:[%.1f,%.1f]",
                t.label.c_str(),
                t.min_length, t.max_length,
                t.min_width, t.max_width,
                t.min_height, t.max_height);
        }
    }

private:
    ros::NodeHandle nh_, pnh_;
    ros::Publisher  pub_objects_;
    ros::Publisher  pub_markers_;
    ros::Subscriber sub_objects_;

    ros::Publisher pub_metrics_; 

    // 话题参数
    std::string input_topic_;
    std::string output_topic_;

    // debug 参数
    bool debug_;

    // OBB 参数
    int  min_cluster_size_for_obb_;
    bool use_pca_;

    // 分类阈值 (按优先级排序, 第一个匹配的生效)
    std::vector<SizeThreshold> thresholds_;

    // 默认标签
    std::string default_label_;

    // ============== 从 YAML 加载分类参数 ==============
    void loadClassificationParams()
    {
        pnh_.param<std::string>("default_label", default_label_, "unknown");

        // 定义分类规则: label, min/max length, min/max width, min/max height
        // 优先级: 从上到下, 第一个匹配的生效
        // 注意: length >= width (已标准化)

        // ---- Car ----
        if (pnh_.hasParam("classification/car")) {
            SizeThreshold t;
            t.label = "car";
            pnh_.param<double>("classification/car/min_length", t.min_length, 3.0);
            pnh_.param<double>("classification/car/max_length", t.max_length, 5.5);
            pnh_.param<double>("classification/car/min_width",  t.min_width,  1.2);
            pnh_.param<double>("classification/car/max_width",  t.max_width,  2.5);
            pnh_.param<double>("classification/car/min_height", t.min_height, 1.0);
            pnh_.param<double>("classification/car/max_height", t.max_height, 2.2);
            thresholds_.push_back(t);
        } else {
            // 默认 car 规则
            thresholds_.push_back({3.0, 5.5, 1.2, 2.5, 1.0, 2.2, "car"});
        }

        // ---- Truck ----
        if (pnh_.hasParam("classification/truck")) {
            SizeThreshold t;
            t.label = "truck";
            pnh_.param<double>("classification/truck/min_length", t.min_length, 5.5);
            pnh_.param<double>("classification/truck/max_length", t.max_length, 15.0);
            pnh_.param<double>("classification/truck/min_width",  t.min_width,  2.0);
            pnh_.param<double>("classification/truck/max_width",  t.max_width,  3.5);
            pnh_.param<double>("classification/truck/min_height", t.min_height, 2.0);
            pnh_.param<double>("classification/truck/max_height", t.max_height, 4.5);
            thresholds_.push_back(t);
        } else {
            thresholds_.push_back({5.5, 15.0, 2.0, 3.5, 2.0, 4.5, "truck"});
        }

        // ---- Bus ----
        if (pnh_.hasParam("classification/bus")) {
            SizeThreshold t;
            t.label = "bus";
            pnh_.param<double>("classification/bus/min_length", t.min_length, 8.0);
            pnh_.param<double>("classification/bus/max_length", t.max_length, 15.0);
            pnh_.param<double>("classification/bus/min_width",  t.min_width,  2.3);
            pnh_.param<double>("classification/bus/max_width",  t.max_width,  3.0);
            pnh_.param<double>("classification/bus/min_height", t.min_height, 2.5);
            pnh_.param<double>("classification/bus/max_height", t.max_height, 4.0);
            thresholds_.push_back(t);
        } else {
            thresholds_.push_back({8.0, 15.0, 2.3, 3.0, 2.5, 4.0, "bus"});
        }

        // ---- Bicycle ----
        if (pnh_.hasParam("classification/bicycle")) {
            SizeThreshold t;
            t.label = "bicycle";
            pnh_.param<double>("classification/bicycle/min_length", t.min_length, 1.2);
            pnh_.param<double>("classification/bicycle/max_length", t.max_length, 2.2);
            pnh_.param<double>("classification/bicycle/min_width",  t.min_width,  0.3);
            pnh_.param<double>("classification/bicycle/max_width",  t.max_width,  1.0);
            pnh_.param<double>("classification/bicycle/min_height", t.min_height, 0.8);
            pnh_.param<double>("classification/bicycle/max_height", t.max_height, 2.0);
            thresholds_.push_back(t);
        } else {
            thresholds_.push_back({1.2, 2.2, 0.3, 1.0, 0.8, 2.0, "bicycle"});
        }

        // ---- Person ----
        if (pnh_.hasParam("classification/person")) {
            SizeThreshold t;
            t.label = "person";
            pnh_.param<double>("classification/person/min_length", t.min_length, 0.3);
            pnh_.param<double>("classification/person/max_length", t.max_length, 1.0);
            pnh_.param<double>("classification/person/min_width",  t.min_width,  0.3);
            pnh_.param<double>("classification/person/max_width",  t.max_width,  0.8);
            pnh_.param<double>("classification/person/min_height", t.min_height, 1.0);
            pnh_.param<double>("classification/person/max_height", t.max_height, 2.2);
            thresholds_.push_back(t);
        } else {
            thresholds_.push_back({0.3, 1.0, 0.3, 0.8, 1.0, 2.2, "person"});
        }

        // ---- Box ----
        if (pnh_.hasParam("classification/box")) {
            SizeThreshold t;
            t.label = "box";
            pnh_.param<double>("classification/box/min_length", t.min_length, 0.1);
            pnh_.param<double>("classification/box/max_length", t.max_length, 3.0);
            pnh_.param<double>("classification/box/min_width",  t.min_width,  0.1);
            pnh_.param<double>("classification/box/max_width",  t.max_width,  3.0);
            pnh_.param<double>("classification/box/min_height", t.min_height, 0.1);
            pnh_.param<double>("classification/box/max_height", t.max_height, 1.5);
            thresholds_.push_back(t);
        } else {
            thresholds_.push_back({0.1, 3.0, 0.1, 3.0, 0.1, 1.5, "box"});
        }
    }

    // ============== 根据 OBB 尺寸分类标签 ==============
    std::string classifyBySize(double length, double width, double height)
    {
        for (const auto& t : thresholds_) {
            if (length >= t.min_length && length <= t.max_length &&
                width  >= t.min_width  && width  <= t.max_width &&
                height >= t.min_height && height <= t.max_height)
            {
                return t.label;
            }
        }
        return default_label_;
    }

    // ============== 根据标签获取颜色 ==============
    void getLabelColor(const std::string& label, float& r, float& g, float& b)
    {
        if (label == "car")       { r = 0.0f; g = 0.8f; b = 0.2f; }      // 绿色
        else if (label == "truck") { r = 0.8f; g = 0.4f; b = 0.0f; }     // 橙色
        else if (label == "bus")   { r = 0.8f; g = 0.0f; b = 0.8f; }     // 紫色
        else if (label == "person") { r = 1.0f; g = 0.2f; b = 0.2f; }    // 红色
        else if (label == "bicycle") { r = 0.2f; g = 0.6f; b = 1.0f; }   // 蓝色
        else if (label == "box")   { r = 1.0f; g = 1.0f; b = 0.0f; }     // 黄色
        else                       { r = 0.5f; g = 0.5f; b = 0.5f; }     // 灰色 (unknown)
    }

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

        // 4. 计算 OBB 尺寸
        Eigen::Vector3f extent = max_proj - min_proj;
        obb_dimensions = extent;

        // 5. 计算 OBB 中心
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

        obb_center = Eigen::Vector3f(position_OBB.x, position_OBB.y, position_OBB.z);
        obb_dimensions = Eigen::Vector3f(
            max_point_OBB.x - min_point_OBB.x,
            max_point_OBB.y - min_point_OBB.y,
            max_point_OBB.z - min_point_OBB.z);
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

    // ============== 创建 OBB 半透明填充 Marker ==============
    visualization_msgs::Marker createOBBCubeMarker(
        const std_msgs::Header& header,
        int id,
        const geometry_msgs::Pose& pose,
        const geometry_msgs::Vector3& dimensions,
        float r, float g, float b)
    {
        visualization_msgs::Marker marker;
        marker.header = header;
        marker.ns = "obb_box";
        marker.id = id;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose = pose;

        marker.scale.x = dimensions.x;
        marker.scale.y = dimensions.y;
        marker.scale.z = dimensions.z;

        // 半透明, 颜色由标签决定
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 0.25f;

        marker.lifetime = ros::Duration(0.2);

        return marker;
    }

    // ============== 创建 OBB 线框 (LINE_LIST) Marker ==============
    visualization_msgs::Marker createOBBLineMarker(
        const std_msgs::Header& header,
        int id,
        const geometry_msgs::Pose& pose,
        const geometry_msgs::Vector3& dimensions,
        float r, float g, float b)
    {
        visualization_msgs::Marker marker;
        marker.header = header;
        marker.ns = "obb_wireframe";
        marker.id = id;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose = pose;

        marker.scale.x = 0.05;

        // 颜色由标签决定
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0f;

        // OBB 8 个顶点 (相对于 OBB 中心, 在 OBB 局部坐标系下)
        double hx = dimensions.x / 2.0;
        double hy = dimensions.y / 2.0;
        double hz = dimensions.z / 2.0;

        geometry_msgs::Point p[8];
        p[0].x = -hx; p[0].y = -hy; p[0].z = -hz;
        p[1].x =  hx; p[1].y = -hy; p[1].z = -hz;
        p[2].x =  hx; p[2].y =  hy; p[2].z = -hz;
        p[3].x = -hx; p[3].y =  hy; p[3].z = -hz;
        p[4].x = -hx; p[4].y = -hy; p[4].z =  hz;
        p[5].x =  hx; p[5].y = -hy; p[5].z =  hz;
        p[6].x =  hx; p[6].y =  hy; p[6].z =  hz;
        p[7].x = -hx; p[7].y =  hy; p[7].z =  hz;

        // 底面 4 条边
        marker.points.push_back(p[0]); marker.points.push_back(p[1]);
        marker.points.push_back(p[1]); marker.points.push_back(p[2]);
        marker.points.push_back(p[2]); marker.points.push_back(p[3]);
        marker.points.push_back(p[3]); marker.points.push_back(p[0]);

        // 顶面 4 条边
        marker.points.push_back(p[4]); marker.points.push_back(p[5]);
        marker.points.push_back(p[5]); marker.points.push_back(p[6]);
        marker.points.push_back(p[6]); marker.points.push_back(p[7]);
        marker.points.push_back(p[7]); marker.points.push_back(p[4]);

        // 4 条竖直边
        marker.points.push_back(p[0]); marker.points.push_back(p[4]);
        marker.points.push_back(p[1]); marker.points.push_back(p[5]);
        marker.points.push_back(p[2]); marker.points.push_back(p[6]);
        marker.points.push_back(p[3]); marker.points.push_back(p[7]);

        marker.lifetime = ros::Duration(0.2);

        return marker;
    }

    // ============== 创建标签文本 Marker ==============
    visualization_msgs::Marker createLabelTextMarker(
        const std_msgs::Header& header,
        int id,
        const geometry_msgs::Point& position,
        const std::string& label,
        double height,
        float r, float g, float b)
    {
        visualization_msgs::Marker marker;
        marker.header = header;
        marker.ns = "obb_label";
        marker.id = id;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;

        // 文本显示在 OBB 顶部上方
        marker.pose.position.x = position.x;
        marker.pose.position.y = position.y;
        marker.pose.position.z = position.z + height / 2.0 + 0.3;
        marker.pose.orientation.w = 1.0;

        marker.scale.z = 0.5;  // 文字高度

        // 颜色由标签决定
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0f;

        // 显示标签
        marker.text = label;

        marker.lifetime = ros::Duration(0.2);

        return marker;
    }

    // ============== 创建尺寸文本 Marker ==============
    visualization_msgs::Marker createSizeTextMarker(
        const std_msgs::Header& header,
        int id,
        const geometry_msgs::Point& position,
        const geometry_msgs::Vector3& dimensions,
        float r, float g, float b)
    {
        visualization_msgs::Marker marker;
        marker.header = header;
        marker.ns = "obb_size";
        marker.id = id;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = position.x;
        marker.pose.position.y = position.y;
        marker.pose.position.z = position.z + dimensions.z / 2.0 + 0.7;
        marker.pose.orientation.w = 1.0;

        marker.scale.z = 0.35;

        // 白色文本
        marker.color.r = 1.0f;
        marker.color.g = 1.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0f;

        std::ostringstream oss;
        oss.setf(std::ios::fixed);
        oss.precision(2);
        oss << dimensions.x << "x" << dimensions.y << "x" << dimensions.z << "m";
        marker.text = oss.str();

        marker.lifetime = ros::Duration(0.2);

        return marker;
    }

    // ============== 主回调 ==============
    void objectsCallback(const autoware_msgs::DetectedObjectArray::ConstPtr& msg) {

        ros::Time cb_start = ros::Time::now(); // 【新增头】
        // 即使没有 objects 订阅者, 如果 debug 模式开启且有 markers 订阅者也要处理
        bool has_object_sub = (pub_objects_.getNumSubscribers() > 0);
        bool has_marker_sub = debug_ && (pub_markers_.getNumSubscribers() > 0);

        if (!has_object_sub && !has_marker_sub) {
            return;
        }

        autoware_msgs::DetectedObjectArray output_array;
        output_array.header = msg->header;

        visualization_msgs::MarkerArray marker_array;

        int obb_count = 0;
        int marker_id = 0;

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
            double length = obb_dimensions[0];
            double width  = obb_dimensions[1];
            double height = obb_dimensions[2];

            // 标准化: length(x) >= width(y)
            if (width > length) {
                std::swap(length, width);
                tf::Quaternion q_orig;
                tf::quaternionMsgToTF(new_obj.pose.orientation, q_orig);
                tf::Quaternion q_rot(tf::Vector3(0, 0, 1), M_PI / 2.0);
                tf::Quaternion q_new = q_orig * q_rot;
                tf::quaternionTFToMsg(q_new, new_obj.pose.orientation);
            }

            new_obj.dimensions.x = length;
            new_obj.dimensions.y = width;
            new_obj.dimensions.z = height;

            // ---- 根据尺寸分类标签 ----
            std::string label = classifyBySize(length, width, height);
            new_obj.label = label;

            obb_count++;
            output_array.objects.push_back(new_obj);

            // ---- debug: 生成 RViz Marker ----
            if (has_marker_sub) {
                // 根据标签获取颜色
                float cr, cg, cb;
                getLabelColor(label, cr, cg, cb);

                // OBB 半透明填充
                visualization_msgs::Marker cube_marker = createOBBCubeMarker(
                    msg->header, marker_id, new_obj.pose, new_obj.dimensions, cr, cg, cb);
                marker_array.markers.push_back(cube_marker);

                // OBB 线框
                visualization_msgs::Marker line_marker = createOBBLineMarker(
                    msg->header, marker_id, new_obj.pose, new_obj.dimensions, cr, cg, cb);
                marker_array.markers.push_back(line_marker);

                // 标签文本 (显示分类标签, 如 car, person, box 等)
                visualization_msgs::Marker label_marker = createLabelTextMarker(
                    msg->header, marker_id, new_obj.pose.position, label, new_obj.dimensions.z, cr, cg, cb);
                marker_array.markers.push_back(label_marker);

                // 尺寸文本
                visualization_msgs::Marker size_marker = createSizeTextMarker(
                    msg->header, marker_id, new_obj.pose.position, new_obj.dimensions, cr, cg, cb);
                marker_array.markers.push_back(size_marker);

                marker_id++;
            }
        }

        // ---- 发布结果 ----
        if (has_object_sub) {
            pub_objects_.publish(output_array);
        }

        if (has_marker_sub) {
            pub_markers_.publish(marker_array);
        }

        // 【新增尾】
        ros::Time cb_end = ros::Time::now();
        lidar_pipeline_monitor::PipelineMetrics metric;
        metric.header.stamp = msg->header.stamp; 
        metric.node_name = "6_shape";
        metric.transmission_delay = (cb_start - msg->header.stamp).toSec() * 1000.0;
        metric.processing_time = (cb_end - cb_start).toSec() * 1000.0;
        metric.total_latency = (cb_end - msg->header.stamp).toSec() * 1000.0;
        pub_metrics_.publish(metric);

        ROS_INFO_THROTTLE(2.0,
            "[Shape Estimation] Input objects: %lu, OBB computed: %d, debug: %s",
            msg->objects.size(), obb_count, debug_ ? "ON" : "OFF");
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