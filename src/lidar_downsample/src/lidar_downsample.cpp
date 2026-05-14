#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <lidar_pipeline_monitor/PipelineMetrics.h>

#include <vector>
#include <cmath>

struct Point2D {
    double x, y;
};

class LidarDownsampleNode {
public:
    LidarDownsampleNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh) {
        // ---- 从 YAML 读取话题参数 ----
        pnh_.param<std::string>("input_topic", input_topic_, "/points_raw");
        pnh_.param<std::string>("output_topic", output_topic_, "/points_downsampled");

        // ---- 从 YAML 读取 VoxelGrid 体素滤波参数 ----
        pnh_.param<double>("voxel_grid/leaf_size_x", leaf_size_x_, 0.1);
        pnh_.param<double>("voxel_grid/leaf_size_y", leaf_size_y_, 0.1);
        pnh_.param<double>("voxel_grid/leaf_size_z", leaf_size_z_, 0.1);
        pnh_.param<int>("voxel_grid/min_points_per_voxel", min_points_per_voxel_, 1);
        pnh_.param<bool>("voxel_grid/downsample_all_data", downsample_all_data_, true);

        // ---- 从 YAML 读取车身过滤 (Body Filter) 参数 ----
        pnh_.param<bool>("body_filter/enable", body_filter_enable_, false);
        pnh_.param<double>("body_filter/min_z", body_min_z_, -2.0);
        pnh_.param<double>("body_filter/max_z", body_max_z_, 0.0);

        // 读取多边形顶点
        loadBodyPolygon();

        // 读取 Marker 参数
        pnh_.param<bool>("body_filter/publish_marker", publish_marker_, false);
        pnh_.param<std::string>("body_filter/marker_topic", marker_topic_, "/car");
        pnh_.param<double>("body_filter/marker_color_r", marker_r_, 0.0);
        pnh_.param<double>("body_filter/marker_color_g", marker_g_, 1.0);
        pnh_.param<double>("body_filter/marker_color_b", marker_b_, 0.0);
        pnh_.param<double>("body_filter/marker_color_a", marker_a_, 0.5);

        // ---- 从 YAML 读取 CropBox 裁剪框参数 ----
        pnh_.param<bool>("crop_box/enable", crop_box_enable_, false);
        pnh_.param<double>("crop_box/min_x", crop_min_x_, -50.0);
        pnh_.param<double>("crop_box/max_x", crop_max_x_, 50.0);
        pnh_.param<double>("crop_box/min_y", crop_min_y_, -50.0);
        pnh_.param<double>("crop_box/max_y", crop_max_y_, 50.0);
        pnh_.param<double>("crop_box/min_z", crop_min_z_, -3.0);
        pnh_.param<double>("crop_box/max_z", crop_max_z_, 3.0);
        pnh_.param<bool>("crop_box/negative", crop_negative_, false);

        // ---- 从 YAML 读取高度过滤参数 ----
        pnh_.param<bool>("height_filter/enable", height_filter_enable_, false);
        pnh_.param<double>("height_filter/min_height", min_height_, -2.0);
        pnh_.param<double>("height_filter/max_height", max_height_, 2.0);

        // ---- 发布者 ----
        pub_downsampled_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 10);

        pub_metrics_ = nh_.advertise<lidar_pipeline_monitor::PipelineMetrics>("/pipeline/metrics", 100);

        if (publish_marker_ && body_filter_enable_) {
            pub_marker_array_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_topic_, 1, true);
            publishBodyMarker();
        }

        // ---- 订阅者 ----
        sub_points_ = nh_.subscribe(input_topic_, 10, &LidarDownsampleNode::pointCloudCallback, this);

        ROS_INFO("\033[1;32m[Lidar Downsample] Node initialized.\033[0m");
        ROS_INFO("  input_topic:       %s", input_topic_.c_str());
        ROS_INFO("  output_topic:      %s", output_topic_.c_str());
        ROS_INFO("  voxel leaf_size:   (%.3f, %.3f, %.3f)", leaf_size_x_, leaf_size_y_, leaf_size_z_);
        ROS_INFO("  min_points_voxel:  %d", min_points_per_voxel_);
        ROS_INFO("  downsample_all:    %s", downsample_all_data_ ? "true" : "false");
        ROS_INFO("  crop_box enable:   %s", crop_box_enable_ ? "true" : "false");
        if (crop_box_enable_) {
            ROS_INFO("  crop_box range:    x[%.1f, %.1f] y[%.1f, %.1f] z[%.1f, %.1f] negative=%s",
                     crop_min_x_, crop_max_x_, crop_min_y_, crop_max_y_,
                     crop_min_z_, crop_max_z_, crop_negative_ ? "true" : "false");
        }
        ROS_INFO("  body_filter:       %s", body_filter_enable_ ? "true" : "false");
        if (body_filter_enable_) {
            ROS_INFO("  body polygon:      %lu points, z[%.1f, %.1f]",
                     body_polygon_.size(), body_min_z_, body_max_z_);
            for (size_t i = 0; i < body_polygon_.size(); ++i) {
                ROS_INFO("    [%lu] x=%.2f, y=%.2f", i, body_polygon_[i].x, body_polygon_[i].y);
            }
        }
        ROS_INFO("  height_filter:     %s", height_filter_enable_ ? "true" : "false");
        if (height_filter_enable_) {
            ROS_INFO("  height range:      [%.2f, %.2f]", min_height_, max_height_);
        }
    }

private:
    ros::NodeHandle nh_, pnh_;
    ros::Publisher pub_downsampled_;
    ros::Publisher pub_marker_array_;
    ros::Subscriber sub_points_;

    ros::Publisher pub_metrics_; 

    // 话题参数
    std::string input_topic_;
    std::string output_topic_;

    // VoxelGrid 参数
    double leaf_size_x_, leaf_size_y_, leaf_size_z_;
    int min_points_per_voxel_;
    bool downsample_all_data_;

    // Body Filter (车身过滤) 参数
    bool body_filter_enable_;
    double body_min_z_, body_max_z_;
    std::vector<Point2D> body_polygon_;

    // Marker 参数
    bool publish_marker_;
    std::string marker_topic_;
    double marker_r_, marker_g_, marker_b_, marker_a_;

    // CropBox 参数
    bool crop_box_enable_;
    double crop_min_x_, crop_max_x_;
    double crop_min_y_, crop_max_y_;
    double crop_min_z_, crop_max_z_;
    bool crop_negative_;

    // 高度过滤参数
    bool height_filter_enable_;
    double min_height_, max_height_;

    // ============== 加载车身多边形顶点 ==============
    void loadBodyPolygon() {
        XmlRpc::XmlRpcValue polygon_list;
        if (!pnh_.getParam("body_filter/polygon", polygon_list)) {
            ROS_WARN("[Lidar Downsample] No body_filter/polygon found. Using empty polygon.");
            return;
        }

        if (polygon_list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            ROS_WARN("[Lidar Downsample] body_filter/polygon is not an array.");
            return;
        }

        body_polygon_.clear();
        for (int i = 0; i < polygon_list.size(); ++i) {
            XmlRpc::XmlRpcValue& pt = polygon_list[i];
            Point2D p;
            p.x = static_cast<double>(pt["x"]);
            p.y = static_cast<double>(pt["y"]);
            body_polygon_.push_back(p);
        }
    }

    // ============== Point-in-Polygon 判断 (射线法) ==============
    bool isPointInsidePolygon(double px, double py) const {
        int n = body_polygon_.size();
        if (n < 3) return false;

        bool inside = false;
        for (int i = 0, j = n - 1; i < n; j = i++) {
            double xi = body_polygon_[i].x, yi = body_polygon_[i].y;
            double xj = body_polygon_[j].x, yj = body_polygon_[j].y;

            if (((yi > py) != (yj > py)) &&
                (px < (xj - xi) * (py - yi) / (yj - yi) + xi)) {
                inside = !inside;
            }
        }
        return inside;
    }

    // ============== 发布车身 MarkerArray ==============
    void publishBodyMarker() {
        visualization_msgs::MarkerArray marker_array;

        // ---- Marker 1: 线框 (LINE_LIST) - 底面、顶面、竖直连接线 ----
        visualization_msgs::Marker line_marker;
        line_marker.header.frame_id = "velodyne";
        line_marker.header.stamp = ros::Time::now();
        line_marker.ns = "body_filter";
        line_marker.id = 0;
        line_marker.type = visualization_msgs::Marker::LINE_LIST;
        line_marker.action = visualization_msgs::Marker::ADD;

        line_marker.scale.x = 0.05;  // 线宽

        line_marker.color.r = marker_r_;
        line_marker.color.g = marker_g_;
        line_marker.color.b = marker_b_;
        line_marker.color.a = marker_a_;

        line_marker.pose.orientation.w = 1.0;

        int n = body_polygon_.size();
        for (int i = 0; i < n; ++i) {
            int j = (i + 1) % n;
            geometry_msgs::Point p1_bot, p2_bot, p1_top, p2_top;

            p1_bot.x = body_polygon_[i].x;
            p1_bot.y = body_polygon_[i].y;
            p1_bot.z = body_min_z_;
            p2_bot.x = body_polygon_[j].x;
            p2_bot.y = body_polygon_[j].y;
            p2_bot.z = body_min_z_;

            p1_top.x = body_polygon_[i].x;
            p1_top.y = body_polygon_[i].y;
            p1_top.z = body_max_z_;
            p2_top.x = body_polygon_[j].x;
            p2_top.y = body_polygon_[j].y;
            p2_top.z = body_max_z_;

            // 底面线段
            line_marker.points.push_back(p1_bot);
            line_marker.points.push_back(p2_bot);

            // 顶面线段
            line_marker.points.push_back(p1_top);
            line_marker.points.push_back(p2_top);

            // 竖直连接线
            line_marker.points.push_back(p1_bot);
            line_marker.points.push_back(p1_top);
        }
        marker_array.markers.push_back(line_marker);

        // ---- Marker 2: 填充面 (TRIANGLE_LIST) - 侧面 ----
        visualization_msgs::Marker tri_marker;
        tri_marker.header.frame_id = "velodyne";
        tri_marker.header.stamp = ros::Time::now();
        tri_marker.ns = "body_filter_triangles";
        tri_marker.id = 1;
        tri_marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
        tri_marker.action = visualization_msgs::Marker::ADD;

        tri_marker.scale.x = 1.0;
        tri_marker.scale.y = 1.0;
        tri_marker.scale.z = 1.0;

        tri_marker.color.r = marker_r_;
        tri_marker.color.g = marker_g_;
        tri_marker.color.b = marker_b_;
        tri_marker.color.a = 0.3f;  // 半透明

        tri_marker.pose.orientation.w = 1.0;

        for (int i = 0; i < n; ++i) {
            int j = (i + 1) % n;
            geometry_msgs::Point p1_bot, p2_bot, p1_top, p2_top;

            p1_bot.x = body_polygon_[i].x;
            p1_bot.y = body_polygon_[i].y;
            p1_bot.z = body_min_z_;
            p2_bot.x = body_polygon_[j].x;
            p2_bot.y = body_polygon_[j].y;
            p2_bot.z = body_min_z_;
            p1_top.x = body_polygon_[i].x;
            p1_top.y = body_polygon_[i].y;
            p1_top.z = body_max_z_;
            p2_top.x = body_polygon_[j].x;
            p2_top.y = body_polygon_[j].y;
            p2_top.z = body_max_z_;

            // 每个侧面由两个三角形组成
            tri_marker.points.push_back(p1_bot);
            tri_marker.points.push_back(p2_bot);
            tri_marker.points.push_back(p1_top);

            tri_marker.points.push_back(p1_top);
            tri_marker.points.push_back(p2_bot);
            tri_marker.points.push_back(p2_top);
        }
        marker_array.markers.push_back(tri_marker);

        // 使用 latched 发布，只发布一次即可
        pub_marker_array_.publish(marker_array);
        ROS_INFO("[Lidar Downsample] Body MarkerArray published on %s (%d markers)",
                 marker_topic_.c_str(), (int)marker_array.markers.size());
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        ros::Time cb_start = ros::Time::now(); // 【新增头】
        if (pub_downsampled_.getNumSubscribers() == 0) {
            return;
        }

        // 转换为 PCL 点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->empty()) {
            ROS_WARN_THROTTLE(5.0, "[Lidar Downsample] Received empty cloud, skipping.");
            return;
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_processed(new pcl::PointCloud<pcl::PointXYZI>());
        *cloud_processed = *cloud;

        // ---- Step 1: 车身过滤 Body Filter (可选, 点到多边形) ----
        if (body_filter_enable_ && body_polygon_.size() >= 3) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_body_filtered(new pcl::PointCloud<pcl::PointXYZI>());
            cloud_body_filtered->reserve(cloud_processed->size());

            for (const auto& pt : cloud_processed->points) {
                // Z 范围检查
                if (pt.z < body_min_z_ || pt.z > body_max_z_) {
                    cloud_body_filtered->points.push_back(pt);
                    continue;
                }
                // XY 多边形检查: 在多边形内则移除 (不加入输出)
                if (!isPointInsidePolygon(pt.x, pt.y)) {
                    cloud_body_filtered->points.push_back(pt);
                }
            }

            cloud_body_filtered->width = cloud_body_filtered->points.size();
            cloud_body_filtered->height = 1;
            cloud_body_filtered->is_dense = true;
            cloud_processed = cloud_body_filtered;

            if (cloud_processed->empty()) {
                ROS_WARN_THROTTLE(5.0, "[Lidar Downsample] Cloud empty after body_filter, skipping.");
                return;
            }
        }

        // ---- Step 2: CropBox 范围裁剪 (可选) ----
        if (crop_box_enable_) {
            pcl::CropBox<pcl::PointXYZI> crop;
            crop.setInputCloud(cloud_processed);
            crop.setMin(Eigen::Vector4f(crop_min_x_, crop_min_y_, crop_min_z_, 1.0));
            crop.setMax(Eigen::Vector4f(crop_max_x_, crop_max_y_, crop_max_z_, 1.0));
            crop.setNegative(crop_negative_);
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cropped(new pcl::PointCloud<pcl::PointXYZI>());
            crop.filter(*cloud_cropped);
            cloud_processed = cloud_cropped;

            if (cloud_processed->empty()) {
                ROS_WARN_THROTTLE(5.0, "[Lidar Downsample] Cloud empty after crop_box, skipping.");
                return;
            }
        }

        // ---- Step 3: 高度过滤 (可选) ----
        if (height_filter_enable_) {
            pcl::PassThrough<pcl::PointXYZI> pass;
            pass.setInputCloud(cloud_processed);
            pass.setFilterFieldName("z");
            pass.setFilterLimits(min_height_, max_height_);
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());
            pass.filter(*cloud_filtered);
            cloud_processed = cloud_filtered;

            if (cloud_processed->empty()) {
                ROS_WARN_THROTTLE(5.0, "[Lidar Downsample] Cloud empty after height_filter, skipping.");
                return;
            }
        }

        // ---- Step 4: VoxelGrid 体素降采样 ----
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::VoxelGrid<pcl::PointXYZI> vg;
        vg.setInputCloud(cloud_processed);
        vg.setLeafSize(leaf_size_x_, leaf_size_y_, leaf_size_z_);
        vg.setMinimumPointsNumberPerVoxel(static_cast<unsigned int>(min_points_per_voxel_));
        vg.setDownsampleAllData(downsample_all_data_);
        vg.filter(*cloud_downsampled);

        if (cloud_downsampled->empty()) {
            ROS_WARN_THROTTLE(5.0, "[Lidar Downsample] Cloud empty after voxel_grid, skipping.");
            return;
        }

        // 转换回 ROS 消息并发布
        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(*cloud_downsampled, output_msg);
        output_msg.header.stamp    = msg->header.stamp;
        output_msg.header.frame_id = msg->header.frame_id;
        pub_downsampled_.publish(output_msg);

         // 【新增尾】
        ros::Time cb_end = ros::Time::now();
        lidar_pipeline_monitor::PipelineMetrics metric;
        metric.header.stamp = msg->header.stamp; 
        metric.node_name = "2_downsample";
        metric.transmission_delay = (cb_start - msg->header.stamp).toSec() * 1000.0;
        metric.processing_time = (cb_end - cb_start).toSec() * 1000.0;
        metric.total_latency = (cb_end - msg->header.stamp).toSec() * 1000.0;
        pub_metrics_.publish(metric);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_downsample_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    LidarDownsampleNode node(nh, pnh);

    ros::spin();

    return 0;
}