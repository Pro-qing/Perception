#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>

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
        ROS_INFO("  height_filter:     %s", height_filter_enable_ ? "true" : "false");
        if (height_filter_enable_) {
            ROS_INFO("  height range:      [%.2f, %.2f]", min_height_, max_height_);
        }
    }

private:
    ros::NodeHandle nh_, pnh_;
    ros::Publisher pub_downsampled_;
    ros::Subscriber sub_points_;

    // 话题参数
    std::string input_topic_;
    std::string output_topic_;

    // VoxelGrid 参数
    double leaf_size_x_, leaf_size_y_, leaf_size_z_;
    int min_points_per_voxel_;
    bool downsample_all_data_;

    // CropBox 参数
    bool crop_box_enable_;
    double crop_min_x_, crop_max_x_;
    double crop_min_y_, crop_max_y_;
    double crop_min_z_, crop_max_z_;
    bool crop_negative_;

    // 高度过滤参数
    bool height_filter_enable_;
    double min_height_, max_height_;

    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
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

        // ---- Step 1: CropBox 裁剪 (可选) ----
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

        // ---- Step 2: 高度过滤 (可选) ----
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

        // ---- Step 3: VoxelGrid 体素降采样 ----
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