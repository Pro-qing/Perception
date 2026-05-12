#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

class LidarDownsampleNode {
public:
    LidarDownsampleNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh) {
        // ---- 从 YAML 读取参数 ----
        pnh_.param<std::string>("input_topic", input_topic_, "/points_raw");
        pnh_.param<std::string>("output_topic", output_topic_, "/points_downsampled");
        pnh_.param<double>("voxel_leaf_size", voxel_leaf_size_, 0.1);

        // ---- 发布者 ----
        pub_downsampled_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 10);

        // ---- 订阅者 ----
        sub_points_ = nh_.subscribe(input_topic_, 10, &LidarDownsampleNode::pointCloudCallback, this);

        ROS_INFO("\033[1;32m[Lidar Downsample] Node initialized. input=%s, output=%s, leaf_size=%.3f\033[0m",
                 input_topic_.c_str(), output_topic_.c_str(), voxel_leaf_size_);
    }

private:
    ros::NodeHandle nh_, pnh_;
    ros::Publisher pub_downsampled_;
    ros::Subscriber sub_points_;

    std::string input_topic_;
    std::string output_topic_;
    double voxel_leaf_size_;

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

        // 体素网格降采样
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::VoxelGrid<pcl::PointXYZI> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
        vg.filter(*cloud_filtered);

        // 转换回 ROS 消息并发布
        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(*cloud_filtered, output_msg);
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