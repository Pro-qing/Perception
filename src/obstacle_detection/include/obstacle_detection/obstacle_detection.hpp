#ifndef __OBSTACLE_DETECTION_HPP_
#define __OBSTACLE_DETECTION_HPP_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <visualization_msgs/MarkerArray.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <std_msgs/UInt32.h>
#include <autoware_msgs/KeyPointArray.h>
#include <autoware_remove_msgs/State.h>
#include <autoware_msgs/ElevatorInfo.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <memory>

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloud;



enum PointType_e {
    POINT_TYPE_ELEVATOR = 0,        // 电梯点
    POINT_TYPE_GARAGE = 1,         // 库位
};
struct TargetPoint {
    geometry_msgs::PoseStamped map_pose;
    PointType_e type_e;
    bool enable;
};

class ObstacleDetection {
private:
    ros::NodeHandle& nh_;
    ros::NodeHandle& private_nh_;
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2>;
    message_filters::Subscriber<sensor_msgs::PointCloud2> mid_cloud_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> tip_cloud_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    ros::Subscriber keyPointPath_sub_;
    ros::Subscriber feedback_status_sub_;
    ros::Subscriber elevator_info_sub_;
    ros::Subscriber lqr_dire_sub_;
    ros::Subscriber floor_set_sub_;
    ros::Subscriber current_pose_sub_;
    ros::Publisher ground_pub_;
    ros::Publisher obstacle_pub_;
    ros::Publisher cluster_pub_;
    ros::Publisher marker_pub_;
    ros::Publisher bbox_pub_;
    ros::Publisher calibration_pub_;
    ros::Publisher target_region_pub_;
    ros::Publisher target_region_cloud_pub_;
    ros::Publisher obstacle_detection_pub_;
    ros::Publisher fused_cloud_pub_;
    
    ros::Timer publish_timer_;
    tf::TransformListener tf_listener_;
    
    // Parameters
    std::string target_frame_;
    std::string points_mid_topic_;  // points_mid topic名称
    double voxel_leaf_size_;
    double cluster_tolerance_;
    int min_cluster_size_;
    int max_cluster_size_;
    double z_axis_min_;
    double z_axis_max_;
    double ground_threshold_;
    double roi_radius_;  // ROI区域半径
    bool use_roi_filter_;
    double distance_threshold_;  // 距离阈值，小于此值时关闭enable
    int8_t elevator_control_flag_;
    int8_t floor_set_;
    TargetPoint targetPoint_;
    geometry_msgs::PoseStamped current_pose_;  // 当前位置

    // 标定参数
    double mid_x_;
    double mid_y_;  
    double mid_z_;
    double mid_roll_;
    double mid_pitch_;
    double mid_yaw_;


    // 电梯范围参数
    double elevator_min_x_, elevator_max_x_;
    double elevator_min_y_, elevator_max_y_;
    double elevator_min_z_, elevator_max_z_;

    std::vector<Eigen::Vector2f> car_vertices_;
    double car_min_z_, car_max_z_;

    // 库位范围参数
    double carports_min_x_, carports_max_x_;
    double carports_min_y_, carports_max_y_;
    double carports_min_z_, carports_max_z_;

    bool target_region_has_noise_;
    autoware_remove_msgs::State feedbackStatus_;

    std_msgs::UInt32 obstacle_detection_;
    
public:
    ObstacleDetection(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
    
private:
    struct ObstacleInfo {
        geometry_msgs::Point center;
        geometry_msgs::Vector3 dimensions;
        double distance;
        int point_count;
    };
    struct MidProcessResult {
        PointCloud::Ptr ground_cloud;
        PointCloud::Ptr filtered_cloud;
        std_msgs::Header header;
        bool valid;
    };
    void timerCallback(const ros::TimerEvent& event);
    void syncCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& mid_msg,
                           const sensor_msgs::PointCloud2::ConstPtr& tip_msg);
    bool preprocessMidCloud(const sensor_msgs::PointCloud2::ConstPtr& input_msg,
                            MidProcessResult& result);
    void keyPointPathCallback(const autoware_msgs::KeyPointArray::ConstPtr& msg);
    void feedbackStatusCallback(const autoware_remove_msgs::State::ConstPtr& msg);
    void elevatorInfoCallback(const autoware_msgs::ElevatorInfo::ConstPtr& msg);
    void currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    bool transformPointCloud(const PointCloud::Ptr& input, PointCloud::Ptr& output,
                            const std::string& source_frame, const std::string& target_frame);
    void lqrDireCallback(const std_msgs::Int8::ConstPtr& msg);
    void floorSetCallback(const std_msgs::Int8::ConstPtr& msg);

    void applyROIFilter(const PointCloud::Ptr& input, PointCloud::Ptr& output);
    
    void segmentGroundPlane(const PointCloud::Ptr& input,
                           PointCloud::Ptr& ground,
                           PointCloud::Ptr& obstacles);
    
    void performClustering(const PointCloud::Ptr& cloud,
                          std::vector<pcl::PointIndices>& cluster_indices);
    
    void processClusters(const PointCloud::Ptr& cloud,
                        const std::vector<pcl::PointIndices>& cluster_indices,
                        std::vector<ObstacleInfo>& obstacles,
                        const std_msgs::Header& header);
    
    void publishPointClouds(const PointCloud::Ptr& ground,
                           const PointCloud::Ptr& obstacles,
                           const std::vector<pcl::PointIndices>& cluster_indices,
                           const std_msgs::Header& header);
    
    void publishColoredClusters(const PointCloud::Ptr& cloud,
                               const std::vector<pcl::PointIndices>& cluster_indices,
                               const std_msgs::Header& header);
    
    void publishObstacleInfo(const std::vector<ObstacleInfo>& obstacles,
                            const std_msgs::Header& header);
    
    void setColorByDistance(std_msgs::ColorRGBA& color, double distance);
    void setLogLevel();


    PointCloud::Ptr points_mid_calibration(PointCloud::Ptr msgPtr, std_msgs::Header header);

    void checkTargetPointRegion(const PointCloud::Ptr& obstacle_cloud,
                                             const std_msgs::Header& header);

    void publishTargetRegionMarker(const std_msgs::Header& header);

    void addSingleMarkerToArray(visualization_msgs::MarkerArray& marker_array, 
                            const std_msgs::Header& header,
                            const Eigen::Vector3f& center_point,
                            const std::vector<Eigen::Vector2f>& polygon_vertices,
                            float min_z, float max_z,
                            const std::string& ns, int id,
                            float r, float g, float b, float yaw_angle);

    bool robustElevatorCheck(const std::string& data);

    geometry_msgs::Pose transformTargetPoseToVelodyne(const geometry_msgs::PoseStamped& input_pose);
};



#endif