#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

#include <autoware_msgs/DetectedObject.h>
#include <autoware_msgs/DetectedObjectArray.h>

#include <lidar_pipeline_monitor/PipelineMetrics.h>

#include <tf/tf.h>

#include <Eigen/Dense>

#include <vector>
#include <cmath>
#include <string>
#include <sstream>
#include <map>
#include <algorithm>
#include <limits>

// ============================================================
//  Lidar Tracking & Prediction
//
//  算法原理:
//    1. 订阅 /detection/lidar_detector/objects_obb (DetectedObjectArray)
//    2. 使用卡尔曼滤波器 (Constant Velocity Model) 对每个目标进行跟踪
//    3. 使用最近邻 (Nearest Neighbor) 进行数据关联
//    4. 估计目标速度和加速度
//    5. 预测目标未来位置
//    6. 发布带速度和预测信息的 DetectedObjectArray
//    7. (debug 模式) 发布轨迹可视化 MarkerArray
// ============================================================

// ============== 卡尔曼滤波器 (匀速模型) ==============
// 状态向量: [x, y, z, vx, vy, vz]
// 观测向量: [x, y, z]
class KalmanFilterCV {
public:
    KalmanFilterCV() : initialized_(false) {}

    void initialize(const Eigen::Vector3d& position, double dt = 0.1) {
        // 状态维度 6, 观测维度 3
        state_ = Eigen::VectorXd::Zero(6);
        state_.segment<3>(0) = position;

        // 状态转移矩阵 F
        F_ = Eigen::MatrixXd::Identity(6, 6);
        F_(0, 3) = dt;
        F_(1, 4) = dt;
        F_(2, 5) = dt;

        // 观测矩阵 H
        H_ = Eigen::MatrixXd::Zero(3, 6);
        H_(0, 0) = 1.0;
        H_(1, 1) = 1.0;
        H_(2, 2) = 1.0;

        // 过程噪声协方差 Q
        double q_pos = 1.0;
        double q_vel = 10.0;
        Q_ = Eigen::MatrixXd::Zero(6, 6);
        Q_(0, 0) = q_pos;
        Q_(1, 1) = q_pos;
        Q_(2, 2) = q_pos;
        Q_(3, 3) = q_vel;
        Q_(4, 4) = q_vel;
        Q_(5, 5) = q_vel;

        // 观测噪声协方差 R
        R_ = Eigen::MatrixXd::Identity(3, 3) * 0.5;

        // 状态协方差 P
        P_ = Eigen::MatrixXd::Identity(6, 6);
        P_.block<3, 3>(0, 0) *= 1.0;
        P_.block<3, 3>(3, 3) *= 100.0;

        initialized_ = true;
    }

    void predict(double dt) {
        if (!initialized_) return;

        // 更新状态转移矩阵的时间步长
        F_(0, 3) = dt;
        F_(1, 4) = dt;
        F_(2, 5) = dt;

        state_ = F_ * state_;
        P_ = F_ * P_ * F_.transpose() + Q_;
    }

    void update(const Eigen::Vector3d& measurement) {
        if (!initialized_) return;

        Eigen::VectorXd z = measurement;
        Eigen::VectorXd y = z - H_ * state_;
        Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
        Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();

        state_ = state_ + K * y;
        P_ = (Eigen::MatrixXd::Identity(6, 6) - K * H_) * P_;
    }

    Eigen::Vector3d getPosition() const {
        return state_.segment<3>(0);
    }

    Eigen::Vector3d getVelocity() const {
        return state_.segment<3>(3);
    }

    bool isInitialized() const { return initialized_; }

private:
    bool initialized_;
    Eigen::VectorXd state_;   // [x, y, z, vx, vy, vz]
    Eigen::MatrixXd F_;       // 状态转移矩阵
    Eigen::MatrixXd H_;       // 观测矩阵
    Eigen::MatrixXd Q_;       // 过程噪声协方差
    Eigen::MatrixXd R_;       // 观测噪声协方差
    Eigen::MatrixXd P_;       // 状态协方差
};

// ============== 跟踪目标 ==============
struct TrackedObject {
    int id;                         // 唯一跟踪 ID
    std::string label;              // 类别标签
    autoware_msgs::DetectedObject last_detection;  // 最近一次检测
    KalmanFilterCV kf;              // 卡尔曼滤波器
    int age;                        // 存活帧数
    int total_visible_count;        // 总检测次数
    int consecutive_invisible;      // 连续未检测到帧数
    ros::Time last_update_time;     // 最后更新时间
    Eigen::Vector3d velocity;       // 估计速度
    Eigen::Vector3d acceleration;   // 估计加速度
    Eigen::Vector3d prev_velocity;  // 上一帧速度 (用于计算加速度)
};

class LidarTrackingPredictionNode {
public:
    LidarTrackingPredictionNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh), pnh_(pnh), next_track_id_(1)
    {
        // ---- 从 YAML 读取话题参数 ----
        pnh_.param<std::string>("input_topic",  input_topic_,  "/detection/lidar_detector/objects_obb");
        pnh_.param<std::string>("output_topic", output_topic_, "/detection/lidar_detector/objects_tracked");

        // ---- 从 YAML 读取跟踪参数 ----
        pnh_.param<double>("max_matching_distance", max_matching_distance_, 3.0);
        pnh_.param<int>("max_consecutive_invisible", max_consecutive_invisible_, 3);
        pnh_.param<int>("min_visible_count", min_visible_count_, 3);
        pnh_.param<double>("prediction_horizon", prediction_horizon_, 1.0);
        pnh_.param<double>("process_noise_pos", process_noise_pos_, 1.0);
        pnh_.param<double>("process_noise_vel", process_noise_vel_, 10.0);
        pnh_.param<double>("measurement_noise", measurement_noise_, 0.5);

        // ---- 从 YAML 读取 debug 参数 ----
        pnh_.param<bool>("debug", debug_, false);

        // ---- 发布者 ----
        pub_objects_ = nh_.advertise<autoware_msgs::DetectedObjectArray>(output_topic_, 10);
        if (debug_) {
            pub_markers_ = nh_.advertise<visualization_msgs::MarkerArray>(
                "/detection/lidar_detector/tracking_markers", 10);
            pub_predicted_ = nh_.advertise<geometry_msgs::PoseArray>(
                "/detection/lidar_detector/predicted_poses", 10);
        }

        pub_metrics_ = nh_.advertise<lidar_pipeline_monitor::PipelineMetrics>("/pipeline/metrics", 100);

        // ---- 订阅者 ----
        sub_objects_ = nh_.subscribe(input_topic_, 10,
            &LidarTrackingPredictionNode::objectsCallback, this);

        ROS_INFO("\033[1;32m[Lidar Tracking Prediction] Node initialized.\033[0m");
        ROS_INFO("  input_topic:              %s", input_topic_.c_str());
        ROS_INFO("  output_topic:             %s", output_topic_.c_str());
        ROS_INFO("  max_matching_distance:    %.2f m", max_matching_distance_);
        ROS_INFO("  max_consecutive_invisible: %d", max_consecutive_invisible_);
        ROS_INFO("  min_visible_count:        %d", min_visible_count_);
        ROS_INFO("  prediction_horizon:       %.2f s", prediction_horizon_);
        ROS_INFO("  process_noise_pos:        %.2f", process_noise_pos_);
        ROS_INFO("  process_noise_vel:        %.2f", process_noise_vel_);
        ROS_INFO("  measurement_noise:        %.2f", measurement_noise_);
        ROS_INFO("  debug:                    %s", debug_ ? "true" : "false");
    }

private:
    ros::NodeHandle nh_, pnh_;
    ros::Publisher  pub_objects_;
    ros::Publisher  pub_markers_;
    ros::Publisher  pub_predicted_;
    ros::Subscriber sub_objects_;

    ros::Publisher pub_metrics_;

    // 话题参数
    std::string input_topic_;
    std::string output_topic_;

    // 跟踪参数
    double max_matching_distance_;
    int    max_consecutive_invisible_;
    int    min_visible_count_;
    double prediction_horizon_;
    double process_noise_pos_;
    double process_noise_vel_;
    double measurement_noise_;

    // debug 参数
    bool debug_;

    // 跟踪目标列表
    std::vector<TrackedObject> tracks_;
    int next_track_id_;

    // ============== 计算两个位置之间的距离 ==============
    double distanceBetween(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
        return (a - b).norm();
    }

    // ============== 最近邻数据关联 (匈牙利算法简化版) ==============
    std::vector<int> associateDetectionsToTracks(
        const std::vector<Eigen::Vector3d>& det_positions,
        const std::vector<int>& track_indices)
    {
        int n_dets = det_positions.size();
        int n_tracks = track_indices.size();

        // 返回值: 每个检测对应的跟踪索引 (-1 表示未关联)
        std::vector<int> assignment(n_dets, -1);

        if (n_dets == 0 || n_tracks == 0) return assignment;

        // 计算代价矩阵
        Eigen::MatrixXd cost(n_dets, n_tracks);
        for (int i = 0; i < n_dets; ++i) {
            for (int j = 0; j < n_tracks; ++j) {
                cost(i, j) = distanceBetween(det_positions[i],
                    tracks_[track_indices[j]].kf.getPosition());
            }
        }

        // 贪心最近邻关联
        std::vector<bool> det_used(n_dets, false);
        std::vector<bool> track_used(n_tracks, false);

        while (true) {
            double min_cost = std::numeric_limits<double>::max();
            int best_det = -1, best_track = -1;

            for (int i = 0; i < n_dets; ++i) {
                if (det_used[i]) continue;
                for (int j = 0; j < n_tracks; ++j) {
                    if (track_used[j]) continue;
                    if (cost(i, j) < min_cost) {
                        min_cost = cost(i, j);
                        best_det = i;
                        best_track = j;
                    }
                }
            }

            if (best_det < 0 || min_cost > max_matching_distance_) break;

            assignment[best_det] = track_indices[best_track];
            det_used[best_det] = true;
            track_used[best_track] = true;
        }

        return assignment;
    }

    // ============== 获取标签颜色 ==============
    void getLabelColor(const std::string& label, float& r, float& g, float& b)
    {
        if (label == "car")        { r = 0.0f; g = 0.8f; b = 0.2f; }
        else if (label == "truck")  { r = 0.8f; g = 0.4f; b = 0.0f; }
        else if (label == "bus")    { r = 0.8f; g = 0.0f; b = 0.8f; }
        else if (label == "person") { r = 1.0f; g = 0.2f; b = 0.2f; }
        else if (label == "bicycle"){ r = 0.2f; g = 0.6f; b = 1.0f; }
        else if (label == "box")    { r = 1.0f; g = 1.0f; b = 0.0f; }
        else                        { r = 0.5f; g = 0.5f; b = 0.5f; }
    }

    // ============== 创建跟踪 ID 文本 Marker ==============
    visualization_msgs::Marker createTrackIdMarker(
        const std_msgs::Header& header,
        int id,
        const geometry_msgs::Point& position,
        const std::string& label,
        int track_id,
        double velocity_mag,
        float r, float g, float b)
    {
        visualization_msgs::Marker marker;
        marker.header = header;
        marker.ns = "track_label";
        marker.id = id;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = position.x;
        marker.pose.position.y = position.y;
        marker.pose.position.z = position.z + 1.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.z = 0.4;

        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0f;

        std::ostringstream oss;
        oss.setf(std::ios::fixed);
        oss.precision(1);
        oss << "[" << track_id << "] " << label << " v=" << velocity_mag << "m/s";
        marker.text = oss.str();

        marker.lifetime = ros::Duration(0.3);

        return marker;
    }

    // ============== 创建速度箭头 Marker ==============
    visualization_msgs::Marker createVelocityArrowMarker(
        const std_msgs::Header& header,
        int id,
        const geometry_msgs::Point& position,
        const Eigen::Vector3d& velocity,
        float r, float g, float b)
    {
        visualization_msgs::Marker marker;
        marker.header = header;
        marker.ns = "track_velocity";
        marker.id = id;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;

        geometry_msgs::Point start = position;
        geometry_msgs::Point end;
        end.x = position.x + velocity[0];
        end.y = position.y + velocity[1];
        end.z = position.z + velocity[2];

        marker.points.push_back(start);
        marker.points.push_back(end);

        marker.scale.x = 0.08;  // 箭头轴直径
        marker.scale.y = 0.15;  // 箭头头部直径
        marker.scale.z = 0.0;   // 未使用

        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0f;

        marker.lifetime = ros::Duration(0.3);

        return marker;
    }

    // ============== 创建预测轨迹 Marker (LINE_STRIP) ==============
    visualization_msgs::Marker createPredictionTrailMarker(
        const std_msgs::Header& header,
        int id,
        const geometry_msgs::Point& current_pos,
        const Eigen::Vector3d& velocity,
        double horizon,
        float r, float g, float b)
    {
        visualization_msgs::Marker marker;
        marker.header = header;
        marker.ns = "track_prediction";
        marker.id = id;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;

        marker.scale.x = 0.06;

        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 0.6f;

        // 生成预测轨迹点 (每隔 0.1s 一个点)
        int num_steps = static_cast<int>(horizon / 0.1);
        for (int i = 0; i <= num_steps; ++i) {
            double t = i * 0.1;
            geometry_msgs::Point p;
            p.x = current_pos.x + velocity[0] * t;
            p.y = current_pos.y + velocity[1] * t;
            p.z = current_pos.z + velocity[2] * t;
            marker.points.push_back(p);
        }

        marker.lifetime = ros::Duration(0.3);

        return marker;
    }

    // ============== 主回调 ==============
    void objectsCallback(const autoware_msgs::DetectedObjectArray::ConstPtr& msg) {

        ros::Time cb_start = ros::Time::now();

        double dt = 0.1;  // 默认时间步长
        if (!tracks_.empty() && !msg->header.stamp.isZero()) {
            dt = (msg->header.stamp - tracks_[0].last_update_time).toSec();
            if (dt <= 0.0 || dt > 2.0) dt = 0.1;  // 合理性检查
        }

        bool has_object_sub = (pub_objects_.getNumSubscribers() > 0);
        bool has_marker_sub = debug_ && (pub_markers_.getNumSubscribers() > 0);
        bool has_predicted_sub = debug_ && (pub_predicted_.getNumSubscribers() > 0);

        if (!has_object_sub && !has_marker_sub && !has_predicted_sub) {
            return;
        }

        // ---- Step 1: 预测所有已有跟踪目标 ----
        for (auto& track : tracks_) {
            track.kf.predict(dt);
            track.age++;
        }

        // ---- Step 2: 提取检测位置 ----
        std::vector<Eigen::Vector3d> det_positions;
        for (const auto& obj : msg->objects) {
            det_positions.push_back(Eigen::Vector3d(
                obj.pose.position.x,
                obj.pose.position.y,
                obj.pose.position.z));
        }

        // ---- Step 3: 收集有效跟踪索引 ----
        std::vector<int> active_track_indices;
        for (int i = 0; i < static_cast<int>(tracks_.size()); ++i) {
            if (tracks_[i].consecutive_invisible <= max_consecutive_invisible_) {
                active_track_indices.push_back(i);
            }
        }

        // ---- Step 4: 数据关联 ----
        std::vector<int> assignment = associateDetectionsToTracks(det_positions, active_track_indices);

        // ---- Step 5: 更新匹配的跟踪目标 ----
        std::vector<bool> track_matched(tracks_.size(), false);
        for (int i = 0; i < static_cast<int>(assignment.size()); ++i) {
            if (assignment[i] < 0) continue;

            int track_idx = -1;
            for (int j = 0; j < static_cast<int>(active_track_indices.size()); ++j) {
                if (active_track_indices[j] == assignment[i]) {
                    track_idx = j;
                    break;
                }
            }
            if (track_idx < 0) continue;

            int ti = assignment[i];
            TrackedObject& track = tracks_[ti];

            // 更新卡尔曼滤波器
            track.kf.update(det_positions[i]);

            // 更新速度和加速度
            Eigen::Vector3d new_velocity = track.kf.getVelocity();
            if (track.total_visible_count > 1) {
                track.acceleration = (new_velocity - track.velocity) / dt;
            }
            track.prev_velocity = track.velocity;
            track.velocity = new_velocity;

            // 更新检测信息
            track.last_detection = msg->objects[i];
            track.label = msg->objects[i].label;
            track.total_visible_count++;
            track.consecutive_invisible = 0;
            track.last_update_time = msg->header.stamp;

            track_matched[ti] = true;
        }

        // ---- Step 6: 处理未匹配的检测 (创建新跟踪) ----
        for (int i = 0; i < static_cast<int>(det_positions.size()); ++i) {
            if (assignment[i] >= 0) continue;

            TrackedObject new_track;
            new_track.id = next_track_id_++;
            new_track.label = msg->objects[i].label;
            new_track.last_detection = msg->objects[i];
            new_track.kf.initialize(det_positions[i], dt);
            new_track.kf.predict(0.0);  // 初始化后无需再次预测
            new_track.age = 1;
            new_track.total_visible_count = 1;
            new_track.consecutive_invisible = 0;
            new_track.last_update_time = msg->header.stamp;
            new_track.velocity = Eigen::Vector3d::Zero();
            new_track.acceleration = Eigen::Vector3d::Zero();
            new_track.prev_velocity = Eigen::Vector3d::Zero();

            tracks_.push_back(new_track);
        }

        // ---- Step 7: 更新未匹配的跟踪目标 ----
        for (int i = 0; i < static_cast<int>(tracks_.size()); ++i) {
            if (!track_matched[i]) {
                tracks_[i].consecutive_invisible++;
            }
        }

        // ---- Step 8: 删除失效的跟踪目标 ----
        tracks_.erase(
            std::remove_if(tracks_.begin(), tracks_.end(),
                [this](const TrackedObject& t) {
                    return t.consecutive_invisible > max_consecutive_invisible_;
                }),
            tracks_.end());

        // ---- Step 9: 构建输出消息 ----
        autoware_msgs::DetectedObjectArray output_array;
        output_array.header = msg->header;

        visualization_msgs::MarkerArray marker_array;
        geometry_msgs::PoseArray predicted_poses;
        predicted_poses.header = msg->header;

        int marker_id = 0;

        for (const auto& track : tracks_) {
            // 只发布达到最小可见次数的跟踪目标
            if (track.total_visible_count < min_visible_count_) continue;

            autoware_msgs::DetectedObject obj = track.last_detection;
            obj.header = msg->header;
            obj.id = track.id;

            // 更新位置为卡尔曼滤波器估计位置
            Eigen::Vector3d pos = track.kf.getPosition();
            obj.pose.position.x = pos[0];
            obj.pose.position.y = pos[1];
            obj.pose.position.z = pos[2];

            // 更新速度
            obj.velocity.linear.x = track.velocity[0];
            obj.velocity.linear.y = track.velocity[1];
            obj.velocity.linear.z = track.velocity[2];
            obj.velocity_reliable = true;

            // 更新加速度
            obj.acceleration.linear.x = track.acceleration[0];
            obj.acceleration.linear.y = track.acceleration[1];
            obj.acceleration.linear.z = track.acceleration[2];
            obj.acceleration_reliable = true;

            output_array.objects.push_back(obj);

            // ---- debug: 可视化 ----
            if (has_marker_sub) {
                float cr, cg, cb;
                getLabelColor(track.label, cr, cg, cb);

                double vel_mag = track.velocity.norm();

                // 跟踪 ID 标签
                visualization_msgs::Marker label_marker = createTrackIdMarker(
                    msg->header, marker_id, obj.pose.position,
                    track.label, track.id, vel_mag, cr, cg, cb);
                marker_array.markers.push_back(label_marker);

                // 速度箭头
                if (vel_mag > 0.1) {
                    visualization_msgs::Marker vel_marker = createVelocityArrowMarker(
                        msg->header, marker_id, obj.pose.position,
                        track.velocity, cr, cg, cb);
                    marker_array.markers.push_back(vel_marker);

                    // 预测轨迹
                    visualization_msgs::Marker pred_marker = createPredictionTrailMarker(
                        msg->header, marker_id, obj.pose.position,
                        track.velocity, prediction_horizon_, cr, cg, cb);
                    marker_array.markers.push_back(pred_marker);
                }

                marker_id++;
            }

            // ---- debug: 预测位姿 ----
            if (has_predicted_sub) {
                geometry_msgs::Pose pred_pose;
                pred_pose.position.x = pos[0] + track.velocity[0] * prediction_horizon_;
                pred_pose.position.y = pos[1] + track.velocity[1] * prediction_horizon_;
                pred_pose.position.z = pos[2] + track.velocity[2] * prediction_horizon_;
                pred_pose.orientation = obj.pose.orientation;
                predicted_poses.poses.push_back(pred_pose);
            }
        }

        // ---- 发布结果 ----
        if (has_object_sub) {
            pub_objects_.publish(output_array);
        }

        if (has_marker_sub) {
            pub_markers_.publish(marker_array);
        }

        if (has_predicted_sub) {
            pub_predicted_.publish(predicted_poses);
        }

        // 【监控指标】
        ros::Time cb_end = ros::Time::now();
        lidar_pipeline_monitor::PipelineMetrics metric;
        metric.header.stamp = msg->header.stamp;
        metric.node_name = "7_tracking";
        metric.transmission_delay = (cb_start - msg->header.stamp).toSec() * 1000.0;
        metric.processing_time = (cb_end - cb_start).toSec() * 1000.0;
        metric.total_latency = (cb_end - msg->header.stamp).toSec() * 1000.0;
        pub_metrics_.publish(metric);

        ROS_INFO_THROTTLE(2.0,
            "[Tracking] Input objects: %lu, Active tracks: %lu, Output objects: %lu",
            msg->objects.size(), tracks_.size(), output_array.objects.size());
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_tracking_prediction_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    LidarTrackingPredictionNode node(nh, pnh);

    ros::spin();

    return 0;
}