#include "safe_obstacle.hpp"
#include <algorithm>

SafeObstacle::SafeObstacle(ros::NodeHandle &nh, ros::NodeHandle &pnh) : nh_(nh), pnh_(pnh)
{
    pnh_.param("debug_mode", debug_mode_, false);

    // 增强型参数解析 Lambda
    auto load_poly = [&](const std::string& key, std::vector<geometry_msgs::Point>& poly) {
        XmlRpc::XmlRpcValue xml_list;
        if (pnh_.getParam(key, xml_list) && xml_list.getType() == XmlRpc::XmlRpcValue::TypeArray) {
            poly.clear();
            for (int i = 0; i < xml_list.size(); ++i) {
                geometry_msgs::Point p;
                auto read_val = [&](const std::string& field) -> double {
                    if (!xml_list[i].hasMember(field)) return 0.0;
                    XmlRpc::XmlRpcValue& v = xml_list[i][field];
                    if (v.getType() == XmlRpc::XmlRpcValue::TypeInt) return static_cast<double>(static_cast<int>(v));
                    if (v.getType() == XmlRpc::XmlRpcValue::TypeDouble) return static_cast<double>(v);
                    return 0.0;
                };
                p.x = read_val("x");
                p.y = read_val("y");
                poly.push_back(p);
            }
            if(debug_mode_) ROS_INFO("[SafeObstacle] Loaded %s with %zu points", key.c_str(), poly.size());
            return true;
        }
        return false;
    };

    // 1. 读取启用的模式列表 (enable 数组)
    std::vector<int> enabled_modes;
    XmlRpc::XmlRpcValue enable_list;
    if (pnh_.getParam("enable", enable_list) && enable_list.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        for (int i = 0; i < enable_list.size(); ++i) {
            if (enable_list[i].getType() == XmlRpc::XmlRpcValue::TypeInt) {
                enabled_modes.push_back(static_cast<int>(enable_list[i]));
            }
        }
    } else {
        ROS_WARN("[SafeObstacle] 'enable' param not found or invalid format. Defaulting to mode [1].");
        enabled_modes.push_back(1);
    }

    // 2. 解析 Behavior ID 到 Mode 的映射关系
    XmlRpc::XmlRpcValue mapping_dict;
    if (pnh_.getParam("mode_mapping", mapping_dict) && mapping_dict.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
        // 遍历 YAML 中的字典
        for (auto const& kv : mapping_dict) {
            int mode_id = std::stoi(kv.first); // 键是模式 ID (XMLRPC 解析字典键会自动转为 string)
            
            // 值是 Behavior ID 的数组
            if (kv.second.getType() == XmlRpc::XmlRpcValue::TypeArray) {
                for (int i = 0; i < kv.second.size(); ++i) {
                    if (kv.second[i].getType() == XmlRpc::XmlRpcValue::TypeInt) {
                        // int behavior_id = static_cast<int>(kv.second[i]);
                        int behavior_id = 0;
                        behavior_id = kv.second[i]; // 利用重载的 = 运算符
                        // 构建反向映射：Behavior ID -> Mode ID
                        behavior_to_mode_map_[behavior_id] = mode_id; 
                    }
                }
            }
        }
    } else {
        ROS_WARN("[SafeObstacle] 'mode_mapping' param not found! Using hardcoded fallback.");
        behavior_to_mode_map_[1] = 1; behavior_to_mode_map_[2] = 1;
        behavior_to_mode_map_[3] = 2; behavior_to_mode_map_[4] = 2;
    }

    if (debug_mode_) {
        for (const auto& pair : behavior_to_mode_map_) {
            ROS_INFO("[SafeObstacle] Behavior ID %d -> triggers Mode %d", pair.first, pair.second);
        }
    }

    // 3. 动态加载多套多边形基准（只加载 enable 列表中的模式）
    for (int mode_id : enabled_modes) {
        std::string suffix = "_" + std::to_string(mode_id);
        
        load_poly("exigencyrect" + suffix, base_exigency_map_[mode_id]);
        load_poly("slowrect" + suffix, base_slow_map_[mode_id]);
        load_poly("reverse_exigencyrect" + suffix, base_rev_exigency_map_[mode_id]);
        load_poly("reverse_slowrect" + suffix, base_rev_slow_map_[mode_id]);

        // 兜底逻辑：如果第 i 套某一项没配置，则使用第 1 套的数据填充
        if (mode_id > 1) {
            if (base_exigency_map_[mode_id].empty()) base_exigency_map_[mode_id] = base_exigency_map_[1];
            if (base_slow_map_[mode_id].empty()) base_slow_map_[mode_id] = base_slow_map_[1];
            if (base_rev_exigency_map_[mode_id].empty()) base_rev_exigency_map_[mode_id] = base_rev_exigency_map_[1];
            if (base_rev_slow_map_[mode_id].empty()) base_rev_slow_map_[mode_id] = base_rev_slow_map_[1];
        }
    }

    // 4. 加载动态缩放配置
    pnh_.param("max_longitudinal_scale", max_longitudinal_scale_, 1.5);
    pnh_.param("min_longitudinal_scale", min_longitudinal_scale_, 0.7);
    pnh_.param("max_lateral_scale", max_lateral_scale_, 1.1);
    pnh_.param("min_lateral_scale", min_lateral_scale_, 0.85);
    pnh_.param("longitudinal_sensitivity", longitudinal_sensitivity_, 0.8);
    pnh_.param("lateral_sensitivity", lateral_sensitivity_, 0.3);
    pnh_.param("reference_speed", reference_speed_, 3.0);

    // 5. 订阅与发布
    cloud_points_sub_ = nh_.subscribe("/points_filter", 1, &SafeObstacle::pointsCallback, this);
    twist_raw_sub_ = nh_.subscribe("/twist_raw", 1, &SafeObstacle::twist_raw_Callback, this);
    can_info_sub_ = nh_.subscribe("/can_info", 1, &SafeObstacle::can_info_Callback, this);
    lqr_targetwayp_sub_ = nh_.subscribe("/lqr_targetwayp", 1, &SafeObstacle::lqr_targetwayp_Callback, this);

    pub_markers = nh_.advertise<visualization_msgs::MarkerArray>("safe_obstacle_markers", 1);
    pub_safe = nh_.advertise<std_msgs::Int8>("safe_obstacle_data", 1);

    ROS_INFO("[SafeObstacle] Node Initialized.");
}

void SafeObstacle::lqr_targetwayp_Callback(const autoware_msgs::Waypoint::ConstPtr& msg) {
    if (!msg) return;
    auto& behaviors = msg->wpsattr.routeBehavior;
    
    // 默认回退为常规模式 1
    int target_mode = 1; 

    // 根据 yaml 中解析出的对应关系，动态查找目标模式
    for (int b : behaviors) {
        auto it = behavior_to_mode_map_.find(b);
        if (it != behavior_to_mode_map_.end()) {
            target_mode = it->second; // 找到了对应的映射关系
        }
    }

    // 状态机发生切换时
    if (target_mode != current_mode_) {
        current_mode_ = target_mode;
        if(debug_mode_) ROS_INFO("[SafeObstacle] Mode Switched to: %d", current_mode_);
    }
}

void SafeObstacle::updateDetectionZones()
{
    int active_mode = current_mode_;
    
    // 拦截器：如果查找到的目标模式没有在 enable 中被加载，安全回退到模式 1
    if (base_exigency_map_.find(active_mode) == base_exigency_map_.end()) {
        active_mode = 1;
    }

    // 根据当前模式ID选择对应的基础框
    const auto& b_ex = base_exigency_map_[active_mode];
    const auto& b_sl = base_slow_map_[active_mode];
    const auto& b_rev_ex = base_rev_exigency_map_[active_mode];
    const auto& b_rev_sl = base_rev_slow_map_[active_mode];

    double speed = can_info_ptr_ ? fabs(can_info_ptr_->speed) : 0.0;
    double ratio = speed / reference_speed_;

    auto calc_scale = [&](double min_s, double max_s, double sens) {
        if (ratio <= 1.0) return min_s + (1.0 - min_s) * ratio;
        return 1.0 + (max_s - 1.0) * (1.0 - exp(-sens * (ratio - 1.0)));
    };

    double lon_s = std::max(min_longitudinal_scale_, std::min(max_longitudinal_scale_, calc_scale(min_longitudinal_scale_, max_longitudinal_scale_, longitudinal_sensitivity_)));
    double lat_s = std::max(min_lateral_scale_, std::min(max_lateral_scale_, calc_scale(min_lateral_scale_, max_lateral_scale_, lateral_sensitivity_)));

    auto apply_scale = [&](const std::vector<geometry_msgs::Point>& in, std::vector<geometry_msgs::Point>& out, bool is_rev) {
        out.clear();
        for (const auto& p : in) {
            geometry_msgs::Point sp;
            sp.x = (is_rev ? (p.x < 0 ? p.x * lon_s : p.x) : (p.x > 0 ? p.x * lon_s : p.x));
            sp.y = p.y * lat_s;
            out.push_back(sp);
        }
    };

    apply_scale(b_ex, current_exigencySize, false);
    apply_scale(b_sl, current_slowSize, false);
    apply_scale(b_rev_ex, current_reverse_exigencySize, true);
    apply_scale(b_rev_sl, current_reverse_slowSize, true);
}

void SafeObstacle::pointsCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    // 1. 无论有没有点云，首先更新并获取当前的检测区域
    updateDetectionZones();
    bool is_reverse = (twist_raw_ptr_ && (twist_raw_ptr_->header.frame_id == "back" || twist_raw_ptr_->twist.linear.x < -0.1));
    const auto& active_exigency = is_reverse ? current_reverse_exigencySize : current_exigencySize;
    const auto& active_slow = is_reverse ? current_reverse_slowSize : current_slowSize;

    // 2. 初始化默认输出状态为 0（安全）
    std_msgs::Int8 safeOut;
    safeOut.data = 0;

    // 3. 只有当点云非空时，才进行点云坐标转换和碰撞检测
    if (!msg->data.empty()) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *in_cloud_ptr);
        pcl::PointCloud<pcl::PointXYZI> filtered_cloud = SlidingWindow_Flitter(*in_cloud_ptr);

        for(const auto& p : filtered_cloud.points) {
            geometry_msgs::Point pt;
            pt.x = p.x; pt.y = p.y;
            
            if (PointInPolygon(pt, active_exigency)) {
                safeOut.data = 1;
                break; // 触发紧急停止，直接跳出
            }
            if (safeOut.data == 0 && PointInPolygon(pt, active_slow)) {
                safeOut.data = 2; // 触发减速，但继续遍历看是否会触发紧急停止
            }
        }
    }

    // 4. 无论如何，都发布可视化检测框和当前状态
    visualization_msgs::MarkerArray markers_;
    std_msgs::ColorRGBA col_ex = (safeOut.data == 1) ? createColor(1,0,0,1) : createColor(1,0.5,0,1);
    std_msgs::ColorRGBA col_sl = (safeOut.data == 2) ? createColor(1,0,0,1) : createColor(0,1,0,1);

    markers_.markers.push_back(CreateMarker(active_exigency, col_ex, 0));
    markers_.markers.push_back(CreateMarker(active_slow, col_sl, 1));
    
    pub_markers.publish(markers_);
    pub_safe.publish(safeOut);
}

bool SafeObstacle::PointInPolygon(const geometry_msgs::Point& pt, const std::vector<geometry_msgs::Point>& poly)
{
    if (poly.size() < 3) return false;
    bool inside = false;
    for (int i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
        if (((poly[i].y > pt.y) != (poly[j].y > pt.y)) &&
            (pt.x < (poly[j].x - poly[i].x) * (pt.y - poly[i].y) / (poly[j].y - poly[i].y) + poly[i].x)) {
            inside = !inside;
        }
    }
    return inside;
}

visualization_msgs::Marker SafeObstacle::CreateMarker(const std::vector<geometry_msgs::Point>& poly, std_msgs::ColorRGBA col, int id)
{
    visualization_msgs::Marker m;
    m.header.frame_id = "velodyne";
    m.header.stamp = ros::Time::now();
    m.ns = "safe_zones";
    m.id = id;
    m.type = visualization_msgs::Marker::LINE_STRIP;
    
    if (poly.empty()) {
        m.action = visualization_msgs::Marker::DELETE;
        return m;
    }

    m.action = visualization_msgs::Marker::ADD;
    m.scale.x = 0.05;
    m.color = col;
    m.pose.orientation.w = 1.0;
    for (const auto& p : poly) m.points.push_back(p);
    m.points.push_back(poly.front()); 
    return m;
}

pcl::PointCloud<pcl::PointXYZI> SafeObstacle::SlidingWindow_Flitter(pcl::PointCloud<pcl::PointXYZI> cloud) {
    static std::vector<pcl::PointCloud<pcl::PointXYZI>> buffer;
    buffer.push_back(cloud);
    if (buffer.size() > 3) buffer.erase(buffer.begin());
    pcl::PointCloud<pcl::PointXYZI> out;
    for (const auto& c : buffer) out += c;
    return out;
}

void SafeObstacle::twist_raw_Callback(const geometry_msgs::TwistStamped::Ptr &msg) { twist_raw_ptr_ = msg; }
void SafeObstacle::can_info_Callback(const autoware_can_msgs::CANInfo::Ptr &msg) { can_info_ptr_ = msg; }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "safe_obstacle_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    SafeObstacle obj(nh, pnh);
    ros::spin();
    return 0;
}