/**
 * @file IrregularPolygonFilter.cpp
 * @brief 不规则多边形区域滤波器实现
 *
 * 实现了基于射线法的多边形内外判断，以及正向/反向点云滤波功能。
 * 在本项目中主要用于过滤车体自身的激光雷达反射点。
 */

#include "obstacle_detection/IrregularPolygonFilter.hpp"

/**
 * @brief 构造函数 - 初始化默认参数
 *
 * Z轴范围默认为float能表示的最大范围（相当于不启用Z过滤）
 * 强度滤波默认关闭
 */
IrregularPolygonFilter::IrregularPolygonFilter() : 
                            min_z_(-std::numeric_limits<float>::max()), 
                            max_z_(std::numeric_limits<float>::max()),
                            min_intensity_(0.0f),
                            max_intensity_(std::numeric_limits<float>::max()),
                            intensity_filter_enabled_(false) {}

// 设置多边形顶点 (XY平面)
void IrregularPolygonFilter::setPolygonVertices(const std::vector<Eigen::Vector2f>& vertices) {
    polygon_vertices_ = vertices;
}

// 设置Z轴范围
void IrregularPolygonFilter::setZRange(float min_z, float max_z) {
    min_z_ = min_z;
    max_z_ = max_z;
}

// 设置强度范围
void IrregularPolygonFilter::setIntensityRange(float min_intensity, float max_intensity) {
    min_intensity_ = min_intensity;
    max_intensity_ = max_intensity;
    intensity_filter_enabled_ = true;
}

// 判断点是否在多边形内 (射线法)
bool IrregularPolygonFilter::isPointInPolygon(const pcl::PointXYZI& point) {
    // 首先检查Z轴范围
    if (point.z < min_z_ || point.z > max_z_) {
        return false;
    }
    
    // 如果多边形顶点为空，返回false
    if (polygon_vertices_.empty()) {
        return false;
    }
    
    // 射线法判断点是否在多边形内
    int crossings = 0;
    int n = polygon_vertices_.size();
    
    for (int i = 0; i < n; i++) {
        const Eigen::Vector2f& v1 = polygon_vertices_[i];
        const Eigen::Vector2f& v2 = polygon_vertices_[(i + 1) % n];
        
        // 检查射线是否与边相交
        if (((v1.y() <= point.y) && (v2.y() > point.y)) || 
            ((v1.y() > point.y) && (v2.y() <= point.y))) {
            
            float vt = (point.y - v1.y()) / (v2.y() - v1.y());
            float x_intersect = v1.x() + vt * (v2.x() - v1.x());
            
            if (point.x < x_intersect) {
                crossings++;
            }
        }
    }
    
    return (crossings % 2) == 1;
}

// 应用滤波器 - 保留多边形内的点
void IrregularPolygonFilter::applyFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
                pcl::PointCloud<pcl::PointXYZI>::Ptr& filtered_cloud) {
    filtered_cloud->clear();
    filtered_cloud->header = input_cloud->header;
    filtered_cloud->width = input_cloud->width;
    filtered_cloud->height = input_cloud->height;
    filtered_cloud->is_dense = input_cloud->is_dense;
    
    for (const auto& point : *input_cloud) {

        if (isPointInPolygon(point)) {
            filtered_cloud->push_back(point);
        }
        else if (intensity_filter_enabled_){
            if (point.intensity >= min_intensity_ && point.intensity <= max_intensity_ && 
                fabs(point.intensity - 9.00) > 0.01) {
                filtered_cloud->push_back(point);
            }
        }
    }
}

// 应用反向滤波器 - 保留多边形外的点
void IrregularPolygonFilter::applyFilterNegative(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
                        pcl::PointCloud<pcl::PointXYZI>::Ptr& filtered_cloud) {
    filtered_cloud->clear();
    filtered_cloud->header = input_cloud->header;
    filtered_cloud->width = input_cloud->width;
    filtered_cloud->height = input_cloud->height;
    filtered_cloud->is_dense = input_cloud->is_dense;
    
    for (const auto& point : *input_cloud) {
        if (!isPointInPolygon(point)) {
            filtered_cloud->push_back(point);
        }
        else if (intensity_filter_enabled_){
            if (point.intensity >= min_intensity_ && point.intensity <= max_intensity_ && 
                fabs(point.intensity - 9.00) > 0.01) {
                filtered_cloud->push_back(point);
            }
        }
    }
}
