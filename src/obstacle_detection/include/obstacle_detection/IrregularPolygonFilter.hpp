#ifndef __IRREGULARPOLYGONFILTER_HPP
#define __IRREGULARPOLYGONFILTER_HPP

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <vector>
#include <Eigen/Dense>

class IrregularPolygonFilter {
private:
    std::vector<Eigen::Vector2f> polygon_vertices_;
    float min_z_, max_z_;
    float min_intensity_;
    float max_intensity_;
    bool intensity_filter_enabled_;

public:
    IrregularPolygonFilter();
    // 设置多边形顶点 (XY平面)
    void setPolygonVertices(const std::vector<Eigen::Vector2f>& vertices);
    
    // 设置Z轴范围
    void setZRange(float min_z, float max_z);
    
    // 判断点是否在多边形内 (射线法)
    bool isPointInPolygon(const pcl::PointXYZI& point);

    void setIntensityRange(float min_intensity, float max_intensity);
    
    // 应用滤波器
    void applyFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
                    pcl::PointCloud<pcl::PointXYZI>::Ptr& filtered_cloud);

    void applyFilterNegative(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
                        pcl::PointCloud<pcl::PointXYZI>::Ptr& filtered_cloud);
};


#endif
