/**
 * @file IrregularPolygonFilter.hpp
 * @brief 不规则多边形区域滤波器头文件
 *
 * 该滤波器用于在XY平面上定义一个不规则多边形区域，并结合Z轴范围和强度范围
 * 对点云进行过滤。主要用途：
 *   1. 过滤车体自身的点（applyFilterNegative - 保留多边形外的点）
 *   2. 特定区域的点云提取（applyFilter - 保留多边形内的点）
 *
 * 核心算法：射线法（Ray Casting）判断点是否在多边形内部
 *   - 从目标点向X轴正方向发射射线
 *   - 统计射线与多边形边的交叉次数
 *   - 奇数次交叉 = 点在多边形内，偶数次 = 点在多边形外
 */
#ifndef __IRREGULARPOLYGONFILTER_HPP
#define __IRREGULARPOLYGONFILTER_HPP

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <vector>
#include <Eigen/Dense>

/**
 * @brief 不规则多边形滤波器类
 *
 * 支持在XY平面上定义任意凸/凹多边形，结合Z轴范围进行3D区域过滤。
 * 同时支持可选的强度(Intensity)过滤功能。
 */
class IrregularPolygonFilter {
private:
    std::vector<Eigen::Vector2f> polygon_vertices_;  ///< 多边形顶点列表(XY平面)
    float min_z_, max_z_;          ///< Z轴有效范围
    float min_intensity_;          ///< 强度最小值(用于强度滤波)
    float max_intensity_;          ///< 强度最大值(用于强度滤波)
    bool intensity_filter_enabled_; ///< 是否启用强度滤波

public:
    /**
     * @brief 构造函数
     *
     * 初始化默认参数：Z范围为float最大范围，强度滤波关闭
     */
    IrregularPolygonFilter();

    /**
     * @brief 设置多边形顶点 (XY平面)
     * @param vertices 顶点列表，按顺序连接形成多边形
     */
    void setPolygonVertices(const std::vector<Eigen::Vector2f>& vertices);

    /**
     * @brief 设置Z轴范围
     * @param min_z Z轴最小值
     * @param max_z Z轴最大值
     */
    void setZRange(float min_z, float max_z);

    /**
     * @brief 判断点是否在多边形内 (射线法)
     * @param point 待检测的点
     * @return true=点在多边形内且Z轴范围内
     *
     * 使用射线法(Ray Casting)算法：
     * 1. 先检查点的Z坐标是否在[min_z_, max_z_]范围内
     * 2. 从点向X轴正方向发射射线
     * 3. 统计与多边形各边的交叉次数
     * 4. 奇数次交叉表示点在多边形内部
     */
    bool isPointInPolygon(const pcl::PointXYZI& point);

    /**
     * @brief 设置强度滤波范围
     * @param min_intensity 强度最小值
     * @param max_intensity 强度最大值
     */
    void setIntensityRange(float min_intensity, float max_intensity);

    /**
     * @brief 正向滤波 - 保留多边形内的点
     * @param input_cloud 输入点云
     * @param filtered_cloud 输出点云（仅保留多边形内+Z范围内的点）
     */
    void applyFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
                    pcl::PointCloud<pcl::PointXYZI>::Ptr& filtered_cloud);

    /**
     * @brief 反向滤波 - 保留多边形外的点
     * @param input_cloud 输入点云
     * @param filtered_cloud 输出点云（仅保留多边形外的点）
     *
     * 主要用于过滤车体自身的点：定义车体轮廓为多边形，
     * 使用反向滤波移除车体范围内的点，保留外部的障碍物点。
     */
    void applyFilterNegative(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
                        pcl::PointCloud<pcl::PointXYZI>::Ptr& filtered_cloud);
};


#endif