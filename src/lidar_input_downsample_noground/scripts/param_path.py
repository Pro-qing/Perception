#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Perception pipeline 参数文件路径配置
修改此文件即可切换所有 YAML 配置路径
"""

# ---- 安全检测框 ----
SAFE_OBSTACLE_YAML = '/home/getq/perception/workspace/param/safe_obstacle.yaml'

# ---- 雷达标定 ----
LIDAR_CALIBRATION_YAML = '/home/getq/perception/workspace/param/lidar_calibration.yaml'

# ---- perception pipeline 参数文件 ----
DOWNSAMPLE_YAML = '/home/getq/perception/workspace/param/downsample.yaml'
GROUND_YAML = '/home/getq/perception/workspace/param/ground.yaml'
CHARGE_YAML = '/home/getq/perception/workspace/param/charge.yaml'
EUCLIDEAN_CLUSTER_YAML = '/home/getq/perception/workspace/param/euclidean_cluster.yaml'
SHAPE_YAML = '/home/getq/perception/workspace/param/shape.yaml'