# lidar_map_filter - 地图点云背景过滤器

## 功能概述

使用静态地图点云构建 KD-Tree，对当前帧的每个点查询地图中最近点，根据距离阈值判断并滤除静态背景（墙壁/树木等），保留新障碍物点。

### 算法流程

```
启动: 加载静态地图 PCD 文件到内存
          ↓
每帧回调:
  1. 根据地图构建模式, 决定使用全地图或局部裁剪地图
  2. 对地图构建 KD-Tree
  3. 根据搜索模式, 对当前帧每个点查询地图
  4. 距离 < 阈值 → 背景(滤除), 距离 ≥ 阈值 → 新障碍物(保留)
```

## 两种独立模式

### 1. 地图构建模式 (`use_local_map`)

| 模式 | 说明 | 适用场景 |
|------|------|----------|
| `true` (局部地图) | 每帧裁剪地图到 velodyne 周围指定范围 | 大地图 + 实时性要求高 |
| `false` (全地图) | 使用完整地图构建 KD-Tree（只构建一次） | 小地图 |

局部地图裁剪范围（相对于 velodyne 原点）：
- X: `[-rear, +front]`，默认 `[-3, +5]` 米
- Y: `[-right, +left]`，默认 `[-2.5, +2.5]` 米

### 2. 搜索模式 (`search_mode`)

| 模式 | 说明 | 判定逻辑 |
|------|------|----------|
| `"nearest"` | 最近邻搜索 (k=1) | 最近距离 < 阈值 → 背景 |
| `"radius"` | 半径搜索 | 半径内有地图点 → 背景 |

## 话题

| 类型 | 话题名 | 消息类型 | 说明 |
|------|--------|----------|------|
| 订阅 | `/lidar_no_ground` | `sensor_msgs/PointCloud2` | 去地面后的点云（输入） |
| 发布 | `/lidar_map_filtered` | `sensor_msgs/PointCloud2` | 过滤后的障碍物点云（主输出） |
| 发布 | `/lidar_map_background` | `sensor_msgs/PointCloud2` | 被滤除的背景点云（调试） |
| 发布 | `/lidar_local_map` | `sensor_msgs/PointCloud2` | 局部地图点云（调试） |
| 发布 | `/pipeline/metrics` | `PipelineMetrics` | 性能监控 |

## 参数配置

参数文件：`workspace/param/map_filter.yaml`

```yaml
# 输入输出话题
input_topic: "/lidar_no_ground"
output_topic: "/lidar_map_filtered"
background_topic: "/lidar_map_background"
local_map_topic: "/lidar_local_map"

# 地图文件
map_pcd_path: "/home/getq/perception/map.pcd"

# 搜索模式: "nearest" 或 "radius"
search_mode: "nearest"
distance_threshold: 0.2                  # 距离阈值/搜索半径(米)

# 地图构建模式: true=局部地图, false=全地图
use_local_map: true
local_map_x_front: 5.0                   # 前方(米)
local_map_x_rear: 3.0                    # 后方(米)
local_map_y_left: 2.5                    # 左侧(米)
local_map_y_right: 2.5                   # 右侧(米)
```

## 使用方法

### 启动节点

```bash
roslaunch lidar_map_filter lidar_map_filter.launch
```

### RViz 调试

在 RViz 中添加以下话题进行可视化：
- `/lidar_map_filtered` — 过滤后的障碍物点云（绿色）
- `/lidar_map_background` — 被滤除的背景点云（红色）
- `/lidar_local_map` — 局部地图点云（蓝色）

## 在管线中的位置

```
[传感器输入] → [降采样] → [地面过滤] → [地图背景过滤] → [聚类/检测]
                                                    ↑
                                              lidar_map_filter
```

节点编号：`4_map_filter`（在 PipelineMetrics 中标识）