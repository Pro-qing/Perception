## obstacle_area_detection 项目架构整理

### 一、项目概述
这是一个基于ROS的**库区障碍物检测节点**，专为**多库位场景**设计。仅使用mid360雷达的点云数据，对多个停车位（库位）区域进行独立的障碍物检测，判断目标区域是否存在异物，并通过位标志报告检测结果。

**与 obstacle_detection 的关键差异**：
- 仅使用mid360单雷达（无tip补盲雷达）
- 去除电梯检测逻辑，专注库位场景
- 支持多库位同时检测（每个库位独立维护状态）
- 启用条件简化为距离判断

### 二、文件结构
```
src/obstacle_area_detection/
├── CMakeLists.txt                              # 构建配置(colcon, C++17, PCL 1.8)
├── package.xml                                 # ROS包描述
├── include/obstacle_area_detection/
│   ├── obstacle_area_detection.hpp             # 主类头文件(多库位管理 + 双层检测)
│   └── IrregularPolygonFilter.hpp              # 不规则多边形滤波器头文件
├── src/
│   ├── obstacle_area_detection.cpp             # 主入口 + 构造函数 + 核心回调
│   ├── pointcloud_preprocessing.cpp            # 点云预处理模块(7步流水线)
│   ├── area_detection.cpp                      # 区域检测模块(Layer1 + Layer2 + 防抖)
│   ├── state_callbacks.cpp                     # 状态回调模块(keypoint_path + current_pose)
│   ├── visualization.cpp                       # 可视化模块(3D边界框 + 状态文本)
│   └── IrregularPolygonFilter.cpp              # 多边形滤波器实现(射线法)
├── launch/
│   └── obstacle_area_detection.launch          # 启动文件
└── params/
    └── obstacle_area_detection.yaml            # 运行时参数配置
```

### 三、核心数据流
```
[mid360 /points_mid_calibration] ──→ midCloudCallback
                                            │
                            ┌───────────────┼───────────────┐
                            ▼                               ▼
                    preprocessMidCloud              遍历所有活跃库位
                    (标定→ROI→变换→                 (ParkingSpot列表)
                     降采样→Z滤波→                         │
                     地面分割→车体过滤)                     ▼
                            │                    checkAreaForSpot(spot, cloud)
                            │                         │
                    filtered_cloud            ┌──────┼──────┐
                                              ▼      ▼      ▼
                                           区域内  Layer2  防抖
                                           点检测  外围聚类 滑动窗口
                                              │
                                              ▼
                                      spot.has_obstacle
                                              │
                                              ▼
                                      发布状态 + 可视化
```

### 四、关键组件说明

| 组件 | 作用 |
|------|------|
| **ObstacleAreaDetection** | 主类，管理多库位检测流水线 |
| **IrregularPolygonFilter** | 不规则多边形滤波器，用于过滤车体自身点 |
| **ParkingSpot** | 库位信息结构体(id + 位姿 + 检测状态 + 启用状态) |
| **AreaBounds** | 检测区域边界框(所有库位共享) |
| **PreprocessResult** | 点云预处理结果结构体 |

### 五、多库位管理机制

**目标点获取**：从 `/keypoint_path` 导航路径中提取所有 `carports` 类型的关键点，每个关键点作为一个 `ParkingSpot`。

**唯一标识**：每个库位通过索引生成ID（如 `carports_0`, `carports_1`）。

**增量更新**：当路径更新时，通过位置比较判断库位是否变化，保留位置未变库位的检测历史记录，避免状态丢失。

**启用条件**（每个库位独立判断）：
- **启用**：`distance_threshold` < 车辆到库位距离 < `area_enable_distance`
- **禁用（过近）**：距离 < `distance_threshold`（避免车辆自身被误检）
- **禁用（过远）**：距离 > `area_enable_distance`（节省计算资源）

### 六、双层检测机制

#### Layer 1：内部区域检测 (`checkAreaForSpot`)
1. 将目标点从map坐标系变换到velodyne坐标系
2. 遍历所有障碍物点，变换到目标点局部坐标系
3. 判断是否落在预定义的3D边界框(AABB)内
4. 使用最小点数阈值（`min_region_points=5`）过滤散射噪声
5. 双重验证确保准确性

#### Layer 2：外围聚类补充检测 (`checkProximityCluster`)
**触发条件**：仅当Layer 1未检测到障碍物时触发

**场景**：大型障碍物完全占据库位入口，激光点落在库位框外，Layer 1无法检测到

**处理流程**：
1. 对预处理后的点云做欧几里得聚类
2. 对每个cluster逐个过滤：
   - z范围检查（过滤地面合并的巨型cluster）
   - 重心z检查（过滤残余地面）
3. 将聚类重心变换到目标点局部坐标系
4. 计算重心到库位边界框的最近距离(AABB最近点距离)
5. 检查是否有部分点进入扩展边界框
6. 距离 < `proximity_threshold` 或有点进入扩展框 → 判定有邻近障碍物

### 七、防误判设计

```
单帧检测 → 最小点数过滤(≥5点) → Layer2外围补充 → 滑动窗口(5帧) → 施密特触发器 → 最终结果
```

#### 三层防误判机制：

**1. 最小点数阈值** (`min_region_points = 5`)
- 区域内点数低于此值不判定为障碍物
- 过滤掉1-2个散射噪声点造成的误报

**2. 滑动窗口防抖** (`area_history_size = 5`)
- 每个库位独立维护 `std::deque<bool>` 滑动窗口
- 记录最近5帧的检测结果

**3. 施密特触发器** (防状态跳变)
- **确认**：`noise_count >= area_confirm_threshold (3)` → 设置障碍物标志
- **清除**：`noise_count <= area_clear_threshold (1)` → 清除障碍物标志
- **中间状态**（count=2）：保持当前状态不变，防止状态跳变
- 例：history_size=5, confirm=3, clear=1 → 确认需≥3帧，清除需≤1帧，中间2帧保持不变

### 八、检测结果输出

通过 `std_msgs::UInt32` 的位标志发布到 `/obstacle_area_detection`（10Hz定时发布）：
- **bit0 (0x01)**：第1个库位检测结果（1=有障碍物）
- **bit1 (0x02)**：第2个库位检测结果（1=有障碍物）
- **bit2 (0x04)**：第3个库位检测结果（1=有障碍物）
- ...最多支持32个库位

### 九、ROS Topic汇总

**订阅**：

| Topic | 消息类型 | 说明 |
|-------|----------|------|
| `/points_mid_calibration` | sensor_msgs/PointCloud2 | mid360点云 |
| `/keypoint_path` | autoware_msgs/KeyPointArray | 导航关键点路径（获取库位列表） |
| `/current_pose` | geometry_msgs/PoseStamped | 当前车辆位姿（距离判断） |

**发布**：

| Topic | 消息类型 | 说明 |
|-------|----------|------|
| `/obstacle_area_detection` | std_msgs/UInt32 | 障碍物检测状态（位标志） |
| `/obstacle_area_markers` | visualization_msgs/MarkerArray | 库位3D边界框+状态文本 |
| `/area_preprocessed_points` | sensor_msgs/PointCloud2 | 预处理后的点云 |
| `/area_ground_points` | sensor_msgs/PointCloud2 | 地面点云 |
| `/area_obstacle_points` | sensor_msgs/PointCloud2 | 障碍物点云 |
| `/area_calibration_points` | sensor_msgs/PointCloud2 | 标定后点云 |
| `/area_target_region_points` | sensor_msgs/PointCloud2 | 目标区域内点云 |
| `/area_proximity_cluster_points` | sensor_msgs/PointCloud2 | 外围聚类检测点云 |
| `/area_proximity_markers` | visualization_msgs/MarkerArray | 外围聚类可视化标记 |

### 十、可视化说明

**库位边界框颜色**：
- 🟢 **青色**：无障碍物（CLEAR）
- 🔴 **红色**：有障碍物（OCCUPIED）
- ⚪ **灰色**：未启用（DISABLED）

**状态文本**：每个库位上方显示 `[库位ID] [状态]`，如 `carports_0 [CLEAR]`

### 十一、参数文件说明

所有参数均支持运行时修改（YAML动态重载，1秒生效）：

| 参数类别 | 关键参数 | 默认值 | 说明 |
|----------|----------|--------|------|
| 基础参数 | `target_frame` | velodyne | 目标坐标系 |
| | `points_mid_topic` | /points_mid_calibration | mid360点云topic |
| | `distance_threshold` | 0.4m | 过近距离禁用阈值 |
| | `area_enable_distance` | 10.0m | 库位检测启用距离 |
| 点云滤波 | `voxel_leaf_size` | 0.0m | 体素降采样(0=不降采样) |
| | `z_axis_min/max` | -1.60/0.50m | Z轴滤波范围 |
| | `roi_radius` | 10.0m | ROI区域半径 |
| 地面分割 | `ground_threshold` | 0.15m | RANSAC距离阈值 |
| 检测区域 | `area_min_x/max_x` | -0.7/0.9m | X轴范围 |
| | `area_min_y/max_y` | -0.60/0.60m | Y轴范围 |
| | `area_min_z/max_z` | 0.2/1.00m | Z轴范围 |
| 防误判 | `min_region_points` | 5 | 最小点数阈值 |
| | `area_history_size` | 5 | 滑动窗口大小 |
| | `area_confirm_threshold` | 3 | 确认阈值 |
| | `area_clear_threshold` | 1 | 清除阈值 |
| 外围聚类 | `enable_proximity_cluster` | true | 是否启用Layer2 |
| | `cluster_tolerance` | 0.2m | 聚类容差 |
| | `cluster_min_points` | 8 | 最小聚类点数 |
| | `proximity_threshold` | 0.5m | 邻近距离阈值 |

### 十二、启动方式

```bash
# 编译
cd /home/getq/perception
colcon build --packages-select obstacle_area_detection

# 启动（使用默认yaml路径）
source install/setup.bash
roslaunch obstacle_area_detection obstacle_area_detection.launch

# 启动（指定yaml路径）
roslaunch obstacle_area_detection obstacle_area_detection.launch yaml_file_path:=/path/to/your/config.yaml