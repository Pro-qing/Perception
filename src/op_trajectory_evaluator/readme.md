## op_trajectory_evaluator 全部参数说明

### 一、通用参数（op_common_params，与其它规划模块共享）

#### 路径规划参数
| 参数 | 默认值 | 含义 |
|------|--------|------|
| `mapSource` | `0` | 地图来源：0=Autoware格式, 1=Vector Map文件夹, 2=KML |
| `mapFileName` | `""` | 地图文件路径（mapSource非0时使用） |
| `pathDensity` | `0.5` | 路径点间距（米），生成rollout路径时的采样密度 |
| `rollOutDensity` | `0.5` | 相邻rollout轨迹之间的横向间距（米） |
| `rollOutsNumber` | `6` | rollout轨迹数量（奇数），中心线两侧各3条 |

#### 速度参数
| 参数 | 默认值 | 含义 |
|------|--------|------|
| `maxVelocity` | `6.0` | 最大允许速度（m/s）= 21.6 km/h |
| `minVelocity` | `0.1` | 最小速度阈值（m/s），低于此值视为静止 |
| `maxLocalPlanDistance` | `50` | 局部规划最大距离（米） |
| `horizonDistance` | `200` | 规划视野距离（米），从当前位置向前提取的路径长度 |

#### 跟车与避障距离参数
| 参数 | 默认值 | 含义 |
|------|--------|------|
| `minFollowingDistance` | `35.0` | 最小跟车距离（米），障碍物在此距离内会触发跟车/制动 |
| `minDistanceToAvoid` | `20.0` | 开始避障距离（米），超过此距离不做避障 |
| `maxDistanceToAvoid` | `5.0` | 最大避障距离（米），小于此距离强制避障 |
| `speedProfileFactor` | `1.2` | 速度规划因子，影响减速曲线的激进程度 |

#### 安全距离参数
| 参数 | 默认值 | 含义 |
|------|--------|------|
| `horizontalSafetyDistance` | `1.2` | 横向安全距离（米），车辆半宽 + 此值 = 横向碰撞检测范围 |
| `verticalSafetyDistance` | `0.8` | 纵向安全距离（米），车辆前后 + 此值 = 纵向碰撞检测范围 |

**调参建议：**
- 车辆经常误判障碍物 → 增大 `horizontalSafetyDistance` 和 `verticalSafetyDistance`
- 跟车太远/太近 → 调整 `minFollowingDistance`
- 车辆行驶轨迹离障碍物太近 → 增大安全距离

#### 行为开关参数
| 参数 | 默认值 | 含义 |
|------|--------|------|
| `enableSwerving` | `true` | 启用绕障功能，开启后会生成多条rollout轨迹供选择 |
| `enableFollowing` | `true` | 启用跟车功能（enableSwerving=true时自动开启） |
| `enableTrafficLightBehavior` | `false` | 启用红绿灯行为 |
| `enableStopSignBehavior` | `false` | 启用停车标志行为 |
| `enableLaneChange` | `false` | 启用车道变换 |

#### 车辆物理参数
| 参数 | 默认值 | 含义 |
|------|--------|------|
| `width` | `1.85` | 车辆宽度（米） |
| `length` | `4.2` | 车辆长度（米） |
| `wheelBaseLength` | `2.7` | 轴距（米），前后轮轴之间的距离 |
| `turningRadius` | `5.2` | 最小转弯半径（米） |
| `maxSteerAngle` | `0.45` | 最大转向角（弧度），约25.8° |
| `maxAcceleration` | `3.0` | 最大加速度（m/s²） |
| `maxDeceleration` | `-3.0` | 最大减速度（m/s²），负值 |

#### 数据源参数
| 参数 | 默认值 | 含义 |
|------|--------|------|
| `velocitySource` | `2` | 速度数据来源：0=里程计odom, 1=current_velocity话题, 2=CAN总线 |

---

### 二、轨迹评估器特有参数

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `enablePrediction` | `false` | 是否启用动态障碍物预测（当前代码中被硬编码为关闭，`if(0)`） |

---

### 三、墙面过滤参数（前面已详细说明）

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `enableWallFilter` | `true` | 启用/禁用墙面过滤 |
| `wallTurnIntensityLevel1` | `0.05` | 轻转弯阈值（rad/s） |
| `wallTurnIntensityLevel2` | `0.2` | 中转弯阈值（rad/s） |
| `wallTurnIntensityLevel3` | `0.5` | 急转弯阈值（rad/s） |
| `wallSkipDistGainLevel1` | `1.5` | 轻转弯跳过距离增益 |
| `wallSkipDistGainLevel2` | `2.5` | 中转弯跳过距离增益 |
| `wallSkipDistGainLevel3` | `4.0` | 急转弯跳过距离增益 |
| `wallSafetyAttenLevel2` | `0.3` | 中转弯安全距离衰减（预留） |
| `wallSafetyAttenLevel3` | `0.6` | 急转弯安全距离衰减（预留） |
| `wallBaseSkipDistance` | `50.0` | 基础跳过距离（米） |

---

### 四、常用调参场景速查

**车辆经常急刹/误停：**
- 增大 `horizontalSafetyDistance`（如 1.2 → 1.5）
- 增大 `verticalSafetyDistance`（如 0.8 → 1.0）
- 增大 `minFollowingDistance`（如 35 → 40）

**车辆转弯时不动（墙面误检）：**
- 启用墙面过滤 `enableWallFilter=true`
- 降低 `wallTurnIntensityLevel3`（如 0.5 → 0.3）
- 增大 `wallSkipDistGainLevel3`（如 4.0 → 6.0）

**车辆避障不够灵敏：**
- 减小 `minFollowingDistance`（如 35 → 25）
- 减小 `horizontalSafetyDistance`（注意安全）

**rollout轨迹太少/太密：**
- `rollOutsNumber`：轨迹数量（如 6 → 10）
- `rollOutDensity`：轨迹间距（如 0.5 → 0.3）

**速度控制不理想：**
- `speedProfileFactor`：值越大减速越激进
- `maxAcceleration` / `maxDeceleration`：加减速限制