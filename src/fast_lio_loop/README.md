# fast_lio_loop

Loop closure module for **FAST-LIO2 (ROS2 Humble)**.  
Loosely-coupled ScanContext place recognition + GTSAM iSAM2 pose graph
optimization — subscribes to FAST-LIO's odometry and point cloud topics,
corrects drift, and publishes a globally consistent trajectory and map.

---

## 数据流

```
FAST-LIO (laserMapping)         fast_lio_loop (this package)
┌────────────────────┐          ┌──────────────────────────────┐
│  /Odometry         │─────────▶│  LoopClosureNode             │
│  /cloud_registered │─────────▶│                              │
│  /Laser_map        │          │  ① 关键帧管理 (距离+角度)     │
└────────────────────┘          │  ② ScanContext 描述子         │
                                │  ③ 几何近邻搜索 (备选)        │
                                │  ④ 粗-精两级 GICP 验证        │
                                │  ⑤ GTSAM iSAM2 图优化        │
                                │                              │
                                │  /loop_closure/path          │
                                │  /loop_closure/odometry      │
                                │  /loop_closure/map           │
                                │  /loop_closure/markers       │
                                │  /loop_closure/save_map      │
                                └──────────────────────────────┘
```

---

## 快速开始

### 编译

```bash
cd /home/hy/ros_ws/fast_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select fast_lio_loop
source install/setup.bash
```

### 启动

**终端 1** — 启动 FAST-LIO：
```bash
cd /home/hy/ros_ws/fast_ws
source install/setup.bash
ros2 launch fast_lio mapping.launch.py
```

**终端 2** — 启动回环节点：
```bash
cd /home/hy/ros_ws/fast_ws
source install/setup.bash
ros2 launch fast_lio_loop loop_closure.launch.py
```

两个节点独立运行，回环节点订阅 FAST-LIO 的话题即可工作。

---

## 包结构

```
src/fast_lio_loop/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── loop_closure.yaml              # 所有可调参数
├── launch/
│   └── loop_closure.launch.py
├── include/fast_lio_loop/
│   ├── keyframe.hpp                    # 关键帧数据结构
│   ├── scan_context.hpp                # ScanContext 描述子
│   ├── pose_graph.hpp                  # GTSAM iSAM2 封装
│   └── loop_closure_node.hpp           # 主节点
└── src/
    ├── scan_context.cpp                # 环-扇区描述子生成 + 匹配
    ├── pose_graph.cpp                  # iSAM2 增量优化
    └── loop_closure_node.cpp           # 核心逻辑
```

---

## 架构说明

### 关键帧管理

关键帧按 **平移 1.0m** 或 **旋转 20°** 的阈值插入。每个关键帧保存：
- 原始位姿（来自 FAST-LIO 里程计）
- 优化位姿（来自 iSAM2，未触发回环时等于原始位姿）
- 下采样后的点云（用于 ICP 验证）
- ScanContext 描述子（20 环 × 60 扇区 = 1200 维）

### 回环检测（两级策略）

**第一级：ScanContext 描述子匹配**
- 对当前帧生成 20×60 的环扇区描述子（每个格子存最大 z 值）
- 环键快速筛选 → 描述子余弦距离（yaw 不变性通过列偏移实现）
- 描述子距离 < `match_threshold` (默认 0.7) → 候选

**第二级：几何近邻搜索（备选）**
- 当 ScanContext 无匹配时，检查原始里程计距离 < 8m 的历史帧
- 同时计算 ScanContext 的 best_shift 作为航向角修正
- 用于 ICP 验证的初始猜测同时包含平移和旋转信息

### ICP 验证（粗-精两级）

粗对齐 → 精对齐，两级验证：

| 阶段 | 对应距离 | 迭代次数 | 精度 | 作用 |
|---|---|---|---|---|
| 粗对齐 | 15.0m | 50 | 1e-3 | 捕获大范围位姿误差 |
| 精对齐 | 3.0m (可配) | 50 | 1e-6 | 精确对齐 |

### 位姿图优化 (GTSAM iSAM2)

- **里程计边**：相邻关键帧之间的相对位姿，噪声随距离自适应
- **回环边**：检测到的回环约束，噪声随 ICP fitness 自适应
- **优化方式**：每帧增量更新，无需 batch 优化
- 回环触发后，所有关键帧的 `opt_position` / `opt_orientation` 被更新

### 地图修正

修正后的地图通过将每个关键帧的点云从原始位姿变换到优化位姿后叠加生成。修正变换矩阵：
```
T_correction = T_opt * T_raw⁻¹
```

---

## 参数参考 (config/loop_closure.yaml)

```yaml
keyframe:
  distance_threshold: 1.0       # 关键帧平移间隔 (m)
  angle_threshold: 0.35         # 关键帧旋转间隔 (rad, ~20°)
  cloud_downsample: 0.5         # 关键帧下采样体素大小 (m)

scan_context:
  max_range: 80.0               # ScanContext 最大有效距离 (m)
  candidate_top_k: 15           # 环键候选数
  history_skip: 15              # 跳过最近 N 帧避免自匹配
  match_threshold: 0.7          # 描述子距离阈值 (越小越严)

loop:
  search_radius: 40.0           # 候选帧位置距离阈值 (m)

icp:
  max_corr_dist: 3.0            # 精对齐阶段最大对应距离 (m)
  max_iterations: 50            # 精对齐最大迭代次数
  fitness_threshold: 2.0        # ICP 接受阈值 (越小越严)

map:
  corrected_leaf_size: 0.2      # 修正地图体素滤波大小 (m)
  save_path: "/home/hy/ros_ws/fast_ws/map"    # 地图保存目录

publish:
  corrected_map: true           # 是否发布修正地图话题
```

---

## 话题和服务

### 订阅话题

| 话题 | 类型 | 来源 |
|---|---|---|
| `/Odometry` | `nav_msgs/Odometry` | FAST-LIO |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | FAST-LIO |

### 发布话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/loop_closure/path` | `nav_msgs/Path` | 修正后的全局轨迹 |
| `/loop_closure/odometry` | `nav_msgs/Odometry` | 修正后的最新里程计 |
| `/loop_closure/map` | `sensor_msgs/PointCloud2` | 修正地图 |
| `/loop_closure/markers` | `visualization_msgs/MarkerArray` | 回环连线 (绿色) |

### 服务

| 服务 | 类型 | 说明 |
|---|---|---|
| `/loop_closure/save_map` | `std_srvs/srv/Trigger` | 手动保存修正地图 PCD |

调用方式：
```bash
ros2 service call /loop_closure/save_map std_srvs/srv/Trigger "{}"
```

---

## RViz 可视化

在 RViz 中添加：

| 显示类型 | 话题 | 用途 |
|---|---|---|
| `Path` | `/loop_closure/path` | 修正轨迹 |
| `PointCloud2` | `/loop_closure/map` | 修正地图 |
| `MarkerArray` | `/loop_closure/markers` | 绿色回环连线 |

---

## 自动保存

- **Ctrl+C 退出时自动保存**修正地图到 `save_path` 目录
- 文件名为 `corrected_map_YYYYMMDD_HHMMSS.pcd`
- 支持手动 service 调用保存

---

## 参数调优经验

### 回环没触发
1. 检查终端是否有 `Loop candidate KF xx` 或 `Geometry proximity candidate` 日志
2. 如有候选但被拒：`icp.fitness_threshold` 不够宽
3. 如无候选：`scan_context.match_threshold` 过严，或 `scan_context.history_skip` 过大
4. 6F/9F 匹配：z 轴差 > 3m 时会被自动过滤（防止跨楼层误匹配）

### 地图分层
- 增大 `icp.fitness_threshold` (当前 2.0) 可接受更多回环候选
- 增大 `icp.max_corr_dist` (当前 3.0) 可提高覆盖

### 6F 自回环
- ScanContext 对单次遍历（楼梯 A 进 → 绕一圈 → 楼梯 B 出）无法匹配
- **几何近邻搜索**作为备选：当里程计距离 < 8m 时尝试匹配
- 当前参数下 6F 绕两圈可产生稠密回环约束

---

## 依赖

- ROS2 Humble
- PCL (`pcl_ros`, `pcl_conversions`)
- GTSAM 4.2+ (`/usr/local`)
- Eigen3
- Livox ROS Driver 2（FAST-LIO 所需）
