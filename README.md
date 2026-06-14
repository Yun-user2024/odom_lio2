# FAST-LIO2 ROS2 工作空间 — `fast_ws`

基于 **ROS 2 Humble** 的完整 FAST-LIO2 建图工作空间，集成回环检测（主要针对的是楼梯+回廊结合的调参）与 `odom → base_link` 坐标变换。适用于实时的纯激光 SLAM。

---

## 功能包一览

| 包 | 路径 | 作用 |
|---|---|---|
| **[FAST_LIO](src/FAST_LIO)** | `src/FAST_LIO/` | 核心 LiDAR-IMU 里程计（FAST-LIO2） |
| **[LIO_loop](src/LIO_loop/)** | `src/LIO_loop/` | 回环检测：ScanContext 场景识别 + GTSAM iSAM2 位姿图优化 |
| **[LIO_odom](src/LIO_odom/)** | `src/LIO_odom/` | 坐标变换：`camera_init → body` → `odom → base_link` |

---

## 数据处理流程

```
   激光雷达点云
          │
          ▼
    ┌─────────────┐
    │  FAST_LIO   │  实时里程计（IMU + LiDAR）
    │ (laserMapping)│
    └──────┬──────┘
           │  /Odometry, /cloud_registered
           │
     ┌─────┴─────┐
     │           │
     ▼           ▼
┌──────────┐ ┌──────────┐
│ LIO_odom │ │ LIO_loop │
│ (odom →  │ │ （回环    │
│ base_link│ │  检测）    │
│ TF变换)  │ │          │
└──────────┘ └──────────┘
     │           │
     ▼           ▼
  /odom     /loop_closure/path
  odom→     /loop_closure/map
  base_link
```

三个节点独立运行，通过 ROS 2 话题连接：

1. **FAST_LIO** 发布原始里程计 (`/Odometry`) 和已配准点云 (`/cloud_registered`)。
2. **LIO_loop** 订阅这些话题，通过 ScanContext 描述子检测回环，经 GICP 两级验证后，用 GTSAM iSAM2 优化位姿图，发布全局一致的轨迹和修正地图。
3. **LIO_odom** 将 FAST-LIO 的 `camera_init → body` 坐标系转换为标准的 `odom → base_link`，供下游模块使用。

---

## 快速开始

### 前置依赖

- **Ubuntu 22.04** + **ROS 2 Humble**（桌面版安装）
- **GTSAM 4.2+**（安装在 `/usr/local`）
- **PCL**、**Eigen3**（ROS 2 Humble 自带）

### 编译

```bash
cd /home/hy/ros_ws/fast_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

> 只编译特定包：`colcon build --packages-select FAST_LIO`（或 `LIO_loop`、`LIO_odom`）。

### 启动方式

#### 方式一：一键启动（FAST-LIO + 回环 + RViz）

```bash
ros2 launch LIO_loop mapping_with_loop.launch.py
```

这是推荐的方式，同时启动 FAST-LIO 建图和回环检测，自动打开 RViz。可通过参数关闭回环或 RViz：

```bash
ros2 launch LIO_loop mapping_with_loop.launch.py loop:=false rviz:=false
```

#### 方式二：分别启动（调试时更灵活）

```bash
# 终端 1 — FAST-LIO
ros2 launch fast_lio mapping.launch.py

# 终端 2 — 回环检测
ros2 launch LIO_loop loop_closure.launch.py

# 终端 3 — 里程计坐标变换（可选）
ros2 launch LIO_odom odom_transformer.launch.py
```

#### 方式三：FAST-LIO 与坐标变换同时启动

```bash
ros2 launch LIO_odom minimal_odom.launch.py
```

---

## 功能包详情

### `LIO_loop` — 回环检测

采用两级检测策略的松耦合回环检测模块：

| 层级 | 方法 | 用途 |
|---|---|---|
| 主检测 | ScanContext 描述子匹配（20 环 × 60 扇区） | 偏航不变性的场景识别 |
| 备选 | 几何近邻搜索（里程计距离 < 8 m） | 补足 ScanContext 遗漏的情况（如单次环形路径） |

检测到候选后，经过**粗对齐 → 精对齐**两级 GICP 验证，确认后向 GTSAM iSAM2 位姿图加入回环边。

**主要输出话题：**

| 话题 | 类型 | 说明 |
|---|---|---|
| `/loop_closure/path` | `nav_msgs/Path` | 修正后的全局轨迹 |
| `/loop_closure/map` | `sensor_msgs/PointCloud2` | 修正后的全局地图（体素滤波） |
| `/loop_closure/markers` | `visualization_msgs/MarkerArray` | 绿色回环连线（RViz 可视化） |
| `/loop_closure/odometry` | `nav_msgs/Odometry` | 最新修正位姿 |

Ctrl+C 退出时自动保存修正地图；也可手动调用 `save_map` 服务：

```bash
ros2 service call /loop_closure/save_map std_srvs/srv/Trigger "{}"
```

详细架构说明和参数调优见 [LIO_loop/README.md](src/LIO_loop/README.md)。

### `LIO_odom` — 里程计坐标变换

将 FAST-LIO 的 `camera_init → body` 输出转换为标准的 `odom → base_link` 坐标系，通过固定 SE(3) 刚体变换完成：

```
T_odom_base = T_camera_init_body × T_body_base
```

需要两组外参：
- **`base_to_livox`** — `base_link` 到 LiDAR 的机械安装外参
- **`livox_to_body`** — LiDAR 到 FAST-LIO `body`（IMU）的标定外参

配置见 [`config/odom_transformer.yaml`](src/LIO_odom/config/odom_transformer.yaml)。

详细参数说明和验证方法见 [LIO_odom/README.md](src/LIO_odom/README.md)。

### `FAST_LIO` — 核心 LiDAR-IMU 里程计

FAST-LIO2 主体。发布的内容：

| 话题 | 类型 | 说明 |
|---|---|---|
| `/Odometry` | `nav_msgs/Odometry` | 原始里程计（`camera_init` → `body`） |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | 已配准点云 |
| `/Laser_map` | `sensor_msgs/PointCloud2` | 全局地图（降采样） |

FAST-LIO 的配置文件位于 `src/FAST_LIO/config/`。

---

## RViz 可视化

推荐的 RViz 显示配置：

| 显示类型 | 话题 | 用途 |
|---|---|---|
| `Path` | `/loop_closure/path` | 修正后的轨迹 |
| `PointCloud2` | `/loop_closure/map` | 修正后的全局地图 |
| `PointCloud2` | `/cloud_registered` | FAST-LIO 原始点云 |
| `MarkerArray` | `/loop_closure/markers` | 绿色回环边 |

---

## 参数调优提示

各包的详细参数参考各自 README：

- **回环检测灵敏度**：[`loop_closure.yaml`](src/LIO_loop/config/loop_closure.yaml) — `scan_context.match_threshold`（越小越严）、`icp.fitness_threshold`（越大越宽松）
- **里程计坐标系**：[`odom_transformer.yaml`](src/LIO_odom/config/odom_transformer.yaml) — 确认 `base_to_livox` 和 `livox_to_body` 与机器人的机械安装和标定数据一致
- **FAST-LIO 性能**：见 FAST_LIO 自身配置文件，调整 LiDAR FOV、特征提取和 IMU 参数

---

## 目录结构

```
fast_ws/
├── README.md                    ← 本文件
├── src/
│   ├── FAST_LIO/               FAST-LIO2 里程计
│   │   ├── config/              FAST-LIO 参数
│   │   ├── launch/              启动文件
│   │   └── ...                  源码
│   │
│   ├── LIO_loop/                回环检测包
│   │   ├── config/
│   │   │   └── loop_closure.yaml
│   │   ├── launch/
│   │   │   ├── loop_closure.launch.py
│   │   │   └── mapping_with_loop.launch.py   ← 一键建图+回环
│   │   ├── include/LIO_loop/
│   │   │   ├── keyframe.hpp
│   │   │   ├── scan_context.hpp
│   │   │   ├── pose_graph.hpp
│   │   │   └── loop_closure_node.hpp
│   │   └── src/
│   │       ├── scan_context.cpp
│   │       ├── pose_graph.cpp
│   │       └── loop_closure_node.cpp
│   │
│   └── LIO_odom/                坐标变换包
│       ├── config/
│       │   ├── odom_transformer.yaml
│       │   └── fast_lio_minimal.yaml
│       ├── launch/
│       │   ├── odom_transformer.launch.py
│       │   ├── minimal_odom.launch.py
│       │   └── fast_lio_with_odom.launch.py
│       └── src/
│           └── odom_transformer_node.cpp
│
├── install/                     colcon 输出（自动生成）
├── build/                       colcon 编译产物
└── log/                         colcon 编译日志
```

---

## 许可

核心 FAST-LIO2 代码请参考原作者 [FAST-LIO2](https://github.com/hku-mars/FAST-LIO) 的开源许可。`LIO_loop` 和 `LIO_odom` 包默认采用相同许可条款，具体以各自 `package.xml` 为准。
