# `lio_odom`

`lio_odom` 是一个轻量的 ROS 2 功能包，用于将 FAST-LIO 输出的里程计转换为更常见的 `odom -> base_link` 形式。

它适用于以下场景：

- FAST-LIO 发布的里程计坐标关系是 `camera_init -> body`
- 下游模块期望接收 `odom -> base_link`
- LiDAR、IMU 和 `base_link` 并不共点，需要进行完整的刚体变换

## 功能说明

本包提供一个节点：

- `odom_transformer`

该节点订阅原始里程计，将 FAST-LIO 的 `body` 坐标系通过一个固定 SE(3) 外参变换到 `base_link`，然后重新发布为：

- `/odom` 话题上的 `nav_msgs/msg/Odometry`
- 可选的 `odom -> base_link` TF

## 变换链说明

在当前工作区中，FAST-LIO 的典型输出为：

- 父坐标系：`camera_init`
- 子坐标系：`body`

本包将其转换为：

- 父坐标系：`odom`
- 子坐标系：`base_link`

内部使用的变换链为：

```text
T_odom_base = T_camera_init_body * T_body_base
```

其中：

```text
T_body_base = (T_livox_body)^-1 * (T_base_livox)^-1
```

参数含义如下：

- `base_to_livox`：`base_link` 到 LiDAR 坐标系的变换
- `livox_to_body`：LiDAR 坐标系到 FAST-LIO `body` 坐标系的变换

因此，该节点需要两组外参：

- `base_link` 与 LiDAR 之间的机械安装外参
- FAST-LIO 或 LI-Init 等标定工具提供的 LiDAR 到 IMU/`body` 的外参

## 包结构

- `src/odom_transformer_node.cpp`：变换节点实现
- `config/odom_transformer.yaml`：节点参数配置
- `config/fast_lio_minimal.yaml`：`minimal_odom.launch.py` 使用的 FAST-LIO 配置
- `launch/odom_transformer.launch.py`：仅启动变换节点
- `launch/minimal_odom.launch.py`：同时启动 FAST-LIO 和变换节点
- `launch/fast_lio_with_odom.launch.py`：本包中的另一种联合启动方式

## 编译

在工作区根目录执行：

```bash
colcon build --symlink-install
source install/setup.bash
```

## 运行

### 只启动变换节点

当 FAST-LIO 已经在运行，并且正在发布 `/Odometry` 时，可使用：

```bash
ros2 launch lio_odom odom_transformer.launch.py
```

### 同时启动 FAST-LIO 和变换节点(只期待得到odom到base_link)

```bash
ros2 launch lio_odom minimal_odom.launch.py
```

## 参数说明

`config/odom_transformer.yaml` 中的主要参数如下：

- `input_odom_topic`：FAST-LIO 原始里程计话题，默认 `/Odometry`
- `output_odom_topic`：转换后的里程计话题，默认 `/odom`
- `input_parent_frame`：原始里程计父坐标系，默认 `camera_init`
- `input_child_frame`：原始里程计子坐标系，默认 `body`
- `output_parent_frame`：输出里程计父坐标系，默认 `odom`
- `output_child_frame`：输出里程计子坐标系，默认 `base_link`
- `publish_tf`：是否发布 `odom -> base_link` 的 TF
- `debug.enable`：是否开启周期性调试日志
- `debug.log_period_sec`：调试日志输出周期，单位为秒
- `base_to_livox.translation`：`base_link` 到 LiDAR 的平移
- `base_to_livox.rpy`：`base_link` 到 LiDAR 的滚转、俯仰、偏航
- `livox_to_body.translation`：LiDAR 到 FAST-LIO `body` 的平移
- `livox_to_body.rpy`：LiDAR 到 FAST-LIO `body` 的滚转、俯仰、偏航

当前示例参数为：

- `base_to_livox.translation = [0.1, 0.0, 0.11]`
- `base_to_livox.rpy = [-0.034, 0.56395, 0.0]`
- `livox_to_body.translation = [-0.012255, -0.016787, 0.050150]`
- `livox_to_body.rpy = [0.0032322401494167274, -0.017154571846166722, 0.003817108262238392]`

## 标定与方向注意事项

请务必确保每组外参的方向与参数名一致。

例如：

- 如果你的静态变换发布器表示的是 `base_link -> livox_frame`，那么它可以直接写入 `base_to_livox`
- 如果 LI-Init 输出的是 `LiDAR -> IMU`，并且 FAST-LIO 的 `body` 就是 IMU/机体系，那么它可以直接写入 `livox_to_body`

如果你手头的外参方向刚好相反，请先求逆，再写入 YAML。

## 验证方法

启动后可以使用以下命令进行检查：

```bash
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom base_link
```

如果 FAST-LIO 也在运行，可以对比以下几项：

- 原始 `/Odometry` 位姿
- 转换后的 `/odom` 位姿
- `odom -> base_link` 的 TF 输出

当 `base_link` 与 LiDAR 之间存在非零安装角时，`odom -> base_link` 与 FAST-LIO 原始位姿在平移和姿态上出现差异是正常现象。

## 备注

- 该节点应用的是完整的刚体变换，而不仅仅是平移补偿
- 姿态会和位置一起被正确变换
- 发布的里程计会保留原始协方差
- 节点在发布前会对数值有效性和四元数范数做基本检查

