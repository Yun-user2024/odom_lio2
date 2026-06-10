# PLAN：聚焦 FAST-LIO2 改造以输出 LeGO-LOAM 可用的标准里程计

## 1. 目标

本文档只聚焦 **FAST-LIO2 侧需要完成的改造**，不再展开 LeGO-LOAM 内部逻辑。

目标是让 FAST-LIO2 对外稳定提供一条可供 LeGO-LOAM 使用的外部里程计，满足以下要求：

- 里程计消息类型为 `nav_msgs/Odometry`；
- 对外语义统一为：
  - `header.frame_id = "odom"`
  - `child_frame_id = "base_link"`
  - `pose = T_odom_base`
- 与里程计同步广播匹配的 TF：
  - `odom -> base_link`
- 保持现有静态安装关系：
  - `base_link -> livox_frame`
- 最终让 LeGO-LOAM 在 `wheel_odometry` 模式下消费这条外部 odom。

一句话概括：

- **不是简单把 FAST-LIO2 当前 odom 改名成 `base_link`；而是要让 FAST-LIO2 真正输出 `base_link` 在 `odom` 下的位姿。**

---

## 2. 当前项目里的关键已知条件

### 2.1 FAST-LIO2 当前已经在发布 `nav_msgs/Odometry`

根据当前源码，FAST-LIO2 已经有现成的 Odometry 发布逻辑：

- 文件：`src/FAST_LIO/src/laserMapping.cpp`
- 发布函数：`publish_odometry(...)`

其中当前对外发布的是：

- `header.frame_id = "camera_init"`
- `child_frame_id = "body"`

并把 `state_point.pos` 与 `state_point.rot` 直接填入 pose。

这说明：

- 消息类型本身已经满足要求；
- 真正需要改的是 **输出坐标语义**，而不是“从零新增一个 odom publisher”。

---

### 2.2 `body` 在当前 FAST-LIO2 中表示 IMU 主体坐标系

当前代码中的点云世界坐标变换为：

```text
p_world = R_world_body * (R_body_lidar * p_lidar + t_body_lidar) + t_world_body
```

这说明：

- `state_point.rot` 表示 `R_world_body`
- `state_point.pos` 表示 `t_world_body`
- `offset_R_L_I` / `offset_T_L_I` 表示 LiDAR 到 IMU/body 的外参

因此当前 FAST-LIO2 内部主状态参考点是：

- `body`（IMU 主体坐标系）

而不是：

- `livox_frame`
- `base_link`

---

### 2.3 FAST-LIO2 内部也能直接得到 LiDAR pose

当前代码里已经显式计算过 LiDAR 在 world 下的位置：

```text
pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I
```

这说明 FAST-LIO2 不仅有 `T_odom_body`，也具备构造 `T_odom_lidar` 的能力。

因此从工程角度，FAST-LIO2 侧有两条可能的对外改造路径：

1. 以 `body` pose 为起点，换算到 `base_link`
2. 以 `livox_frame` pose 为起点，换算到 `base_link`

结合你当前已知外参，第二条路径更自然。

---

## 3. 当前硬件与外参条件

### 3.1 LiDAR 与 IMU 为刚性连接

你当前使用的是 `Livox Mid-360`，雷达与 IMU 刚性连接，且已经有 LiDAR 到 IMU 的标定结果。

你给出的精标结果可概括为：

- `T_lidar_imu` 已知
- 即 LiDAR 到 IMU/body 的刚体外参已知

这与 FAST-LIO2 内部使用 `offset_R_L_I`、`offset_T_L_I` 的方式是一致的。

因此在 FAST-LIO2 内部：

- `body` 与 `livox_frame` 的关系是明确的；
- 当前算法状态与当前安装外参并不冲突。

---

### 3.2 当前系统已有 `base_link -> livox_frame`

当前 launch 已有静态 TF：

```xml
<node pkg="tf2_ros" exec="static_transform_publisher" name="baselink2mid360" args="0.1 0 0.11 -0.034 0.56395 0 base_link livox_frame" />
```

这意味着当前系统里已经明确定义了：

- `T_base_livox`

这是机器人主坐标系到雷达坐标系的固定安装外参。

因此：

- 你并不缺少 `base_link` 与 LiDAR 之间的关系；
- 只要 FAST-LIO2 能提供 `T_odom_livox`，就能推导出 `T_odom_base`。

---

## 4. 为什么本次改造应优先走 LiDAR -> base_link 这条链

如果只看 FAST-LIO2 侧改造，当前最合适的思路不是：

- 先纠结 `body` 能不能等同于 `base_link`

而是：

- 直接利用已知的 `LiDAR <-> base_link` 固定外参
- 将 FAST-LIO2 输出整理为 `odom -> base_link`

原因如下：

1. 当前系统已经明确存在 `base_link -> livox_frame`；
2. 当前 FAST-LIO2 内部可以构造 LiDAR pose；
3. 不需要额外引入新的 `body -> base_link` 静态外参定义；
4. 不需要把 IMU/body 强行解释成机器人主体原点；
5. 对你当前项目来说，这条链更直接，也更不容易犯语义错误。

所以本文档的推荐实现路线是：

- **FAST-LIO2 内部先得到 `T_odom_livox`，再利用 `T_base_livox` 求出 `T_odom_base`，最终对外发布 `odom -> base_link`。**

---

## 5. 这次改造真正要完成的核心事情

如果只聚焦 FAST-LIO2，本次改造的核心只有三件事：

### 5.1 保持输出消息类型为 `nav_msgs/Odometry`

这件事当前已经满足，不需要重新设计消息接口。

要做的是保留当前 publisher，同时修改它发布的：

- frame 名
- pose 数值语义
- 对应 TF 广播语义

---

### 5.2 将当前内部位姿换算成 `T_odom_base`

当前 FAST-LIO2 内部天然状态更接近：

- `T_odom_body`

但本项目推荐不直接用它对外发布，而是通过 LiDAR 中间量得到：

- `T_odom_livox`

再结合静态外参：

- `T_base_livox`

最终构造：

```text
T_odom_base = T_odom_livox * T_livox_base
```

而：

```text
T_livox_base = inverse(T_base_livox)
```

所以最终实现关系为：

```text
T_odom_base = T_odom_livox * inverse(T_base_livox)
```

---

### 5.3 让 Odometry 与 TF 完全同语义

FAST-LIO2 最终必须同时做到：

- Odometry 发布：`odom -> base_link`
- TF 广播：`odom -> base_link`

不能出现：

- Odometry 里写 `base_link`
- 实际数值还是 `body` 或 `livox_frame`

也不能出现：

- Odometry 是 `odom -> base_link`
- TF 却还是 `camera_init -> body`

两者必须完全一致。

---

## 6. 推荐采用的数学关系

### 6.1 FAST-LIO2 内部状态

从当前代码语义，内部可认为已有：

- `T_odom_body`

其中：

- `odom` 在当前代码里实际对应 `camera_init` / `world`
- `body` 对应 IMU 主体坐标系

若使用矩阵形式表达，则：

```text
T_odom_body = [ R_odom_body , t_odom_body ]
```

其中：

- `R_odom_body = state_point.rot`
- `t_odom_body = state_point.pos`

---

### 6.2 由 `body` 构造 `lidar` 位姿

已知 FAST-LIO2 内部外参：

- `T_body_lidar`

则：

```text
T_odom_livox = T_odom_body * T_body_lidar
```

这正是当前代码在几何上已经在做的事情。

从位置角度看，就是：

```text
t_odom_livox = t_odom_body + R_odom_body * t_body_lidar
```

当前代码中 `pos_lid` 就是这一步的位置实现。

LiDAR 朝向也可由：

```text
R_odom_livox = R_odom_body * R_body_lidar
```

得到。

---

### 6.3 由 `lidar` 构造 `base_link` 位姿

系统当前已有：

- `T_base_livox`

需要先求逆得到：

```text
T_livox_base = inverse(T_base_livox)
```

然后：

```text
T_odom_base = T_odom_livox * T_livox_base
```

这就是本项目最推荐的最终对外里程计构造方式。

---

## 7. 对当前 `base_link -> livox_frame` 的使用注意事项

当前静态 TF 声明的是：

- 父坐标系：`base_link`
- 子坐标系：`livox_frame`

因此它表达的是：

- `T_base_livox`

不是：

- `T_livox_base`

所以在 FAST-LIO2 里计算 `T_odom_base` 时，**不能直接拿 `base_link -> livox_frame` 右乘**，必须先求逆。

也就是说实现时必须明确：

```text
T_livox_base = inverse(T_base_livox)
```

如果这一步方向弄反，最终会导致：

- 位置偏移方向错误
- 姿态叠乘方向错误
- LeGO-LOAM 收到的外部 odom 参考点整体错位

这是本次改造最需要警惕的地方之一。

---

## 8. 推荐的 FAST-LIO2 代码改造点

以下内容只描述 **改哪里、为什么改**，不展开具体代码实现细节。

### 8.1 保留现有 `publish_odometry(...)`，但修改其发布语义

当前发布函数集中在：

- `src/FAST_LIO/src/laserMapping.cpp`
- `publish_odometry(...)`

这是最适合动手的位置，因为：

- 当前 Odometry 与 TF 已经在同一个函数中发布；
- 只需要在这里把“要填入的 pose”从 `body` 替换成 `base_link`；
- 可以最大程度减少对主滤波与建图逻辑的侵入。

建议改造为：

1. 在发布前先构造 `T_odom_livox`
2. 再由 `T_base_livox` 求出 `T_livox_base`
3. 最终得到 `T_odom_base`
4. 用 `T_odom_base` 填充 Odometry 与 TF

---

### 8.2 不建议继续复用当前 `set_posestamp(...)` 的“body 直接填充”语义

当前 `set_posestamp(...)` 直接把：

- `state_point.pos`
- `geoQuat`（当前对应 `body` 姿态）

写入消息。

这对于发布 `camera_init -> body` 没问题，但不适合直接拿来表达 `odom -> base_link`。

因此更合理的做法是：

- 保留这个函数作为旧逻辑参考；
- 或将其改造成“填充任意已计算 pose”的通用函数；
- 或者新增一个明确面向 `base_link` 输出的填充逻辑。

关键原则只有一个：

- **不要让“当前内部状态是 body”与“最终对外发布是 base_link”混在同一个未区分语义的函数里。**

---

### 8.3 对外 frame 命名统一成 `odom` 与 `base_link`

当前 FAST-LIO2 对外使用：

- `camera_init`
- `body`

为了与导航栈和 LeGO-LOAM 对齐，建议最终对外统一为：

- `header.frame_id = "odom"`
- `child_frame_id = "base_link"`

注意这里的“统一命名”必须建立在“数值语义也一起改对”的前提下。

不能只是把：

- `camera_init` 改名为 `odom`
- `body` 改名为 `base_link`

而不改变 pose 构造方式。

---

### 8.4 TF 广播必须同步改成 `odom -> base_link`

当前 `publish_odometry(...)` 内同时广播 TF，因此修改 Odometry 后，还必须同步修改：

- `trans.header.frame_id`
- `trans.child_frame_id`
- `trans.transform.translation`
- `trans.transform.rotation`

让它们与最终发布的 `T_odom_base` 完全一致。

否则会出现：

- 话题里 odom 一套语义
- TF 里另一套语义

这会让 LeGO-LOAM、RViz、TF 工具、其他下游节点得到互相冲突的位姿解释。

---

## 9. 推荐的实施顺序

### 步骤 1：明确 FAST-LIO2 内部保持不动的部分

本次改造 **不需要动** 以下部分：

- IMU 预积分 / 状态传播
- 点云去畸变
- 点云配准与地图更新
- EKF 主状态定义
- LiDAR-IMU 外参标定读取方式

也就是说：

- 算法主链不改；
- 只改“最终如何对外表达位姿”。

这会显著降低改造风险。

---

### 步骤 2：在发布层明确构造 `T_odom_livox`

这一步的目标是：

- 不再直接把 `state_point.pos` / `state_point.rot` 当成最终输出；
- 而是显式构造出 LiDAR 在 odom 下的位姿。

原因是这样可以把几何链写清楚：

```text
body -> livox -> base_link
```

同时也让后续调试更容易验证。

---

### 步骤 3：引入或读取 `T_base_livox`

FAST-LIO2 要完成 `odom -> base_link` 的最终构造，就必须知道当前系统约定的：

- `base_link -> livox_frame`

这一点有两种工程实现方式：

1. 在 FAST-LIO2 参数中显式配置一份同值外参；
2. 在运行时从 TF 中查询静态 `base_link -> livox_frame`。

如果只考虑本次目标与工程稳定性，推荐优先采用：

- **在 FAST-LIO2 自己的参数中显式持有这份外参**

原因：

- 发布 odom 不依赖外部 TF 查询时序；
- 启动顺序更可控；
- 调试时更直接；
- 不会因为 TF 尚未就绪而影响里程计首帧发布。

但无论采用哪种方式，语义必须唯一：

- FAST-LIO2 使用的 `T_base_livox` 必须与系统静态 TF 保持一致。

---

### 步骤 4：统一对外发布 `odom -> base_link`

这一步完成后，FAST-LIO2 对外输出应变成：

- Odometry：`odom -> base_link`
- TF：`odom -> base_link`

这样 LeGO-LOAM 才能把它当标准机器人主体里程计来使用。

---

### 步骤 5：LeGO-LOAM 切到外部 odom 模式

等 FAST-LIO2 侧完成后，LeGO-LOAM 侧只需要完成接入配置：

- `odom_type = wheel_odometry`
- 订阅 FAST-LIO2 对外发布的话题

这一步不属于 FAST-LIO2 改造本身，但它是最终联调闭环的必要条件。

---

## 10. 为什么不推荐本次先走 `body -> base_link`

理论上当然可以走：

```text
T_odom_base = T_odom_body * T_body_base
```

但对你当前项目来说，这不是第一优先选择，原因有三：

1. 你当前已经明确给出了 `base_link -> livox_frame`；
2. FAST-LIO2 内部已经可以构造 LiDAR pose；
3. `body -> base_link` 并不是系统里现成公开、统一使用的主静态外参。

因此如果本次目标只是“让 FAST-LIO2 达到 LeGO-LOAM 可用的标准输出”，那么最省事也最清晰的路线就是：

- **LiDAR pose -> base_link pose**

而不是：

- 先额外补一层 `body -> base_link`

---

## 11. 本次改造的主要风险点

### 11.1 只改 frame 名，不改 pose 数值

这是最严重的错误。

错误示例：

- 把 `child_frame_id` 从 `body` 改成 `base_link`
- 但 `pose` 仍然直接使用 `state_point.pos` / `state_point.rot`

这样发布出去的就只是：

- 名字叫 `base_link`
- 数值却还是 `body` 的位姿

这会导致整个系统静默地使用错误参考点。

---

### 11.2 `T_base_livox` 与 `T_livox_base` 方向搞反

当前 static TF 提供的是：

- `base_link -> livox_frame`

而最终计算需要的是：

- `livox_frame -> base_link`

如果方向错了，输出会整体发生固定偏差与姿态错误。

---

### 11.3 Odometry 与 TF 使用不同语义

如果出现：

- 话题中发布的是 `odom -> base_link`
- TF 还在广播 `camera_init -> body`

或者两者都叫 `odom -> base_link`，但数值不是同一套位姿，都会造成联调混乱。

因此本次改造必须把消息与 TF 一并收口。

---

### 11.4 与系统中其他 `odom -> base_link` 动态来源冲突

如果系统里已经有其他节点在广播同名动态 TF，例如底盘驱动，那么 FAST-LIO2 再广播同名链会产生冲突。

因此在联调阶段必须确保：

- 系统里只有一个主动态 `odom -> base_link` 来源。

---

## 12. 完成改造后的预期结果

改造完成后，FAST-LIO2 对外应满足：

### 12.1 话题层

- 发布 `nav_msgs/Odometry`
- `header.frame_id = "odom"`
- `child_frame_id = "base_link"`
- `pose = T_odom_base`

---

### 12.2 TF 层

- 动态广播：`odom -> base_link`
- 静态保留：`base_link -> livox_frame`

最终 TF 主链为：

```text
odom -> base_link -> livox_frame
```

---

### 12.3 对 LeGO-LOAM 的意义

此时 LeGO-LOAM 可以将 FAST-LIO2 看成一个标准外部里程计来源，而不需要理解 FAST-LIO2 内部的：

- `camera_init`
- `body`
- LiDAR-IMU 外参细节

这正是本次改造的价值所在：

- **把 FAST-LIO2 内部语义整理成导航系统可直接消费的标准机器人主体里程计。**

---

## 13. 本项目的推荐结论

对于当前项目，FAST-LIO2 的最佳改造方向是：

1. 保留 FAST-LIO2 主算法不动；
2. 在发布层显式构造 `T_odom_livox`；
3. 使用已知 `T_base_livox` 求得 `T_livox_base`；
4. 计算 `T_odom_base = T_odom_livox * T_livox_base`；
5. 对外统一发布标准 `nav_msgs/Odometry`：
   - `header.frame_id = "odom"`
   - `child_frame_id = "base_link"`
6. 同步广播一致的 TF：
   - `odom -> base_link`

这是当前项目里：

- 最贴合现有外参条件的方案；
- 对 FAST-LIO2 主算法侵入最小的方案；
- 最容易让 LeGO-LOAM 正确接入的方案；
- 也是最不容易在 frame 语义上留下隐患的方案。

---

## 14. 一句话总结

如果只看 FAST-LIO2 侧，你真正要做的不是“再发一条 odom”，而是：

- **把 FAST-LIO2 当前内部可得到的 LiDAR 位姿，严格换算成 `base_link` 位姿，并以标准 `odom -> base_link` 语义发布出去。**

只要这一步做对，LeGO-LOAM 使用它就会顺很多。