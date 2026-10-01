# LSN10 激光 + 飞控 IMU Cartographer 建图踩坑记录

> 从一个被删掉的 ROS1 工程出发，在 ROS2 Humble 上重新搭了一套带 IMU 的 2D 激光建图。
> 整个过程踩了四个坑：删 brltty 防黑屏 → 修解析器作用域 bug → 补 ros2 run 软链接 → 改雷达驱动的时间戳 bug。
> 记录得比较啰嗦，主要是给自己以后回头看的时候能想起当时怎么想的。

---

## 零、理论篇：为什么这样选传感器

> 这一节是自己学习机器人感知过程中的笔记，整理为什么要用激光雷达和 IMU 组合来建图，以及 Cartographer 在背后做了什么。

### 0.1 建图到底是什么问题

一个机器人在未知环境里移动，想要画出一张环境的地图——这听起来像是一个「画图」问题，但它实际上是一个**状态估计问题**。

机器人拿着传感器（激光雷达、IMU），每个时刻收到一堆数据。如果它精确知道自己每个时刻的位置，建图是平凡的：把激光数据按位姿拼到一起就行了。但现实是——没人告诉它自己的精确位置。

所以它必须**同时做两件事**：根据已有地图推断自己的位置，同时根据新的观测更新地图。这就是 SLAM（Simultaneous Localization And Mapping）。

用数学语言说，SLAM 在求解一个联合概率：

```
P(x₁ₜ, m | z₁ₜ, u₁ₜ)
```

其中 `x` 是机器人每个时刻的位姿（位置 + 朝向），`m` 是地图，`z` 是传感器观测，`u` 是运动控制量。这是个鸡生蛋蛋生鸡的循环——需要知道地图才能定位，需要知道位置才能画地图。

### 0.2 为什么用激光雷达做主传感器

激光雷达（LiDAR）输出的是**稀疏但有深度信息**的点云。相机给出的是**稠密但无深度信息**的像素。

对比一下几种传感器在室内建图场景下的优劣：

| 传感器 | 测距精度 | 更新频率 | 光照依赖 | 计算开销 | 适合场景 |
|--------|---------|---------|---------|---------|---------|
| 2D 激光雷达 | 毫米级 | 10-40Hz | 无 | 低 | 室内平面建图 |
| 双目相机 | 厘米～米级 | 30-60Hz | 强依赖 | 极高（GPU） | 室外/3D/语义 |
| RGB-D 相机 | 毫米～厘米级 | 30Hz | 依赖红外 | 中等 | 室内 3D |
| 超声波 | 厘米级 | 慢 | 无 | 极低 | 避障，不能建图 |

对于室内 2D 平面建图，激光雷达有几个不可替代的优势：

1. **距离精度高**：测距误差通常在 ±2cm 以内，远好于视觉的深度估计。这意味着每一帧激光数据是一个可靠的几何约束。
2. **不受光照影响**：不管白天黑夜、灯光闪烁、阴影变化，激光的飞行时间测距完全不受影响。视觉 SLAM 在暗光、过曝、白墙场景下非常容易丢。
3. **计算开销低**：一帧 2D 激光扫描大概 1000 个点，在 CPU 上做扫描匹配只需要几毫秒。同样的工作用视觉里程计做，需要提取特征、匹配、三角化、BA 优化，算力差一两个数量级。
4. **不会有尺度漂移**：视觉 SLAM 的一个经典问题是尺度模糊——单目相机无法确定真实世界的尺度，移动 1 米和移动 10 米在像素面上看起来是一样的。激光雷达直接给出米的距离，不存在尺度问题。

但是，激光雷达也有自己的短板。

### 0.3 为什么还要加 IMU

纯激光 SLAM 最大的弱点是**旋转估计不可靠**。

想象一下：机器人面对一堵笔直的白墙。每一帧激光打过去，都是一模一样的直线段。机器人沿着墙平移了几个厘米——激光能看到变化，可以纠正。但如果机器人原地转了一个小角度——激光打在墙上的点几乎完全一样，看不出区别。

简单说：

- **平移**：激光看得很清楚（墙的距离变了）
- **旋转**：激光几乎看不到（墙还是那面墙，角度差不影响距离）

这时候 IMU 上场了。

IMU（惯性测量单元）同时输出三种数据：

| 数据 | 物理含义 | 频率 | 优缺点 |
|------|---------|------|--------|
| 角速度 `ω` | 三轴旋转速率 (rad/s) | 100-1000Hz | 短时积分很准，但长期漂移 |
| 加速度 `a` | 三轴线加速度 (m/s²)，含重力 | 100-1000Hz | 能感知重力方向（绝对参考） |
| 姿态四元数 `q` | 融合后的朝向 | 100-1000Hz | 飞控内部解算完成 |

IMU 的优势正好弥补激光的短板：

1. **旋转敏感**：陀螺仪对角速度极其敏感，0.01 rad/s 的旋转也能测到。在激光看不清的「长廊」「白墙」场景，IMU 提供旋转约束。
2. **重力方向**：IMU 的加速度计永远知道「哪个方向是下」。这对建图非常关键——它能告诉 Cartographer：机器人没有歪，地图的 z 轴应该和重力方向对齐。没有 IMU 的纯激光建图，地图可能因为误差累积而倾斜。
3. **高频插值**：IMU 以 1000Hz 更新，激光只有 10Hz。两次激光帧之间的 100ms 里，IMU 提供了连续的角速度和加速度，让 Cartographer 能精确外推机器人的位姿变化。

### 0.4 Cartographer 是怎么融合两者的

Cartographer 是 Google 开源的 SLAM 方案，核心算法是「scan-to-submap matching」，也就是把每一帧激光扫描和已构建的局部子图做配准，找到最优的刚体变换。

IMU 在里面的角色分三层：

**第一层：位姿外推（Pose Extrapolator）**

```lua
use_pose_extrapolator = true
```

两帧激光之间隔了 100ms。这 100ms 里机器人可能在动，Cartographer 不能假设激光来时机器人的位姿就是上一帧的位姿。它用 IMU 的角速度和线加速度，从上一帧位姿外推当前帧位姿，给扫描匹配提供一个**更好的初始猜测**。初始猜测越准，扫描匹配收敛越快，精度越高。

**第二层：扫描匹配的旋转约束**

```lua
TRAJECTORY_BUILDER_2D.use_imu_data = true
```

在 Ceres 扫描匹配器求解最优位姿时，Cartographer 会把 IMU 的角速度积分结果加进优化问题，作为旋转分量的惩罚项。这意味着：

- 如果激光数据说「机器人转了吗？好像没转」（白墙场景）
- 但 IMU 说「我角速度积分下来确实转了 5°」
- 优化器会更多信任激光的平移、更多信任 IMU 的旋转

这就克服了纯激光在旋转上的不足。

**第三层：重力对齐**

IMU 的加速度计测量的是「比力」，静止时读数指向天（重力反方向）。Cartographer 用这个信息保证建图结果的 z 轴和真实重力方向一致。没有 IMU 的话，如果机器人走过一个斜坡，地图的「水平线」可能跟着坡倾斜。

### 0.5 为什么选 Cartographer：主流 2D SLAM 算法对比

> 不是 Cartographer 有多完美，而是在这个场景下它最合适。做技术选型最怕的是「别人用我也用」——这里把对比过什么、为什么不选、为什么选，都掰开来讲清楚。

先看整个 2D SLAM 的算法族谱。2D 激光 SLAM 大致分两派：**滤波派**和**图优化派**。

**滤波派**（Filter-based）把 SLAM 建模为一个递归贝叶斯估计问题。每个时刻只维护当前位姿和地图的后验概率，新的观测进来就更新，旧的扔掉。优点是计算量恒定，缺点是误差会累积、无法回头修正历史错误。

**图优化派**（Graph-based）把 SLAM 建模为一个图结构——节点是历史位姿，边是传感器约束。每次加新数据就加节点和边，然后用非线性优化全局调整所有节点的位姿。优点是精度高、能做回环检测，缺点是计算量和内存随时间和地图大小增长。

下面是 2D 激光 SLAM 里最常见的四个方案：

| 算法 | 方法论 | 核心思路 | 回环检测 | 精度 | 计算需求 |
|------|--------|---------|---------|------|---------|
| **Gmapping** | 滤波派 | 粒子滤波 + 栅格地图 | ❌ 无 | 中 | 低（CPU 单核） |
| **Hector SLAM** | 滤波派 | 高斯-牛顿扫描匹配 + 多分辨率地图 | ❌ 无 | 中低 | 极低 |
| **Karto SLAM** | 图优化派 | 稀疏位姿图 + 非线性最小二乘 | ✅ 有 | 高 | 中 |
| **Cartographer** | 图优化派 | 扫描到子图匹配 + 分支定界回环检测 | ✅ 强 | 最高 | 高（多线程） |

逐个分析为什么选或不选。

#### Gmapping：经典但不适合现在

Gmapping 是 2007 年的方案，基于 Rao-Blackwellized 粒子滤波器。每个粒子包含机器人轨迹的一个假设和一个对应的栅格地图。通过重采样机制淘汰「画得不好的」粒子，最终保留最一致的那个。

它的优点：实现简单，ROS1 时代是标配，跑起来只需要一个单核 CPU。

但它有几个硬伤：

1. **没有回环检测**。如果机器人绕了一大圈回到起点，Gmapping 永远不知道「我回来了」——地图在回环处会错位，像两张撕开的纸拼错了一样。对于稍大一点的环境（几百平米以上），这是致命的。

2. **粒子数量随面积爆炸**。每个粒子都要维护一份完整的栅格地图。环境越大，需要的粒子越多，内存不是线性增长，而是 `N_粒子 × 地图_分辨率`。

3. **ROS2 没有官方移植**。Gmapping 的 ROS2 版本是社区自己改的，维护不活跃，功能也不完整。

总结：Gmapping 适合小房间、单次建图、不需要回环检测的场景。我们这里至少要画几百平米的楼层，Gmapping 顶不住。

#### Hector SLAM：太依赖激光质量

Hector 是纯扫描匹配方案——它根本不用里程计，只靠高斯-牛顿法把当前帧激光配准到已有地图上。因为不依赖外部里程计，它很适合**无人机**这种里程计不可靠的平台。

但它有一个前提条件：**激光扫描速率必须足够高**。Hector 要求激光帧率在 20Hz 以上（最好 40Hz），因为两帧之间纯靠扫描匹配连起来，如果帧率太低、两帧之间机器人移动太多，匹配初始化差太远，直接发散。

我们的 LSN10 只有 10Hz。Hector 在 10Hz 雷达上表现非常差，稍微转快一点就丢。

而且 Hector 也没有回环检测。

总结：Hector 适合高帧率雷达（如 Hokuyo 40Hz）+ 无里程计的场景。我们的条件一个都不满足。

#### Karto SLAM：方案对，但太老了

Karto 的思路其实和 Cartographer 很接近：也是图优化 + 稀疏位姿图。它最早提出了「扫描匹配 + 回环检测 + 全局优化」这条路线。

但它的扫描匹配用的是相关匹配（correlative scan matching），在 2010 年代还行，放到现在精度和速度都不如 Cartographer 的双阶段方案。

而且 Karto 已经停止维护十多年了，ROS 版也只是第三方移植，ROS2 更是没有。

总结：思路上是 Cartographer 的前辈，但技术上已经被全面超越。

#### Cartographer：图优化的工业级实现

Cartographer 是 Google 在 2016 年开源的，在 Karto 的基础上做了两件最重要的事：

1. **扫描匹配升级为「扫描到子图」**。Cartographer 不是把当前帧和「整个地图」匹配，也不是和「上一帧」匹配，而是和一个**最近的局部子图**（submap，包含最近若干帧的拼接结果）做匹配。子图的质量远高于单帧，匹配的成功率和精度自然更高。子图一旦建好就锁定不变，只优化位姿图，不重算子图——这大大节省了计算量。

2. **用分支定界做快速回环检测**。回环检测的暴力方案是对所有历史位姿做全量匹配——O(n²) 复杂度，帧数多了直接爆炸。Cartographer 用分支定界（branch-and-bound）把搜索空间按树形结构剪枝，能在对数时间内找到候选回环，然后做精确校验。这是它能在几千帧后依然跑得动的关键。

另外，Cartographer 的 IMU 融合是原生支持的——不是事后补丁，而是从架构上就设计为多传感器融合。这对我们这种「激光 + 飞控 IMU」的组合是天然的契合。

Cartographer 的缺点：**参数多、调参难**。尤其是 `submaps.num_range_data`、`ceres_scan_matcher` 的权重、`motion_filter` 的阈值，调不好建图质量很差。但这是开源方案的通病——能力越强，越需要理解才能用好。

#### 选 Cartographer 的逻辑链

```
有回环检测 ← 环境大，必须的
    ↓
原生 IMU 融合 ← 飞控已经外发 IMU 数据，不能浪费
    ↓
图优化精度高 ← 需要生成用于导航的精确地图
    ↓
ROS2 官方支持 ← Humble 的 cartographer_ros 是 Google 维护的
    ↓
Cartographer
```

反过来看为什么不选其他：

```
Gmapping   → 没有回环检测 ✗
Hector     → 激光只有 10Hz ✗
Karto      → ROS2 没有维护 ✗
```

### 0.6 Cartographer 算法内部：从激光点到地图的过程

> 这一节把 Cartographer 的内部流水线走一遍。不是翻译论文，而是从「数据进来→地图出去」的视角，理解每一步在做什么、和我们配的那些 Lua 参数有什么关系。

Cartographer 的建图分两个阶段：**局部 SLAM**（前端）和**全局 SLAM**（后端）。局部 SLAM 负责快速构建子图和估计位姿，全局 SLAM 负责回环检测和全局优化。两者并行跑在不同的线程上。

#### 阶段一：局部 SLAM — 扫描到子图匹配

第一步，一帧新的激光扫描进来（`/scan` 消息，10Hz）。

**Step 1：运动滤波**

```lua
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 3.
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = 0.004
```

不是每一帧激光都会被处理。如果机器人几乎没动（距离移动 < 0.1m、角度变化 < 0.004 rad），Cartographer 会选择**跳过这一帧**——因为冗余数据只会浪费算力，对子图质量没有任何提升。超过 3 秒没处理的话，不管怎么样都会强制处理一帧。

我们这里的 0.1m 和 0.004 rad 是比较保守的设置，适合慢速精细建图。如果移动快可以适当放宽。

**Step 2：位姿外推**

```lua
use_pose_extrapolator = true
imu_sampling_ratio = 1.
```

两帧激光之间隔了 100ms（10Hz 雷达）。机器人在这 100ms 里移动了，Cartographer 需要知道「收到这帧激光时，机器人大概在什么位姿」。它用 IMU 的角速度和上一帧的线速度，从上一帧的位姿外推出当前帧的初始猜测。

这个初始猜测非常重要——扫描匹配是局部优化，如果初始猜测差太远，优化很容易掉进局部极小值。IMU 给的初始猜测越准，匹配成功率越高。

**Step 3：扫描到子图匹配（Ceres 非线性优化）**

这是局部 SLAM 的核心。问题表述为：

```
给定：一帧激光扫描 + 一个已构建的子图 + 一个初始位姿猜测
求解：最优的刚体变换 T(x, y, θ) = 新扫描在子图坐标系下的位姿
```

Ceres 求解器构建一个最小二乘问题：

```
min Σ(1 - 占据概率(变换后的激光点))²
```

通俗解释：把当前帧的激光点「搬到」子图坐标系里，看这些点落在子图的「墙」上（占据概率高的格子）的数量。数量越多，说明正在对齐的位姿越准。Ceres 用非线性优化（梯度下降的变种）迭代调整 (x, y, θ)，直到找到最好的对齐。

几个关键权重：

```lua
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 20.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.
```

- **occupied_space_weight**：控制「激光点落在已有占据格子里」的权重。设高了会让匹配更激进地往已有墙上靠，但可能过度拟合噪声。
- **translation_weight** 和 **rotation_weight**：分别是平移和旋转的代价。注意这里的 `rotation_weight = 40` 是 `translation_weight = 20` 的两倍——这反映了我们对 IMU 旋转信息的信任。IMU 陀螺仪的旋转测量比平移估计更准，所以优化器在旋转方向上更「保守」，更信任初始猜测，在平移方向上更「自由」，让激光数据去调整。

**Step 4：插入子图**

优化完的位姿确定了这一帧激光在子图坐标系下的位置。把激光点击中到的栅格标记为「占据」，光束穿过的栅格标记为「空闲」。一帧一帧叠加，子图逐渐显现。

```lua
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90  -- 默认值
```

一个子图包含 90 帧激光扫描（约 9 秒的数据 @ 10Hz）。子图一旦包含足够帧数就「完成」了——它的栅格内容锁定不变，之后的全局优化只调整子图的整体位姿，不修改内容。

**Step 5：位姿发布**

```lua
pose_publish_period_sec = 5e-3          -- 5ms
trajectory_publish_period_sec = 30e-3   -- 30ms
```

Cartographer 不断对外发布当前的位姿估计和完整轨迹。这些位姿通过 TF 广播 `map → odom → base_link`。

#### 阶段二：全局 SLAM — 回环检测与位姿图优化

局部 SLAM 在进行的同时，全局 SLAM 在后台做两件事。

**Step 6：回环检测**

```lua
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6
```

机器人走了一大圈回到之前经过的位置。当前的激光帧和某个**已经完成的旧子图**之间，应该存在一个匹配。全局 SLAM 不断扫描已完成的子图，寻找和当前帧匹配的候选。

Cartographer 的回环检测不用暴力搜索（O(n²)），而是用**分支定界（branch-and-bound）**：把搜索区域（x, y, θ 三个维度）按分辨率从粗到细划分成树状结构。先在粗分辨率上快速排掉不可能的位姿（bound 操作），再往细的分支深入搜索（branch 操作）。这能在大规模场景下保持实时。

`min_score = 0.55` 是回环匹配的最低得分——匹配得分低于这个值就认为不是真的回环，丢掉。`global_localization_min_score = 0.6` 是全局重定位的得分门槛——如果机器人完全不知道自己在哪里（比如被人抱起来换了个位置），需要把这个分数设高，避免误匹配。

**Step 7：位姿图优化**

```lua
POSE_GRAPH.optimize_every_n_nodes = 90
POSE_GRAPH.optimization_problem.huber_scale = 1e1
```

当检测到回环时，会在位姿图中插入一条新的约束边（「帧 A 和子图 B 匹配上了，它们之间的相对位姿是 T」）。有了新约束，整个位姿图需要重新优化——调整所有节点的位姿，使得约束边的残差平方和最小。

这和前面 Ceres 的优化是同一个数学框架，但规模大得多：不是单帧的 (x, y, θ)，而是几百个节点的联合优化。`optimize_every_n_nodes = 90` 表示每插入 90 个新节点就触发一次全局优化——不用每帧都做，因为全局优化的开销远大于局部 SLAM。

`huber_scale = 1e1` 是 Huber 损失函数的阈值——对残差中特别大的离群值（可能来自错误的回环匹配）施加 L1 惩罚而不是 L2，防止一个错误回环把整个地图扯歪。

**Step 8：输出占据栅格地图**

```lua
submap_publish_period_sec = 0.3
```

`cartographer_occupancy_grid_node` 把所有完成的子图拼接起来，转换成标准的 `nav_msgs/OccupancyGrid`，发布到 `/map`。这是给 Nav2 导航用的标准地图格式。

#### 整个流水线总结

把以上步骤串起来：

```
/scan (10Hz)
  │
  ├─ motion_filter ── 动得不够？跳过，不处理
  │
  ├─ PoseExtrapolator ← /imu/data (1090Hz)
  │    └─ 外推初始位姿
  │
  ├─ CeresScanMatcher (局部 SLAM)
  │    └─ min Σ(1 - 占据概率(T × 激光点))²
  │    └─ 输出: 当前帧的精确位姿
  │
  ├─ Submap: 插入激光点，更新占据栅格
  │
  ├─ BranchAndBound (全局 SLAM，后台)
  │    └─ 扫描已完成子图，寻找回环候选
  │    └─ 发现回环 → 插入约束边
  │
  ├─ PoseGraph (全局优化)
  │    └─ 调整所有节点位姿，最小化全局误差
  │
  └─ cartographer_occupancy_grid_node
       └─ 拼接子图 → /map (OccupancyGrid)
```

其中，IMU 的贡献贯穿局部 SLAM 的全程：初始猜测靠 IMU 外推，Ceres 优化里旋转权重靠 IMU 约束，重力对齐靠 IMU 加速度。去掉 IMU 的话，这个流水线的前两步精度会明显下降。

### 0.7 tracking_frame 为什么选 imu_link

```lua
tracking_frame = "imu_link"  -- 而不是 "laser"
```

Cartographer 用 `tracking_frame` 来跟踪机器人的运动。选择哪个坐标系，直接决定了哪个传感器「看到」的运动被信任。

选 `imu_link` 的逻辑：

1. **IMU 直连物理运动**：陀螺仪直接测量角速度，不依赖外部环境。不管白墙还是玻璃，IMU 始终知道自己在转。
2. **激光要通过 TF 换算**：如果 tracking_frame 选 `laser`，Cartographer 看到的运动是激光坐标系下的运动。但激光坐标系和机器人 base 之间的 TF 是机器人运动的一部分——这会导致循环依赖。
3. **IMU 频率高两个数量级**：IMU 1kHz vs 激光 10Hz。跟踪帧用高频源，两帧之间丢失的信息更少。

ROS1 的老配置就是选 `imu_link`，是验证过的实践。

### 0.8 传感器协同的完整图景

把整个感知管线串起来看：

```
IMU (1kHz)
  │ 角速度 → 位姿外推 → 扫描匹配初始猜测
  │ 加速度 → 重力方向 → 地图对齐
  │
激光 (10Hz)
  │ 距离测量 → 扫描匹配 → 更新子图
  │
Cartographer
  │ 位姿优化 → /tf: map → odom → base_link
  │ 子图拼接 → /map (占据栅格)
```

激光提供**长期的、漂移小的绝对几何约束**（墙在哪里、障碍物在哪里）。IMU 提供**短期的、高频的相对运动约束**（我在转、我在加速、重力在那边）。两者互补，缺谁都会出问题。

纯激光建图的问题不是不能建——也能建——而是地图容易漂、转角处精度差、长廊场景容易丢。加了 IMU 后，这些场景的鲁棒性有质的提升。

---

## 一、背景

手里有一套设备：

- **LSN10 镭神激光雷达**，串口协议，10Hz，发布 `/scan`
- **ANO 匿名飞控**，STM32F407，通过 CH340 USB 转串口连电脑，921600bps，用私有协议（ANO PT v7）往外吐 IMU 数据
- **电脑上已有一个 `wheeltec_ros2` 工作空间**，但只用激光建图，IMU 被显式关掉了（`use_imu_data = false`）

之前这套设备在 ROS1 Noetic 上跑过一个完整方案：飞控串口 → `anorosdt` 包解析 → 发布 `ano_imu` → Cartographer 用 `imu_link` 做 tracking_frame。但那个 ROS1 工程已经被删了，只剩下 `~/ros2/参考/anorosdt_ws/` 下面的源代码和一些 launch 文件。

目标很简单：**把这套东西在 ROS2 上重新跑起来，让 Cartographer 能吃 IMU 数据**。

---

## 二、分析对比：ROS1 是怎么做的，ROS2 差了什么

### 2.1 先看 ROS1 的启动方式

在 `~/ros2/参考/` 下面找到了 ROS1 的启动脚本 `self.sh`：

```bash
source /opt/ros/noetic/setup.bash
source /home/uu20/catkin_ws/devel/setup.bash
source /home/uu20/anorosdt_ws/devel/setup.bash

gnome-terminal -- bash -c "roslaunch anorosdt cartographer_imu_dt.launch;exec bash"
```

`cartographer_imu_dt.launch` 里面干了四件事：

1. `<include file="anoros_dt.launch"/>` — 启动飞控串口桥接节点
2. `<include file="Lidar.launch"/>` — 启动激光雷达驱动
3. `<include file="cartographer_ros/launch/demo_revo_lds_imu.launch"/>` — 启动 Cartographer
4. `<node name="rviz" pkg="rviz" type="rviz"/>` — 可视化

### 2.2 ROS1 的飞控桥接包 `anorosdt`

这是一个 C++ 包，做三件事：

1. **打开串口**：`/dev/ttyUSB0`，`921600bps`，在 `config.yaml` 里配置
2. **解析 ANO PT v7 协议**：
   - 帧头 `0xAA`，广播地址 `0xFF`
   - 帧 ID `0x01`：加速度 + 陀螺仪（int16 × 3 组 = 12 字节 payload）
   - 帧 ID `0x04`：四元数（int16 × 4 = 8 字节 payload）
   - 校验：SC1 是前 N 字节累加和模 256，SC2 是 SC1 的累加和模 256
3. **发布话题**：`ano_imu`（`sensor_msgs/Imu`），frame_id = `imu_link`

数据转换公式（从 `anoSerial.cpp` 里面抄的）：

```
加速度: raw_int16 / 100.0 = m/s²
角速度: raw_int16 / 16.384 × π / 180.0 = rad/s
四元数: raw_int16 / 10000.0
```

### 2.3 ROS2 当前的差距

打开 `wheeltec_ros2/src/lslidar_driver/config/lsn10.lua`：

```lua
tracking_frame = "laser",          -- ROS1 用的是 "imu_link"
TRAJECTORY_BUILDER_2D.use_imu_data = false  -- ROS1 里是 true
```

再看 `lsn10_cartographer.launch.py`，完全没有：

- 飞控桥接节点
- IMU 话题的重映射
- URDF 里的 `imu_link`

结论很直接：**ROS2 这边差一个能读飞控串口、解析 ANO PT v7 协议、发布 `sensor_msgs/Imu` 的节点，以及对应的配置修改**。

---

## 三、动手：写飞控 IMU 桥接节点 `anorosdt2`

### 3.1 为什么用 Python 而不是 C++

ROS1 的 `anorosdt` 用 C++ 写的，但代码量并不大（核心就 `anoSerial.cpp` + `anoPublisher.cpp`，加起来不到 500 行）。用 Python 重写有几点好处：

- 串口读取在 Python 里直接用 `pyserial`，不需要自己管理缓冲区
- 解析逻辑直接用 `struct.unpack` 一把搞定，比 C++ 的指针强转可读性好太多
- ROS2 的 Python 包不用写 CMakeLists，`setup.py` + `setup.cfg` 就能搞定
- 这种 1kHz 级别的串口数据，Python 性能绰绰有余

### 3.2 包结构

```
anorosdt2/
├── package.xml              # ament_python
├── setup.py                 # 数据文件 + 自定义安装
├── setup.cfg                # entry_points 注册可执行文件
├── config/anorosdt2.yaml    # 可配置参数
├── launch/                  # (预留)
├── resource/anorosdt2       # ament index 标记文件
└── anorosdt2/
    ├── __init__.py
    └── anoros_dt_node.py    # 核心节点
```

### 3.3 协议解析器的状态机

ANO PT v7 协议是字节流协议，帧之间没有分隔符，只能靠 `0xAA` 帧头来同步。解析器用一个 7 状态的状态机逐字节处理：

```
STATE_HEADER (0)  等待 0xAA
STATE_ADDR   (1)  读取目标地址（必须是 0xFF 广播）
STATE_ID     (2)  读取帧 ID
STATE_LEN    (3)  读取数据长度
STATE_DATA   (4)  读取 payload
STATE_SC1    (5)  读取校验字节 1
STATE_SC2    (6)  读取校验字节 2 → 验证 → 回调
```

这里面有一个踩坑点：**SC1 和 SC2 必须用实例变量存，不能用局部变量**。因为状态机的 `feed()` 每次只处理一个字节，SC1 在状态 5 被读进来，但校验逻辑在状态 6 才执行——两个状态之间跨越了两次 `feed()` 调用。如果用局部变量 `sc1 = data`，下次 `feed()` 进来的时候 `sc1` 已经没了，直接 `UnboundLocalError`。

这是写状态机的一个经典坑：**状态跨调用时的数据都要挂在实例上**。

---

## 四、踩坑一：brltty 抢串口导致系统黑屏

### 4.1 现象

飞控插上 USB 后几分钟，电脑直接黑屏，只能硬重启。

### 4.2 排查

重启后翻 `journalctl`，找到关键线索：

```
brltty[3029]: USB URB status error 108: 无法在传输端点关闭以后发送
brltty[3029]: USB bulk transfer error 19: 没有那个设备
```

时间线和 CH340 驱动加载（`ch341-uart converter now attached to ttyUSB0`）完全吻合。

### 4.3 根因分析

`brltty` 是 Linux 的**盲文显示器支持服务**，Ubuntu 默认安装。它有一个非常讨厌的特性：会主动探测所有 USB 串口设备，试图判断它们是不是盲文显示器。CH340/CH341 芯片恰好是盲文显示器常用的通信芯片之一，所以 `brltty` 会直接打开它、发配置指令——

然后和飞控的正常数据通信产生冲突，USB 栈崩溃 → 整个 USB 子系统挂掉 → Xorg 被拖垮 → 黑屏。

这个 bug 在 GitHub 上挂了快十年了，受害者覆盖 Arduino/STM32/CH340/CP2102 几乎所有 USB 串口用户。

### 4.4 解决

```bash
sudo apt purge brltty -y
sudo reboot
```

一句话总结：**Ubuntu 上但凡要用 USB 转串口，先把 brltty 删了再说**。

---

## 五、踩坑二：`ros2 run` 找不到可执行文件

### 5.1 现象

编译成功了，`install/anorosdt2/bin/anoros_dt` 也在，但：

```bash
$ ros2 run anorosdt2 anoros_dt
No executable found
```

### 5.2 排查

`which anoros_dt` 能找到。包也在 `ros2 pkg list` 里面。`ros2 pkg executables anorosdt2` 为空。

问题出在哪里？`ros2 run` 查找可执行文件的路径不是 `bin/`，而是 `<prefix>/lib/<package_name>/`。也就是说它期望在 `lib/anorosdt2/anoros_dt` 找到可执行文件，而不是 `bin/anoros_dt`。

### 5.3 根因

这是 ament_python 和 ament_cmake 的一个历史差异。CMake 包的输出在 `lib/<pkg>/`，所以 `ros2 run` 优先查那里。Python 包的 `console_scripts` 默认装到 `bin/`，`ros2 run` 找不到。

### 5.4 解决

在 `setup.py` 里加了一个 `CustomInstall` 类，编译完成后自动在 `lib/<pkg>/` 下面创建指向 `bin/` 的软链接：

```python
class CustomInstall(install):
    def run(self):
        install.run(self)
        lib_dir = os.path.join(self.install_base, 'lib', package_name)
        os.makedirs(lib_dir, exist_ok=True)
        os.symlink('../../bin/anoros_dt', os.path.join(lib_dir, 'anoros_dt'))
```

---

## 六、验证 IMU 数据

接上飞控后，直接用 Python 脚本读原始字节确认数据在流动：

```python
ser = serial.Serial('/dev/ttyUSB0', 921600)
data = ser.read(1024)  # 一瞬间就收满
# 能看到大量 aaff 帧头
```

然后用修复后的解析器测试：

```
Accel: x=0.60, y=-1.82, z=9.99     # z≈1g，传感器平放，正确
Gyro:  x=-0.017, y=0.015, z=-0.002  # 几乎为0，传感器静止，正确
Quat:  w=0.749, x=-0.126, y=0.114, z=-0.638  # 非平凡姿态，飞控在解算
```

ROS2 节点也正常工作了：

```bash
$ ros2 topic hz /imu/data
average rate: 1091.493  # 飞控 1ms 发一次，实际到 ROS 大约 1090Hz
```

---

## 七、踩坑三：Cartographer 丢弃所有激光数据

这是一个典型的「数据没人看，直到下游报错才追」的案例。也是这次踩的最深的一个坑——需要追到 LSLIDAR 驱动的 C++ 源码才能定位。

### 7.1 现象

一切启动正常：节点都在跑，话题都有数据在飞，IMU 1090Hz 稳稳的，激光 10Hz 也正常。但 `/map` 打死不出来，Rviz 永远报 `Frame [map] does not exist`。

翻了 Cartographer 的终端输出，看到刷屏的警告：

```
[cartographer logger]: W0703 13:43:48.000000 sensor_bridge.cpp:211]
Ignored subdivision of a LaserScan message from sensor scan because
previous subdivision time 647267391820023398 is not before
current subdivision time 639186542280522082
```

### 7.2 第一层分析：这个警告在说什么

警告文件 `sensor_bridge.cpp`，第 211 行。去 Cartographer 源码里翻，这段逻辑大致是：

> 对于每一个 LaserScan 消息，Cartographer 会把它拆成多个「子段」（subdivision），每个子段包含若干个连续的激光点。每个子段会被分配一个时间戳。这些子段的时间戳必须严格递增——如果第 N+1 个子段的时间戳比第 N 个还要早（或相等），Cartographer 就认为这个 LaserScan 消息的时间信息不可靠，直接丢弃。

丢一帧两帧问题不大，但**每帧都丢**，那就永远凑不出第一个子图，`/map` 当然出不来。

现在看警告里的两个数字：

```
previous: 647267391820023398
current:  639186542280522082
```

单位是纳秒。首先，`previous > current`，违反了严格递增的要求——这解释了为什么被丢掉。

但更奇怪的是这两个时间戳的绝对值。换算一下：

```
647267391820023398 ns ÷ 10⁹ = 647,267,391 秒 ≈ 20.5 年
639186542280522082 ns ÷ 10⁹ = 639,186,542 秒 ≈ 20.3 年
```

两个时间都在 20 年后。当前 Unix 时间戳大约是 17.8 亿秒——差了整整 11 亿秒。这不可能是 ROS 的系统时间。

所以问题一定不在 `header.stamp`（ROS 系统时间是准的），而在于**消息内部的 `time_increment`**——也就是相邻两个激光点之间的时间间隔。

### 7.3 第二层分析：`time_increment` 是从哪来的

先理解 LaserScan 消息的时间模型。

`sensor_msgs/LaserScan` 有两个和时间相关的字段：

```
header.stamp       — 这一帧扫描的第一个点的时间戳（ROS 时间）
time_increment     — 相邻两个点之间的时间间隔（秒）
```

Cartographer 计算第 i 个点的时间戳时用：

```
point_i_timestamp = header.stamp + i × time_increment
```

如果 `time_increment` 是天文数字，后面所有点的时间戳都会飞到未来。

那么 `time_increment` 是怎么被赋值的？追到 LSLIDAR 驱动源码 `lslidar_driver.cc`。

### 7.4 第三层分析：追到驱动源码里的 bug

LSLIDAR 驱动有一个函数叫 `getScan()`，用来获取一帧扫描的元信息。简化后的代码是这样的：

```cpp
// lslidar_driver.cc 第 480 行附近
int LslidarDriver::getScan(
    std::vector<ScanPoint> &points,
    rclcpp::Time &scan_time,    // 输出：整帧扫描的时间
    float &scan_duration)       // 输出：整帧扫描的耗时（秒）
{
    scan_time = pre_time_;                                       // ①
    scan_duration = time_.seconds() - pre_time_.seconds();       // ②
}
```

这里 `pre_time_` 和 `time_` 都是 `rclcpp::Time` 类型的成员变量。在驱动每次收完一帧雷达数据时，会调：

```cpp
pre_time_ = time_;              // 上次的时间
time_ = get_clock()->now();     // 当前 ROS 时间
```

所以 `time_` 是「刚刚收完这一帧的时刻」，`pre_time_` 是「上一次收完的时
刻」——它们的差 `scan_duration` 就是两帧之间的真实耗时，对 10Hz 雷达来说大约是 0.1 秒。这部分逻辑是对的。

问题出在第 ① 行。来看调用方怎么用 `scan_time`：

在 N10 型号的代码路径里（第 1165-1166 行）：

```cpp
float scan_time;                        // 注意：这里声明的类型是 float
this->getScan(points, start_time, scan_time);  // rclcpp::Time 隐式转换给 float
// ...
scan->scan_time      = scan_time;                                 // ③
scan->time_increment = scan_time / (double)(count_num - 1);       // ④
```

第 ① 行 `scan_time = pre_time_` 是一次隐式类型转换。`pre_time_` 是 `rclcpp::Time`，`scan_time` 是 `float`。C++ 编译器会怎么处理？

`rclcpp::Time` 类重载了 `operator float()` 吗？追一下 rclcpp 源码——没有。但它有 `nanoseconds()` 方法返回 `uint64_t`，而这个值可以被隐式转为 `float`。

**结论：`scan_time = pre_time_` 的结果不是纳秒数转成秒数，而是直接把纳秒数这个大整数赋值给了 float。**

举个例子说明这个差异到底有多大：

```
当前 ROS 时间：1,783,050,000 秒
                = 1,783,050,000,000,000,000 纳秒（约 1.78 × 10¹⁸）

如果赋值的是秒数 → 1.78 × 10⁹ → 合理的浮点值
如果赋值的是纳秒数 → 1.78 × 10¹⁸ → 超出 float 的精确表示范围，已经丢精度了
```

纳秒数放到 `float`（单精度，有效数字约 7 位）里，1.78 × 10¹⁸ 这个数已经远超过 float 的精确表示范围，尾数部分直接被截断。

然后到了第 ④ 行，这个已经失真的大数除以点数（约 1000）：

```
scan->time_increment = 1.78 × 10¹⁸ / 1000 = 1.78 × 10¹⁵ 纳秒 ≈ 20.6 天
```

也就是说，**Cartographer 被告诉：这帧激光的每个相邻点，时间上差了 20 天**。

于是 Cartographer 忠实地计算：

```
point_0_timestamp = now
point_1_timestamp = now + 20天
point_2_timestamp = now + 40天
...
```

然后把它拆成子段时，发现「第 N+1 个子段的起始时间」比「第 N 个子段的结束时间」还要早——因为 `time_increment` 太大了，子段之间根本没有可比性——全部丢掉。

### 7.5 第四层分析：为什么其他型号没这个 bug

更有意思的是，同一个文件里，`N10_P` 和 `M10_DOUBLE` 型号的代码路径已经被修过了。

在第 1007-1008 行：

```cpp
// 对于 N10_P / M10_DOUBLE：
// scan->scan_time = scan_time;
// scan->time_increment = scan_time / (double)(count_num);
```

注意这两行已经**被注释掉了**。

但第 1165-1166 行，N10（也就是 LSN10）走的 else 分支：

```cpp
// 对于 N10：
scan->scan_time      = scan_time;                            // ← 没注释
scan->time_increment = scan_time / (double)(count_num - 1);  // ← 没注释
```

为什么会出现这种「修一半」的情况？大概率是厂商的开发者先发现在 N10_P 上 `time_increment` 的计算有问题（可能是 N10_P 用户报的 bug），修完之后只测了 N10_P，确认没问题就发了新版本。N10 的那个分支在 `else` 块里，他们要么没注意到，要么觉得「N10 没改过，应该不需要改」。

这是典型的「复制粘贴代码 + 只修一个分支」引发的不对称 bug。

### 7.6 为什么不设 `time_increment` 反而是对的

修复方式：照抄 N10_P 的处理，注释掉这两行。

```cpp
// scan->scan_time = scan_time;
// scan->time_increment = scan_time / (double)(count_num - 1);
```

不设 `time_increment` 意味着 Cartographer 认为所有激光点共享同一个时间戳（即 `header.stamp`）。

对 2D 激光雷达来说，这反而是正确的行为。原因：

1. **2D 激光雷达的扫描速度远快于机器人移动速度**。LSN10 的扫描频率是 10Hz，一帧的 360° 扫描在 100ms 内完成。但激光内部的旋转镜面速度大约是 600-1200 RPM（每秒 10-20 转），单个激光脉冲在几微秒内完成测距。所有点可以认为是「瞬时的」——至少对 Cartographer 的 2D 来说精度足够了。

2. **2D Cartographer 不使用 `time_increment`**。在 Cartographer 的 sensor_bridge 里，2D 模式下激光点的子时间戳主要用于「运动畸变去除」，即补偿机器人在扫描过程中本身的移动。但如果 `tracking_frame` 设置正确（`imu_link`），IMU 提供的高频位姿外推已经足够补偿运动畸变了，不需要再靠 `time_increment` 细分。

3. **设置错误的值不如不设**。`time_increment = 0` 会让 Cartographer 跳过时间戳细分。`time_increment = 20天` 则会导致全部数据被丢弃。在这两者之间，设为 0 明显是更好的选择。

重新编译后，移动雷达，`/map` 立刻出现了。

---

## 八、最终结果

### 8.1 系统架构

```
飞控 CH340 (/dev/ttyUSB0, 921600)
  │ ANO PT v7 帧 (ID 0x01 加速度+陀螺仪, ID 0x04 四元数)
  ▼
anorosdt2 ──► /imu/data (1090Hz, frame_id: imu_link)

雷达 CH343 (/dev/ttyACM0, 230400)
  │ LSN10 协议
  ▼
lslidar_driver ──► /scan (10Hz, frame_id: laser)

robot_state_publisher: base_link → imu_link (fixed)
                       base_link → laser (fixed)

Cartographer:
  tracking_frame = "imu_link"
  use_imu_data = true
  num_laser_scans = 1
  ──► /map (占据栅格地图)
  ──► /tf: map → odom → base_link
```

### 8.2 启动方式

```bash
source ~/ros2/wheeltec_ros2/install/setup.bash
ros2 launch lslidar_driver lsn10_cartographer.launch.py
```

### 8.3 保存地图

```bash
ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
  -pbstream_filename ~/.ros/cartographer.pbstream \
  -map_filestem ./my_map \
  -resolution 0.05
```

### 8.4 修改清单

| 文件 | 改动 |
|------|------|
| `src/anorosdt2/` | **新建** Python ROS2 包（串口读飞控 → ANO PT v7 解析 → `/imu/data`） |
| `src/lslidar_driver/config/lsn10.lua` | `tracking_frame` → `imu_link`，`use_imu_data` → `true` |
| `src/lslidar_driver/launch/lsn10_cartographer.launch.py` | URDF 加 `imu_link`，Cartographer 加 IMU remap，启动 `anorosdt2` 节点 |
| `src/lslidar_driver/src/lslidar_driver.cc` | 注释掉 N10 路径的 `time_increment`（修复时间戳 bug） |
| 系统 | `sudo apt purge brltty`（修复 CH340 串口被抢占黑屏） |

---

## 九、反思

### 9.1 做得对的事情

- **先用 Python 裸测串口数据**，确认协议解析正确后再写 ROS 节点。如果直接写节点去调试，出了问题不好判断是 ROS 问题还是解析问题。
- **从 `journalctl` 和 `dmesg` 里找黑屏原因**，而不是猜测。brltty 这个坑如果不去翻日志，根本想不到是盲文服务在搞鬼。
- **对比 ROS1 的参考代码**，逐行对应转换公式和帧结构，避免了猜测。

### 9.2 可以改进的

- `time_increment` 那个 bug 其实之前 `ros2 topic echo /scan` 如果仔细看了输出就能发现异常，但我没做这个检查，直到 Cartographer 报错才去追。
- `anorosdt2` 的 `sc1` 作用域 bug 是写代码时的低级错误，单元测试一下就能发现。但串口设备不在手边的时候没法测，可以 mock 一段字节流来测状态机。

### 9.3 后续

接下来要整合小车底盘里程计、配 Nav2 做导航。底盘是一个 STM32F103，通过另一个 USB 串口和电脑通信，有自己的私有协议（`0xBB` 速度指令、`0xCC` 里程计回传）。基本思路是写一个 `wheeltec_chassis` 桥接节点，格式和 `anorosdt2` 一样：读串口 → 解析二进制帧 → 发 `/odom`，同时把 Nav2 的 `/cmd_vel` 转成 `0xBB` 帧发下去。这是另一个故事了。
