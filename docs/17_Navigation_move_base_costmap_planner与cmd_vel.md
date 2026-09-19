<meta name="referrer" content="no-referrer" />

# ROS教程17：Navigation / move_base——从定位、Costmap、Planner 到 cmd_vel

> 摘要：把 AMCL、TF、里程计和激光观测接入 move_base，理解 global/local costmap、Navfn、DWA 与 cmd_vel 的真实数据依赖，并完成阶段 C 的 AGV 导航闭环。

![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/6c9d2da7ce4c411bb8edb4c83cf22ed4.png)

@[toc]
第 13～16 章已经分别建立了移动机器人导航上游最关键的几条数据链：

```text
第 13 章：TF / tf2
map -> odom -> base_link -> sensor

第 14 章：URDF + robot_state_publisher
base_link -> laser_link / imu_link / wheel_link

第 15 章：robot_localization
/odom/raw + /imu/data_raw
        -> /odometry/filtered
        -> odom -> base_link

第 16 章：AMCL
/map + /scan + odom + TF
        -> map -> odom
```

到这里，机器人已经能够回答：

```text
我在地图哪里？
我现在怎么运动？
周围哪里有障碍物？
```

但还没有回答最后一个问题：

```text
给定一个目标点，我应该怎样安全地走过去？
```

第 17 章把这些上游结果接入 ROS1 Navigation Stack。核心链路是：

```text
localization
    -> costmap
    -> global planner
    -> local planner
    -> /cmd_vel
```

先从完整数据依赖看这一章真正连接了哪些上游和下游模块：

![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/c55718ad3d354c9d8dac9df6d5d5c34e.png)


图中最重要的不是模块数量，而是两条不同职责的链：global costmap 在 `map` 中承接全局定位结果，local costmap 在 `odom` 中承接连续局部运动；两者最终都影响 DWA 输出的 `/cmd_vel`，底盘反馈又重新进入状态估计。

本章不会回到“怎样写激光雷达 Driver”，也不会推导 Dijkstra、DWA 的完整数学证明。重点是理解每一个组件**是什么、解决什么问题、基本原理、输入输出、怎样配置、什么场景使用，以及上游数据错误怎样沿导航链传播**。

配套 package：

```text
/workspace/ros_ws/src/ros1_navigation_lab
```

本章完成后，阶段 C 的 AGV 数据链路与 Navigation 到此结束。

---

## 1. move_base 是什么：它不是一种规划算法

第一次接触 ROS1 Navigation 时，很容易把 `move_base` 理解成“路径规划算法”。实际上它更接近一个**导航编排器**。

`move_base` 自己负责把以下组件组织在一起：

```text
导航目标
  |
  v
全局 Costmap ----> Global Planner
                     |
                     v
                  全局路径
                     |
                     v
局部 Costmap ----> Local Planner
                     |
                     v
                  /cmd_vel
```

它通过插件接口加载具体 planner，例如本章使用：

```text
Global Planner：navfn/NavfnROS
Local Planner ：dwa_local_planner/DWAPlannerROS
```

因此必须区分：

| 名称 | 本章中的职责 |
| --- | --- |
| `move_base` | 管理导航目标、costmap、planner 和导航状态 |
| `NavfnROS` | 根据全局 costmap 求一条从当前位置到目标点的路径 |
| `DWAPlannerROS` | 根据局部环境和机器人当前运动状态计算下一条速度命令 |
| `/cmd_vel` | local planner 输出给底盘的期望速度，不是电机 PWM，也不是轮速反馈 |

官方 API 中，`MoveBase` 同时依赖 `nav_core::BaseGlobalPlanner`、`nav_core::BaseLocalPlanner` 和两套 `costmap_2d::Costmap2DROS`，正好说明 `move_base` 的角色是装配和调度这些模块，而不是把所有算法写死在一个类中。

参考：

- <https://docs.ros.org/en/api/move_base/html/move__base_8h.html>

### 什么时候使用 move_base

`move_base` 适合经典 ROS1 二维移动底盘导航：

```text
二维或近似二维环境
+ 已有地图或可构造二维 costmap
+ 能提供机器人位姿
+ 能提供障碍物观测
+ 底盘接受 geometry_msgs/Twist
```

对于本系列的差速 AGV，这正是需要的接口形态。

---

## 2. 为什么 planner 不能直接拿 `/map` 和 `/scan` 算 `/cmd_vel`

如果只有静态 `/map`，系统只知道建图时环境是什么样：

```text
/map
  -> 墙
  -> 固定设备
  -> 建图时已经存在的障碍
```

如果只看 `/scan`，系统又只能知道机器人此刻附近看到了什么：

```text
/scan
  -> 当前附近墙体
  -> 临时箱子
  -> 新出现的托盘
  -> 人或其它移动物体的瞬时观测
```

同时，planner 还必须考虑机器人自身尺寸。如果只把机器人当成一个点，即使路径中心线没有撞墙，机器人两侧也可能已经发生碰撞。

因此 Navigation Stack 先把这些信息转换为统一的数据结构：

```text
Costmap
```

Costmap 是 planner 和真实世界之间的“风险表达层”。

---

## 3. Costmap 是什么：不是另一张 SLAM 地图

`costmap_2d` 可以理解为二维栅格代价地图。每个栅格不再只表达“占据 / 空闲”，而是表达：

```text
机器人经过这个位置的代价有多高？
```

典型语义包括：

```text
自由空间
    -> 可以经过

障碍物
    -> 不能经过

靠近障碍物的区域
    -> 可以计算，但代价逐渐升高

未知区域
    -> 是否允许进入由配置决定
```

这和第 16 章的 `/map` 有明显区别：

| `/map` | costmap |
| --- | --- |
| `nav_msgs/OccupancyGrid` 静态环境表达 | Navigation 内部使用的可更新代价格网 |
| 主要来自 SLAM / map_server | 可以由多层数据共同合成 |
| 不直接表达机器人安全余量 | 可以通过 footprint + InflationLayer 表达碰撞和安全距离 |
| 临时障碍通常不在保存地图中 | 可以实时接收激光或点云更新 |

`Costmap2DROS` 内部维护一个 master costmap，各个 layer 把自己的结果更新进去。官方接口也明确把它描述为 2D costmap 的 ROS wrapper，并通过 TF 把障碍物观测转换到 costmap 的参考坐标系。

参考：

- <https://docs.ros.org/en/latest-available/api/costmap_2d/html/classcostmap__2d_1_1Costmap2DROS.html>

---

## 4. Layered Costmap：StaticLayer、ObstacleLayer、InflationLayer 各干什么

本章 global costmap 使用三层：

```yaml
plugins:
  - {name: static_layer, type: "costmap_2d::StaticLayer"}
  - {name: obstacle_layer, type: "costmap_2d::ObstacleLayer"}
  - {name: inflation_layer, type: "costmap_2d::InflationLayer"}
```

local costmap 使用：

```yaml
plugins:
  - {name: obstacle_layer, type: "costmap_2d::ObstacleLayer"}
  - {name: inflation_layer, type: "costmap_2d::InflationLayer"}
```

这三个 layer 的含义不能只记名字。

### 4.1 StaticLayer：把已有地图变成导航基础环境

`StaticLayer` 读取地图中的静态占据信息。

本章配置：

```yaml
static_layer:
  enabled: true
  map_topic: /map
  subscribe_to_updates: false
  track_unknown_space: true
```

数据关系是：

```text
map_server
   |
   v
 /map
   |
   v
StaticLayer
   |
   v
global costmap
```

它解决的问题是：

> 全局规划不能每次只靠传感器重新观察整栋建筑，应该先利用已有地图中的固定墙体和结构。

适合：

```text
墙体
固定货架
房间边界
建图阶段已经确认的静态结构
```

不适合单独承担：

```text
刚放到通道中的托盘
临时纸箱
移动人员
建图以后新增的设备
```

这些变化必须由实时传感器层补充。

### 4.2 ObstacleLayer：把实时传感器观测变成障碍物

本章真正把第 16 章 `/scan` 接进导航的是 `ObstacleLayer`：

```yaml
obstacle_layer:
  observation_sources: laser_scan_sensor

  laser_scan_sensor:
    sensor_frame: laser_link
    data_type: LaserScan
    topic: /scan
    marking: true
    clearing: true
```

`marking` 的含义可以先理解为：

```text
传感器看到这里有障碍
    -> 把对应 costmap cell 标记为障碍
```

`clearing` 则是另一半：

```text
从传感器原点沿测量射线向外 raytrace
    -> 射线穿过的可见空间可以被清除
```

为什么需要 clearing？

假设一个纸箱之前位于机器人前方：

```text
机器人 ----> [纸箱]
```

ObstacleLayer 把它标成障碍。

纸箱被移走后，如果系统只会 `marking` 而不会 `clearing`，旧障碍就可能一直留在 costmap 中。射线清除机制让“现在已经能看穿的空间”重新变成可通行区域。

本章激光仿真在没有命中障碍时使用 `Inf`，所以配置：

```yaml
inf_is_valid: true
```

允许 ObstacleLayer 把这类读数按有效的“看到最大距离仍没有障碍”处理，用于 clearing。

ROS1 Noetic 的 `ObstacleLayer` 源码中同时提供 `laserScanCallback()`、`laserScanValidInfCallback()` 和 `pointCloud2Callback()`，说明 `LaserScan` 与 `PointCloud2` 都可以成为这一层的观测源。

参考：

- <https://docs.ros.org/en/noetic/api/costmap_2d/html/classcostmap__2d_1_1ObstacleLayer.html>

### 4.3 obstacle_range 与 raytrace_range 分别控制什么

本章配置：

```yaml
obstacle_range: 4.5
raytrace_range: 6.0
```

可以理解成：

```text
obstacle_range
    -> 多远以内的命中点允许用于“标记障碍”

raytrace_range
    -> 多远以内的射线允许用于“清除自由空间”
```

二者不是激光雷达自身 `range_max` 的替代品。

`LaserScan.range_max` 是传感器消息的数据契约；`obstacle_range` / `raytrace_range` 是 costmap 决定如何使用这份数据的策略参数。

### 4.4 InflationLayer：为什么障碍物周围还有一圈代价

如果 costmap 只把墙本身标成不可通行：

```text
自由 自由 自由 障碍 自由 自由
```

planner 很可能给出一条几乎贴着墙走的路径。

真实机器人有宽度，而且定位、控制和传感器都存在误差，所以通常需要让障碍附近的格子逐渐变“贵”：

```text
低代价 -> 中代价 -> 高代价 -> 障碍 -> 高代价 -> 中代价
```

这就是 `InflationLayer` 的作用。

本章：

```yaml
inflation_radius: 0.55
cost_scaling_factor: 3.0
```

`inflation_radius` 决定障碍影响向外延伸多远；`cost_scaling_factor` 影响代价随距离衰减的速度。

重要的是：

> InflationLayer 不是简单把墙“物理加粗 0.55 m”，而是在障碍周围生成一圈距离相关的代价梯度，planner 再根据这些代价选择更安全的路线。

适合用于：

```text
希望机器人与墙、货架、设备保持一定余量
避免全局路径长期贴边
让局部轨迹评分对障碍距离敏感
```

如果 inflation 太大，窄通道会看起来完全不可走；如果太小，规划结果可能离障碍过近。因此它必须结合真实 footprint、通道宽度和定位误差调整。

---

## 5. footprint 是什么：planner 不能把 AGV 当作一个点

本章机器人主体约 0.60 m 长，轮组外廓宽度略大于 0.44 m 车体，因此配置一个覆盖整车外廓的矩形：

```yaml
footprint:
  - [ 0.30,  0.28]
  - [ 0.30, -0.28]
  - [-0.30, -0.28]
  - [-0.30,  0.28]

footprint_padding: 0.02
```

这些点都位于 `base_link` 坐标系。

作用是告诉 costmap / local planner：

```text
机器人不是一个数学点
而是这个多边形区域都会占用空间
```

实际 AGV 常见来源包括：

```text
URDF / CAD 外形
底盘保险杠边界
伸出的轮子、叉臂或传感器支架
需要额外保留的碰撞安全余量
```

footprint 配得过小，会出现“路径看起来没碰撞，但真实车体会刮到障碍物”；配得过大，则可能把本来能通过的走廊判定成不可通行。

---

## 6. 为什么有 global costmap 和 local costmap 两套

这是 ROS1 Navigation 最关键的设计之一。

本章配置：

```text
global costmap：global_frame = map
local costmap ：global_frame = odom
```

### 6.1 Global Costmap：回答“整张地图上应该往哪走”

全局 costmap 使用：

```yaml
global_frame: map
rolling_window: false
```

因为全局 planner 需要在整张地图上规划路径，所以它需要稳定的全局参考：

```text
map
```

机器人在 global costmap 中的位置来自 TF：

```text
map
  -> odom          AMCL
  -> base_link     robot_localization
```

因此：

```text
AMCL 定位错误
    -> map -> odom 错误
    -> global costmap 认为机器人起点错误
    -> global planner 的起点随之错误
```

### 6.2 Local Costmap：回答“眼前这几米应该怎么走”

本章 local costmap：

```yaml
global_frame: odom
rolling_window: true
width: 4.0
height: 4.0
```

这意味着它不是固定覆盖整张地图，而是保持一个随机器人移动的局部窗口。

为什么使用 `odom`，而不是 `map`？

第 16 章已经建立：

```text
map -> odom
允许因为全局定位修正而变化

odom -> base_link
应该保持局部连续
```

局部控制器每秒会计算很多次速度命令。如果局部运动参考本身不断因为全局定位修正发生跳动，速度控制就会非常难稳定。

所以经典分层是：

```text
全局规划关注 map 中的全局正确性
局部控制关注 odom 中的运动连续性
```

这就是第 16 章 `map -> odom -> base_link` 分层设计在 Navigation 中真正发挥作用的位置。

---

## 7. Global Planner 是什么：先决定“走哪条路”

本章使用：

```yaml
base_global_planner: navfn/NavfnROS
```

它的输入可以概括为：

```text
机器人当前全局位姿
+ 导航目标位姿
+ global costmap
```

输出是：

```text
从起点到终点的一串 PoseStamped
```

也就是一条全局路径。

在本章 RViz 中观察：

```text
/move_base/NavfnROS/plan
```

### 7.1 Navfn 的原理需要理解到什么程度

`NavfnROS` 是 `nav_core::BaseGlobalPlanner` 插件的一个实现。在 ROS1 Noetic 的实际 `NavfnROS::makePlan()` 路径中，它调用 `calcNavFnDijkstra(true)` 在栅格 costmap 上计算 navigation function；底层 `NavFn` 类同时保留 `calcNavFnAstar()` API，但本章配置的 `navfn/NavfnROS` 主路径使用 Dijkstra。

对工程使用而言，本章保留下面这个心智模型即可：

```text
costmap 中每个格子都有代价
    -> 从目标/起点传播路径代价
    -> 避开不可通行区域和高代价区域
    -> 回溯得到一条全局路径
```

所以 global planner 关注的是：

```text
整个环境中的拓扑和整体路线
```

而不是直接决定此刻左右轮应该转多快。

参考：

- <https://docs.ros.org/en/noetic/api/navfn/html/classnavfn_1_1NavFn.html>

### 7.2 什么时候 global planner 会重新规划

本章设置：

```yaml
planner_frequency: 1.0
```

意味着存在有效导航目标时，move_base 可以周期性重新进行全局规划。

这对实时障碍物进入 global costmap 很重要。例如：

```text
/map 中原本没有托盘
       |
       v
/scan 看见临时托盘
       |
       v
global ObstacleLayer 标记障碍
       |
       v
下一次 global planning 使用更新后的 costmap
```

如果障碍只影响很局部、且局部 planner 可以绕开，全局路径未必每次都需要大幅变化；如果障碍切断原全局路线，就可能需要重新寻找另一条路径。

---

## 8. Local Planner 是什么：决定“下一小段怎么运动”

本章使用：

```yaml
base_local_planner: dwa_local_planner/DWAPlannerROS
```

Local planner 的输入比 global planner 更接近真实控制：

```text
当前机器人位姿
+ 当前速度
+ global plan
+ local costmap
+ 速度/加速度约束
```

它输出：

```text
geometry_msgs/Twist
```

最终由 `move_base` 发布到：

```text
/cmd_vel
```

### 8.1 为什么 DWA 需要当前速度

DWA 的全称是 Dynamic Window Approach。

它并不是在任意速度空间里随便选择一条命令，而是首先根据：

```text
当前速度
+ 加速度限制
+ 控制周期
```

得到短时间内机器人实际上能够达到的一组候选速度。

然后对这些候选速度向前模拟轨迹。

可以粗略理解成：

```text
候选 (vx1, wz1) -> 模拟 1.5 s -> 一条轨迹
候选 (vx2, wz2) -> 模拟 1.5 s -> 一条轨迹
候选 (vx3, wz3) -> 模拟 1.5 s -> 一条轨迹
...
```

再根据：

```text
是否碰撞
离 global plan 多远
离目标多远
离障碍物多近
是否存在振荡等问题
```

对轨迹评分，最终选择一个可执行的速度命令。

官方接口 `computeVelocityCommands()` 的描述就是根据当前 position、orientation 和 velocity 计算发给底盘的速度命令。

参考：

- <https://docs.ros.org/en/noetic/api/dwa_local_planner/html/classdwa__local__planner_1_1DWAPlannerROS.html>

### 8.2 为什么这里必须把 odom_topic 指到 `/odometry/filtered`

第 15 章的最终局部状态输出是：

```text
/odometry/filtered
```

所以本章明确配置：

```yaml
DWAPlannerROS:
  odom_topic: /odometry/filtered
```

这样 DWA 获取的当前速度与本系列最终采用的 `robot_localization` 状态估计保持一致。

如果仍然使用一个与阶段 C 数据链无关的默认 `/odom`，就会出现：

```text
TF 使用一套状态
local planner 的当前速度却使用另一套状态
```

这会让教程的数据 ownership 重新混乱。

### 8.3 差速底盘为什么把 y 速度锁为 0

本章配置：

```yaml
max_vel_y: 0.0
min_vel_y: 0.0
vy_samples: 1
```

因为普通差速底盘不能像全向轮底盘一样直接横向平移。

候选控制主要是：

```text
linear.x
angular.z
```

这也和第 12 章底盘 Driver 的 `geometry_msgs/Twist` 契约保持一致。

---

## 9. `/cmd_vel` 到底是什么，以及为什么导航闭环还没有在这里结束

`/cmd_vel` 表示机器人期望的车体速度命令，例如：

```text
linear.x  = 0.25 m/s
angular.z = 0.30 rad/s
```

它不是：

```text
左轮 PWM
右轮 PWM
CAN 报文
电机转速反馈
```

真正的底盘链路仍然是第 12 章建立的：

```text
/cmd_vel
   |
   v
chassis_driver_node
   |
   +-> 差速逆运动学
   +-> 左右轮目标速度
   +-> 实际硬件协议 / 本教学仿真
```

同时 Driver 继续反馈：

```text
/odom/raw
/imu/data_raw
```

再进入第 15 章：

```text
robot_localization
    -> /odometry/filtered
    -> odom -> base_link
```

因此 Navigation 是一个闭环，而不是单向流水线：

```text
planner
  -> /cmd_vel
  -> chassis
  -> odom / IMU
  -> localization
  -> costmap / planner
  -> 下一条 /cmd_vel
```

这也是实际 AGV 排障时不能只盯着 planner 的原因。

---

## 10. LaserScan、PointCloud2、Range 在 costmap 中分别怎样使用

第 16 章已经讲过 `LaserScan` 和 3D 多线雷达的数据模型，本章只讨论 Navigation 真正怎样消费它们。

### 10.1 LaserScan

本实验使用：

```text
sensor_msgs/LaserScan
Topic: /scan
frame: laser_link
```

ObstacleLayer 会利用 TF 把扫描数据变换到 costmap 的 `global_frame`。

所以一帧 `/scan` 能被 costmap 正确使用至少依赖：

```text
正确 header.frame_id
正确 header.stamp
对应时间点的 TF 可查询
有效 ranges[]
```

对于 global costmap，需要能找到：

```text
map -> ... -> laser_link
```

对于 local costmap，需要能找到：

```text
odom -> ... -> laser_link
```

### 10.2 PointCloud2

ObstacleLayer 也支持：

```text
sensor_msgs/PointCloud2
```

因此真实 AGV 如果把 3D LiDAR 经过地面分割、高度过滤或其它预处理后生成障碍点云，可以把相应 PointCloud2 作为 observation source。

典型使用场景：

```text
3D LiDAR
  -> 点云过滤
  -> 保留机器人关心高度范围内的障碍点
  -> PointCloud2
  -> ObstacleLayer / VoxelLayer
```

本章不展开 PCL 和点云字段，那仍属于后面的感知阶段。

### 10.3 Range

`sensor_msgs/Range` 常用于超声波、红外等单束或锥形测距传感器。

它并不是本章 `costmap_2d::ObstacleLayer` 的 `LaserScan` / `PointCloud2` 输入类型。ROS1 生态中通常通过独立的 `range_sensor_layer` costmap plugin 把 `Range` 数据投影到代价地图。

适合的场景包括：

```text
近距离盲区补偿
超声波防撞
低成本 IR / sonar 辅助
```

本实验不安装这一插件，因为当前成功路径已经由 360° `LaserScan` 闭环；把 Range plugin 加进主流程反而会混淆“标准 ObstacleLayer”和“额外导航层”的边界。

参考：

- <https://index.ros.org/p/range_sensor_layer/>

---

## 11. 实现 ros1_navigation_lab

本章新增：

```text
ros_ws/src/ros1_navigation_lab/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   ├── costmap_common.yaml
│   ├── dwa_local_planner.yaml
│   ├── global_costmap.yaml
│   ├── local_costmap.yaml
│   ├── move_base.yaml
│   ├── navigation_world.yaml
│   └── navfn.yaml
├── launch/
│   └── navigation.launch
└── rviz/
    └── navigation.rviz
```

本章没有新增自定义 C++/Python 导航算法。原因是学习目标不是重写 Navigation Stack，而是让第 13～16 章的输出真正接入 ROS1 已有导航组件，并能够沿 Topic / TF / costmap / planner 追踪数据。

---

## 12. 导航实验为什么故意让 `/map` 和真实环境有一点不同

第 16 章保存的 `lab_map` 仍然作为已有地图：

```text
/map
```

第 17 章的 `navigation_world.yaml` 在相同墙体基础上额外加入：

```yaml
- [-0.75, -1.0, 0.75, -1.0]
```

它表示一段建图以后才放进环境中的临时障碍。

因此：

```text
StaticLayer
    -> 看不到这段临时障碍

ObstacleLayer
    -> /scan 能看到
```

如果从起点：

```text
(0, 0)
```

导航到：

```text
(0, -2.2)
```

静态地图中的直线路径并不知道中间多了一段障碍，但是实时 costmap 会把它标出来。

这正好回答一个实际 AGV 问题：

> 已经有地图了，为什么导航还必须持续读取激光雷达？

因为地图描述的是环境的持久结构，实时传感器负责补上地图建立以后发生的变化。

---

## 13. 启动前先补齐 Navigation 依赖

Dockerfile 新增：

```text
ros-noetic-move-base
ros-noetic-navfn
ros-noetic-dwa-local-planner
```

Dockerfile 改动后，需要在 Host 重建镜像：

```bash
cd /home/wdfk/share/ros1-docker
docker compose up -d --build
```

进入 Container：

```bash
docker compose exec ros1-dev bash
```

本系列没有在聊天环境中执行 Docker rebuild，因此这里是需要在实际开发机执行的步骤，不应把它写成已经验证通过。

---

## 14. 构建第 17 章 package

在 Container 中：

```bash
cd /workspace/ros_ws
source /opt/ros/noetic/setup.bash
catkin build ros1_navigation_lab
source /workspace/ros_ws/devel/setup.bash
```

如果前面 workspace 已使用 `--merge-devel`，继续沿用原有 profile，不重新创建另一套 workspace。

确认 ROS 能找到 package：

```bash
rospack find ros1_navigation_lab
```

预期路径：

```text
/workspace/ros_ws/src/ros1_navigation_lab
```

---

## 15. 一次启动完整 Navigation 链

执行：

```bash
roslaunch ros1_navigation_lab navigation.launch
```

这个 launch 一次完成：

```text
ros1_driver_lab
    -> /odom/raw + /imu/data_raw

robot_localization
    -> /odometry/filtered
    -> odom -> base_link

robot_state_publisher
    -> base_link -> laser_link

simple_laser_world
    -> /scan

map_server
    -> /map

AMCL
    -> map -> odom

move_base
    -> global/local costmap
    -> Navfn
    -> DWA
    -> /cmd_vel
```

注意 TF ownership 仍然保持：

| TF | Owner |
| --- | --- |
| `map -> odom` | AMCL |
| `odom -> base_link` | `robot_localization` |
| `base_link -> laser_link` | `robot_state_publisher` |

`move_base` 不应该抢着发布这些定位 TF；它是这些 TF 的消费者。

---

## 16. 第一轮观察：先确认输入齐全，再发导航目标

查看关键 Topic：

```bash
rostopic list | grep -E 'map|scan|odom|cmd_vel|move_base|costmap|plan|particle'
```

至少应能找到本章关键链路中的 Topic，例如：

```text
/map
/scan
/odometry/filtered
/cmd_vel
/move_base/global_costmap/costmap
/move_base/local_costmap/costmap
/move_base/NavfnROS/plan
```

确认 TF：

```bash
rosrun tf tf_echo map base_link
```

再确认局部连续链：

```bash
rosrun tf tf_echo odom base_link
```

只有在定位与传感器链都成立以后，Navigation 的行为才有解释基础。

---

## 17. 在 RViz 里分别看四类信息

`navigation.rviz` 默认打开：

```text
Static Map
Global Costmap
Local Costmap
LaserScan
RobotModel
TF
Global Plan
Local Plan
AMCL Particle Cloud
```

观察时不要把这些图层混成“都是地图”。

### 17.1 Static Map

来源：

```text
/map
```

回答：

```text
已有地图认为环境长什么样？
```

### 17.2 Global Costmap

来源：

```text
StaticLayer + ObstacleLayer + InflationLayer
```

回答：

```text
从整张地图角度，目前哪里可以走、哪里代价高？
```

这里应该能够看到临时障碍对 costmap 的影响，即使它不在静态 `/map` 中。

### 17.3 Local Costmap

它是 `odom` 下跟随机器人移动的滚动窗口。

回答：

```text
机器人当前附近几米范围内，local planner 认为哪里安全？
```

### 17.4 Global Plan 与 Local Plan

```text
Global Plan
    -> 整体路线

Local Plan
    -> 当前控制周期附近实际选择的短期轨迹
```

不要把 local plan 理解成第二条独立全局路径。

---

## 18. 发一个真正的导航目标

最直观的方法是在 RViz 中使用：

```text
2D Nav Goal
```

可以把目标点选在起点下方，例如：

```text
x = 0.0
y = -2.2
```

也可以直接发布：

```bash
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: -2.2, z: 0.0}, orientation: {z: -0.7071068, w: 0.7071068}}}"
```

发目标以后同时观察：

```bash
rostopic echo /cmd_vel
```

以及：

```bash
rostopic echo /odometry/filtered
```

这时整个闭环第一次真正跑起来：

```text
目标
 -> global plan
 -> DWA 选择局部轨迹
 -> /cmd_vel
 -> Driver
 -> odom
 -> EKF
 -> TF / 当前速度
 -> 下一次 local planning
```

---

## 19. 为什么一个上游错误会表现成“导航算法不正常”

Navigation 的很多异常，根因并不在 planner。

这一点是阶段 C 最重要的工程结论之一。

### 19.1 `map -> odom` 错：全局位置整体错位

如果 AMCL 定位错误：

```text
AMCL 错
 -> map -> odom 错
 -> map 中 base_link 位姿错
 -> global costmap 机器人位置错
 -> global planner 起点错
 -> 路径可能从错误位置开始
```

表面现象可能是：

```text
机器人在 RViz 地图里“飘到墙里”
全局路径起点不在机器人上
规划失败或路线明显不合理
```

这时继续调 `path_distance_bias` 没有意义，应该先回到定位链。

### 19.2 `odom -> base_link` 抖动：局部控制直接受影响

local costmap 和 DWA 依赖局部连续运动。

如果 EKF / odom 异常：

```text
odom -> base_link 抖动
 -> local costmap 中机器人位姿抖动
 -> 当前速度估计不稳定
 -> DWA 候选速度窗口变化
 -> /cmd_vel 可能抖动、频繁转向或无法找到有效轨迹
```

所以“AMCL 很准”不能替代好的 odometry。

### 19.3 `laser_link` TF 错：障碍物被投影到错误位置

例如真实雷达位于机器人前方 0.25 m，但 TF 配置成 0.50 m：

```text
/scan 本身数值正常
但 sensor -> base_link 几何关系错
```

结果：

```text
ObstacleLayer 把障碍标到错误位置
 -> costmap 出现整体偏移
 -> planner 可能错误绕障或靠障碍过近
```

这类问题不能只通过 `rostopic echo /scan` 发现，因为消息本身可以完全合法。

### 19.4 timestamp / TF 不匹配：有 Topic 不等于 costmap 真正使用了数据

ObstacleLayer 使用 message filter 等机制等待对应时间的 TF。

如果：

```text
/scan 在持续发布
但 header.stamp 对应的 TF 查不到
```

就可能出现：

```text
Topic 明明有数据
costmap 却没有正确更新障碍物
```

因此传感器调试必须一起检查：

```text
Topic
frame_id
stamp
TF availability
```

### 19.5 footprint 错：软件认为能过，真实车却过不去

如果 footprint 小于真实车体：

```text
costmap / planner：可以通过
真实 AGV：轮子、车壳或货叉已经碰撞
```

如果 footprint 过大：

```text
真实 AGV：明明能通过
planner：认为走廊被完全堵死
```

所以 footprint 是机械尺寸与 Navigation 之间非常直接的接口契约。

### 19.6 `/cmd_vel` 正常但车走错：继续向下追 Driver

如果：

```text
/cmd_vel 看起来正确
```

但真实车方向或速度不对，应继续沿第 12 章的数据链检查：

```text
Twist
 -> 差速逆运动学
 -> 左右轮符号
 -> 单位
 -> gear ratio
 -> 硬件命令
```

不能因为命令来自 `move_base` 就把底盘错误归类成“导航算法问题”。

---

## 20. 一张表把阶段 C 的排障入口固定下来

| 现象 | 优先观察 | 首先怀疑的层 |
| --- | --- | --- |
| `/scan` 有数据但 costmap 没障碍物 | `frame_id`、stamp、TF、ObstacleLayer source | Sensor / TF / costmap 配置 |
| global plan 起点不在机器人位置 | `map -> base_link` | AMCL / `map -> odom` |
| 全局路径合理但机器人局部抖动 | `odom -> base_link`、`/odometry/filtered`、local costmap | Odom / EKF / local planner 输入 |
| 机器人总贴墙 | footprint、inflation、局部障碍物位置 | Robot geometry / costmap |
| 临时障碍完全不起作用 | `/scan` 与 obstacle layer | Sensor -> costmap |
| `/cmd_vel` 一直为空 | move_base goal、global plan、local planner 是否找到合法轨迹 | Planner / costmap / TF |
| `/cmd_vel` 正常但底盘方向错误 | Driver 输入输出、左右轮符号、单位 | Driver / hardware |
| 地图里机器人整体错位 | `/amcl_pose`、`map -> odom` | Localization |

这张表的用途不是背故障答案，而是确定排查顺序：

```text
先验证上游数据契约
再看 costmap
再看 planner
最后看 /cmd_vel
```

不要从最终运动现象直接跳到 planner 参数。

---

## 21. global/local costmap 与 planner 的使用场景怎样选择

到这里可以把几个核心组件的“什么时候使用”压缩成下面这张表。

| 组件 | 主要问题 | 典型场景 |
| --- | --- | --- |
| Global Costmap | 整个地图哪里可通行 | 跨房间、跨走廊、长距离导航 |
| Local Costmap | 当前附近哪里安全 | 实时避障、短时间轨迹控制 |
| StaticLayer | 已有地图中的固定结构 | 墙体、固定货架、房间结构 |
| ObstacleLayer | 当前传感器观测到的障碍 | 临时箱子、人员、建图后新增物体 |
| InflationLayer | 与障碍保持合理余量 | 避免贴墙、给定位和控制误差留空间 |
| Navfn | 生成地图级全局路径 | 二维栅格地图上的经典全局规划 |
| DWA | 计算短时间可执行速度 | 差速/移动底盘实时局部控制和避障 |
| `/cmd_vel` | 把导航结果交给底盘 | 上层 Navigation 与底盘 Driver 的控制接口 |

任何一个模块都不是孤立的。比如 DWA 本身并不知道 `LaserScan` 消息格式，它看到的是已经由 local costmap 转换出的障碍代价；Navfn 也不直接订阅 AMCL pose，而是通过 costmap / TF 获得规划所需的机器人全局位置。

这就是“数据依赖”比背 Topic 名更重要的原因。

---

## 22. 阶段 C 到这里形成了什么完整能力

阶段 C 从第 13 章开始时，只有一个看起来很简单的问题：

```text
移动机器人怎样知道自己在哪里并走到目标点？
```

现在已经可以沿真实 ROS1 数据链回答：

```text
URDF
 -> 定义机器人 link/joint 与传感器安装几何

Driver
 -> /odom/raw
 -> /imu/data_raw

robot_localization
 -> /odometry/filtered
 -> odom -> base_link

LiDAR
 -> /scan

AMCL
 -> 使用 /map + /scan + odom
 -> map -> odom

Costmap
 -> 把静态地图、实时障碍和机器人 footprint 合成导航代价

Global Planner
 -> 在 global costmap 上生成全局路径

Local Planner
 -> 在 local costmap 上结合当前速度生成可执行局部轨迹

move_base
 -> 发布 /cmd_vel

Driver
 -> 把 Twist 转成底盘执行量
```

因此阶段 C 最终形成的是一个闭环：

```text
Sensor / Driver
      -> TF + State Estimation
      -> Localization
      -> Costmap
      -> Global / Local Planner
      -> /cmd_vel
      -> Driver / Chassis
      -> 新的运动与传感器数据
```

第 18 章不再单独拆“AGV 跨层故障定位”。原计划中的故障矩阵已经并入本章，因为只有看到 Navigation 真正消费这些数据以后，`Topic -> TF -> localization -> costmap -> planner -> cmd_vel` 的跨层故障传播才具有完整上下文。

后续阶段开始进入点云、相机等感知接口时，不再改变这里建立的基本原则：上游传感器输出首先要满足时间、frame 和数据契约，然后才有资格被定位、costmap、规划或感知算法正确消费。
