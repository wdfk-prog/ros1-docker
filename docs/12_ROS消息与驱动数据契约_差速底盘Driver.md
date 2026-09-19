<meta name="referrer" content="no-referrer" />

# ROS教程12：ROS消息与驱动数据契约——用差速底盘理解 Twist、JointState、Imu 与 Odometry

> 摘要：以差速底盘为例，从 Twist 的 v/w、左右轮运动学和真实误差来源，讲到 Odometry 与 6×6 covariance 的含义、测量和工程使用。
![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/93ef40a8aec74316800083ea5437df90.png)

@[toc]
阶段 A 已经完成 Topic、Service、Action、CallbackQueue、Spinner、测试和整机启动链。本章开始进入驱动数据，但刻意不展开 TCP、CAN、串口、自定义设备协议，也不讨论如何从 Linux Socket 收字节。

本章只做两件事：

```text
方向 1：设备数据 -> ROS

已经取得的底盘数据
    -> JointState
    -> Imu
    -> Odometry
    -> DiagnosticArray

方向 2：ROS -> 底盘

/cmd_vel
    -> geometry_msgs/Twist
    -> 差速底盘逆运动学
    -> 左右轮目标角速度
    -> 假定已经交给底层设备
```

这样可以把注意力放在 ROS Driver 最重要的数据契约上：**物理量是什么、单位是什么、坐标系是什么、ROS 消费者期待什么。**

本章工程位于：

```text
/workspace/ros_ws/src/ros1_driver_lab
```

核心源码只有一个：

```text
ros_ws/src/ros1_driver_lab/src/chassis_driver_node.cpp
```

代码里已经加入中文注释，文章中的代码也保留必要注释，便于逐行阅读。

---

## 1. 先认识本章使用的 4 组 ROS 标准消息

Dockerfile 中新增了：

```text
ros-noetic-geometry-msgs
ros-noetic-sensor-msgs
ros-noetic-nav-msgs
ros-noetic-diagnostic-msgs
```

这一章只需要知道它们分别解决什么问题，不展开安装包、生成头文件和上游源码仓库之间的关系。

### 1.1 geometry_msgs：表达几何量和运动量

本章使用：

```cpp
#include <geometry_msgs/Twist.h>
```

`geometry_msgs/Twist` 表示一个物体的线速度和角速度：

```text
linear.x
linear.y
linear.z

angular.x
angular.y
angular.z
```

差速底盘只使用：

```text
linear.x   前进/后退线速度，单位 m/s
angular.z  绕 Z 轴旋转的角速度，单位 rad/s
```

官方定义：

- https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html

### 1.2 sensor_msgs：表达传感器和关节状态

本章使用：

```cpp
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
```

`JointState` 用来表达左右轮这种关节的状态；`Imu` 用来表达角速度、线加速度和可选姿态估计。

官方定义：

- https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/JointState.html
- https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html

### 1.3 nav_msgs：表达导航相关数据

本章使用：

```cpp
#include <nav_msgs/Odometry.h>
```

`nav_msgs/Odometry` 同时带：

```text
pose   位置 + 姿态

twist 线速度 + 角速度
```

它是移动底盘最常见的数据接口之一。

官方定义：

- https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html

### 1.4 diagnostic_msgs：表达设备健康状态

本章使用：

```cpp
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
```

这里只用它表达最基本的：

```text
设备数据可用 -> OK
设备离线     -> ERROR
```

watchdog、频率监控、超时和恢复属于驱动可靠性专题；当前学习主线不再单独拆成第 14 章，需要时再作为专项扩展。

官方定义：

- https://docs.ros.org/en/noetic/api/diagnostic_msgs/html/msg/DiagnosticArray.html

---

## 2. 本章为什么把“设备通信”直接省略

真实驱动里，数据可能来自：

```text
TCP
UDP
CAN
串口
USB
厂商 SDK
共享内存
```

但无论底层怎么拿数据，进入 ROS 层之后都应该先形成明确的物理量。

本章统一用：

```cpp
struct ChassisData
{
    double left_wheel_position_rad;
    double right_wheel_position_rad;
    double left_wheel_velocity_rad_s;
    double right_wheel_velocity_rad_s;

    double imu_angular_velocity_z_rad_s;
    double imu_linear_acceleration_x_m_s2;
    double imu_linear_acceleration_y_m_s2;
    double imu_linear_acceleration_z_m_s2;

    bool online;
};
```

这意味着从这一行开始：

```cpp
const ChassisData data = device_->readChassisData(dt_s);
```

底层通信已经结束。

后面只研究：

> `ChassisData` 中这些物理量应该怎样正确进入 ROS 消息。

为了让工程可以独立运行，源码内部有一个极薄的 `DemoChassisDevice`。它不使用 Socket，不创建第二个进程，也不模拟网络协议，只负责产生教学数据。

真实项目里替换这一层即可：

```text
DemoChassisDevice
       |
       | 真实项目替换
       v
真实底盘 SDK / CAN Driver / Ethernet Driver
```

ROS 消息映射部分可以继续保持。

---

## 3. `/cmd_vel` 中的 Twist 到底是什么意思

先查看：

```bash
rosmsg show geometry_msgs/Twist
```

可以看到：

```text
geometry_msgs/Vector3 linear
  float64 x
  float64 y
  float64 z
geometry_msgs/Vector3 angular
  float64 x
  float64 y
  float64 z
```

差速底盘通常约定：

```text
linear.x = v
angular.z = w
```

其中：

```text
v：机器人中心沿自身 X 轴方向的线速度，单位 m/s
w：机器人绕自身 Z 轴的角速度，单位 rad/s
```

例如：

```yaml
linear:
  x: 0.3
angular:
  z: 0.4
```

表示：

```text
机器人向前 0.3 m/s
同时绕 Z 轴逆时针旋转 0.4 rad/s
```

对于普通二维差速底盘，下面这些分量没有对应执行能力：

```text
linear.y
linear.z
angular.x
angular.y
```

所以代码只消费：

```cpp
msg->linear.x
msg->angular.z
```

### 为什么 `/cmd_vel` 使用 Topic，而不是 Action

因为 `/cmd_vel` 表达的是连续变化的瞬时速度指令。

例如控制器可能持续输出：

```text
0.30 m/s
0.30 m/s
0.25 m/s
0.10 m/s
0.00 m/s
```

它不是一个“提交一次然后等完成”的长任务。

所以分层通常是：

```text
高层目标：到达某个位置
        -> Action

局部控制器不断计算速度
        -> /cmd_vel Topic

底盘 Driver
        -> 左右轮目标速度
```

因此第 11 章学过的 Action 并不是没用了，而是处在更高层。

---

## 4. 差速底盘：为什么一个 v 和 w 能换算成两个轮子的速度

这是本章最重要的算法部分。第一次看下面两个式子：

```text
v_left  = v - w * L / 2
v_right = v + w * L / 2
```

很容易只把它们当成需要背下来的公式。更好的理解方式是先看机械结构，再从“直行、转弯、原地转”三个运动状态推到一般公式。

### 4.1 先把机械结构、坐标轴和变量放到一张图里

本章讨论的是最典型的二维差速底盘：左右各有一个独立驱动轮，两轮轴线重合。运动学参考点 `C` 取在左右轮轴线的中点。

![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/c96d5ad6656d47b08c983c4bcebc97f5.png)


图中的量分别是：

```text
C        左右驱动轮轴线的中点，也是本章的运动学参考点
+X       机器人前进方向
+Y       机器人左侧方向
Z        垂直地面向上

v        C 点沿 +X 的线速度，单位 m/s
w        机器人绕 Z 轴的角速度，单位 rad/s
L        左右驱动轮中心距离，也叫 wheel separation / wheel track，单位 m
R        车轮半径，单位 m
v_left   左轮轮缘线速度，单位 m/s
v_right  右轮轮缘线速度，单位 m/s
```

按照右手定则，本章约定：

```text
w > 0  -> 逆时针 -> 向左转
w < 0  -> 顺时针 -> 向右转
```

这张图最重要的不是记住颜色，而是建立一个机械直觉：**`v` 描述整个车体“向前走多快”，`w` 描述整个车体“转得多快”，左右轮速度则是实现这两个运动要求的执行量。**

### 4.2 先看纯直线：左右轮为什么相等

如果：

```text
w = 0
```

机器人不旋转，只沿 +X 前进。刚体左右两侧没有额外的旋转速度分量，因此：

```text
v_left  = v
v_right = v
```

也就是：

```text
左轮和右轮一样快
-> 机器人直行
```

### 4.3 再看原地旋转：为什么一边正、一边负

如果：

```text
v = 0
w > 0
```

参考点 `C` 本身不向前移动，但整个底盘绕 `C` 逆时针旋转。

左右轮到 `C` 的距离都是：

```text
L / 2
```

旋转刚体上某一点的切向速度大小满足：

```text
线速度 = 角速度 * 到旋转中心的距离
```

所以左右轮速度绝对值都是：

```text
w * L / 2
```

但方向相反：

```text
v_left  = -w * L / 2
v_right =  w * L / 2
```

这就是原地左转时“左轮向后、右轮向前”的来源。

### 4.4 把“向前”和“旋转”叠加起来，就是一般公式

一般运动同时包含：

```text
整体向前速度 v
+
绕 C 的旋转速度 w
```

对于左轮：

```text
向前分量       = v
旋转产生的分量 = -w * L / 2
```

所以：

```text
v_left = v - w * L / 2
```

对于右轮：

```text
向前分量       = v
旋转产生的分量 = +w * L / 2
```

所以：

```text
v_right = v + w * L / 2
```

这两个值仍然是**轮缘线速度**，单位为 `m/s`。

电机、减速器或轮轴控制接口更常使用角速度 `rad/s`，因此还要除以车轮半径：

```text
omega_left  = v_left  / R
omega_right = v_right / R
```

合起来：

```text
omega_left  = (v - w * L / 2) / R
omega_right = (v + w * L / 2) / R
```

源码对应：

```cpp
// 逆运动学：先得到左右轮轮缘线速度，再除以车轮半径得到轮轴角速度。
const double left_linear_m_s =
    msg->linear.x - msg->angular.z * wheel_separation_ / 2.0;

const double right_linear_m_s =
    msg->linear.x + msg->angular.z * wheel_separation_ / 2.0;

WheelCommand command;
command.left_wheel_velocity_rad_s = left_linear_m_s / wheel_radius_;
command.right_wheel_velocity_rad_s = right_linear_m_s / wheel_radius_;
```

### 4.5 把当前例子完整算一遍

发送：

```text
v = 0.3 m/s
w = 0.4 rad/s
L = 0.5 m
R = 0.1 m
```

左轮轮缘线速度：

```text
v_left
= 0.3 - 0.4 * 0.5 / 2
= 0.3 - 0.1
= 0.2 m/s
```

右轮轮缘线速度：

```text
v_right
= 0.3 + 0.4 * 0.5 / 2
= 0.3 + 0.1
= 0.4 m/s
```

再转换成轮轴角速度：

```text
omega_left
= 0.2 / 0.1
= 2 rad/s
```

```text
omega_right
= 0.4 / 0.1
= 4 rad/s
```

因此应该得到：

```text
left  = 2 rad/s
right = 4 rad/s
```

右轮更快，所以机器人向左转。

### 4.6 为什么运动学参考点通常选在左右轮中点 C

前面的差速公式有一个经常被忽略的前提：`v` 和 `w` 描述的是**左右驱动轮轴线中点 `C`** 的运动，而不是车体上任意一点的运动。

如果 `base_link`、传感器安装点或者上层控制参考点恰好就在 `C`，事情最简单；如果它们位于车体其他位置，就不能只把同一个 `linear.x` 原样拿过来使用，而应该先做**刚体上不同参考点之间的速度变换**。

下面这张图把 `C`、任意点 `P` 以及偏移量放到同一张俯视图中：

![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/690fd507885e4162b4a5d60fcb75c26e.png)


设点 `P` 相对 `C` 的位置为：

```text
r_CP = (a, b)

a：沿机器人 +X 方向的偏移，向前为正
a > 0  -> P 在 C 前方
a < 0  -> P 在 C 后方

b：沿机器人 +Y 方向的偏移，向左为正
b > 0  -> P 在 C 左侧
b < 0  -> P 在 C 右侧
```

对于同一个刚体，两个点的速度满足经典关系：

$$
\mathbf v_P
=
\mathbf v_C
+
\boldsymbol\omega\times\mathbf r_{CP}
$$

本章只考虑平面运动，因此：

$$
\boldsymbol\omega=(0,0,w)
$$

标准差速底盘在轮轴中点 `C` 不能主动横移，所以：

$$
\mathbf v_C=(v_C,0)
$$

把叉乘展开以后得到：

$$
v_{P_x}=v_C-wb
$$

$$
v_{P_y}=wa
$$

这两个式子就是“参考点变化”最需要记住的结果。

#### 4.6.1 `P` 只向左或向右偏：前进速度会变化

假设：

```text
a = 0
b != 0
```

那么：

$$
v_{P_y}=0
$$

但：

$$
v_{P_x}=v_C-wb
$$

也就是说，点 `P` 虽然仍然只沿车体前后方向运动，但是转弯时它的前进速度已经和轮轴中点 `C` 不一样。

例如机器人正在左转：

```text
w > 0
```

如果 `P` 在左侧：

```text
b > 0
```

则：

```text
v_P_x < v_C
```

左侧是转弯内侧，走过的圆弧更短，所以速度更小。

如果 `P` 在右侧：

```text
b < 0
```

则：

```text
v_P_x > v_C
```

右侧是转弯外侧，圆弧半径更大，所以速度更大。

现在反过来考虑更接近工程使用的情况：**上层给的是点 `P` 的速度，而底层差速解算需要 `C` 点速度。**

由前式反推：

$$
v_C=v_{P_x}+wb
$$

例如：

```text
P 在 C 右侧 0.10 m：b = -0.10 m
上层要求 P 点前进速度：v_P_x = 0.30 m/s
角速度：w = 0.40 rad/s
```

则轮轴中点真正应该使用的速度为：

$$
v_C=0.30+0.40\times(-0.10)=0.26\ \text{m/s}
$$

如果轮距仍然是：

```text
L = 0.50 m
```

再进入差速公式：

$$
v_L=0.26-0.40\times\frac{0.50}{2}=0.16\ \text{m/s}
$$

$$
v_R=0.26+0.40\times\frac{0.50}{2}=0.36\ \text{m/s}
$$

这就是“先把 `P` 点速度换算回 `C`，再做左右轮分解”的完整过程。

#### 4.6.2 `P` 只向前或向后偏：转弯时会自然产生横向速度

假设：

```text
a != 0
b = 0
```

此时：

$$
v_{P_x}=v_C
$$

但会出现：

$$
v_{P_y}=wa
$$

例如：

```text
P 在 C 前方 0.20 m：a = 0.20 m
C 点前进速度：v_C = 0.30 m/s
机器人左转：w = 0.40 rad/s
```

那么：

$$
v_{P_y}=0.40\times0.20=0.08\ \text{m/s}
$$

也就是说，`P` 点的真实速度不是：

```text
linear.x = 0.30
linear.y = 0.00
```

而是：

```text
linear.x = 0.30 m/s
linear.y = 0.08 m/s
angular.z = 0.40 rad/s
```

这并不表示差速底盘突然获得了横移能力。

真正发生的是：**车体整体在旋转，所以位于轮轴中点前方的那个固定点会沿圆弧向侧面扫过。**

因此，如果某个上层模块声明：

```text
“这条 Twist 描述的是前方 P 点”
```

却同时在转弯时坚持：

```text
linear.y = 0
```

那么这个 Twist 与理想差速刚体运动并不一致。

工程上通常有两种处理方式：

1. 统一约定 `/cmd_vel` 的运动学参考点就是轮轴中点 `C`；
2. 如果业务确实必须以其他点 `P` 为参考，就传递完整平面 Twist，并先做刚体速度变换。

#### 4.6.3 一般情况：P 同时前后、左右都有偏移

如果：

```text
a != 0
b != 0
```

那么点 `P` 的速度同时包含两种变化：

$$
\boxed{
\begin{aligned}
v_{P_x} &= v_C-wb \\
v_{P_y} &= wa
\end{aligned}}
$$

反过来，从 `P` 换算回 `C`：

$$
\boxed{
\begin{aligned}
v_C &= v_{P_x}+wb \\
v_{C_y} &= v_{P_y}-wa
\end{aligned}}
$$

对于理想差速底盘，还必须满足：

$$
v_{C_y}=0
$$

所以给定的 `P` 点 Twist 应满足：

$$
v_{P_y}=wa
$$

这就是为什么“参考点换了，只改一个 `linear.x`”有时是不够的。

完整流程应当是：

```text
上层给出 P 点 Twist
        ↓
确认 P 相对 C 的位置 (a, b)
        ↓
如果 P 坐标轴和 C 坐标轴方向不同，先处理坐标轴旋转
        ↓
使用刚体速度关系把 P 点 Twist 换算到 C
        ↓
检查 C 点是否满足差速约束：v_C_y = 0
        ↓
取 v_C_x 和 w
        ↓
进入本章差速公式
        ↓
得到左、右轮目标速度
```

在 ROS 工程中，如果 `P` 和 `C` 对应不同 TF frame，而且两者不只是平移、还存在姿态旋转，那么除了上面的“参考点平移”外，还要对速度向量做坐标轴旋转。实际程序通常应使用已经维护好的 TF/tf2 变换，而不是在各个 Driver 中重复手写一套坐标变换公式。

#### 4.6.4 如果只是 Z 方向高度不同呢

假设某个 IMU 安装在车体上方：

```text
r_CP = (a, b, h)
```

而机器人仍然只有平面 yaw：

```text
omega = (0, 0, w)
```

则：

$$
\boldsymbol\omega\times\mathbf r_{CP}
=
(-wb,\ wa,\ 0)
$$

可以看到高度 `h` 不参与结果。

所以，只要仍然是本章的纯二维平面运动：

> **单纯的 Z 高度偏移不会改变左右轮差速解算。**

只有开始考虑 `roll`、`pitch`、车体颠簸、悬架运动或者完整三维角速度时，传感器高度才会进入更一般的刚体速度关系。

### 4.7 三轮、四轮还能直接使用这套公式吗

不能只数轮子数量。真正决定运动学公式的是：

- 哪些轮子主动驱动；
- 哪些轮子可以转向；
- 轮子能否侧向自由滚动；
- 每个轮子的滚动方向和安装角度；
- 底盘是否能够主动产生横向速度 `vy`。

下面三种结构看起来都可能有 3～4 个轮子，但运动学完全不同：

![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/f4715770ed3b45e9bf6b3dec304434ba.png)


先用表格建立整体认识：

| 机械结构 | 是否可以沿用本章差速模型 | 真正决定运动的量 |
| --- | --- | --- |
| 2 个主动轮 + 1 个万向脚轮 | 可以 | 左右两个主动轮速度 |
| 2 个主动轮 + 2 个万向脚轮 | 可以 | 左右两个主动轮速度 |
| 左右各 2 个主动轮、同侧等速 | 通常按 skid-steer / 四轮差速近似 | 左、右两侧轮组速度，但存在明显侧滑 |
| 三轮车式单轮转向 | 不可以直接套用 | 驱动速度 + 转向角 |
| 汽车式前轮转向 Ackermann | 不可以直接套用 | 车速 + 前轮转向几何 |
| 4 个麦克纳姆轮 | 不可以直接套用 | `vx`、`vy`、`w` 与四个轮速之间的运动学矩阵 |
| 3/4 个全向轮 | 不可以直接套用 | `vx`、`vy`、`w` 与各轮速度之间的运动学矩阵 |

#### 4.7.1 汽车式前轮转向：为什么不能靠左右轮差速决定转弯

典型汽车式底盘具有：

```text
前轮：可以改变转向角
后轮：方向固定，负责或参与驱动
```

最简单的入门模型称为 **bicycle model（自行车模型）**：把左右两个前轮等效成前轴中心的一个虚拟轮，把左右两个后轮等效成后轴中心的一个虚拟轮。

定义：

```text
v       车辆纵向速度
w       yaw 角速度
L_a     前后轴距（wheelbase）
delta   虚拟前轮转向角
R_c     车体参考点到瞬时转弯中心 ICR 的半径
```

无侧滑的理想几何关系为：

$$
R_c=\frac{L_a}{\tan\delta}
$$

因此曲率：

$$
\kappa=\frac{1}{R_c}=\frac{\tan\delta}{L_a}
$$

又因为：

$$
w=v\kappa
$$

所以：

$$
\boxed{w=\frac{v\tan\delta}{L_a}}
$$

这和差速底盘的思路完全不同。

差速底盘是：

```text
v + w
  ↓
直接分解为左轮速度、右轮速度
```

汽车式转向是：

```text
v + 期望曲率 / w
  ↓
先求需要的转向角 delta
  ↓
转向机构把前轮摆到对应角度
  ↓
驱动轮再提供车辆前进速度
```

例如：

```text
v   = 1.0 m/s
L_a = 1.0 m
希望 w = 0.5 rad/s
```

需要的曲率：

$$
\kappa=\frac{w}{v}=0.5\ \text{m}^{-1}
$$

因此：

$$
\tan\delta=L_a\kappa=0.5
$$

$$
\delta=\arctan(0.5)\approx26.6^\circ
$$

也就是说，这台车要通过“前轮大约转 26.6°”来获得这个转弯曲率，而不是像差速机器人那样故意让左轮慢、右轮快来转向。

##### 真正的 Ackermann 左右前轮转角为什么还不一样

四轮汽车转弯时，四个轮子绕的是**同一个瞬时转弯中心 ICR**，但是内侧前轮和外侧前轮到 ICR 的半径不同。

如果：

```text
T = 前轮轮距
R_c = 后轴中心到 ICR 的转弯半径
```

左转时，内侧前轮转角：

$$
\delta_{in}
=
\arctan\left(\frac{L_a}{R_c-T/2}\right)
$$

外侧前轮转角：

$$
\delta_{out}
=
\arctan\left(\frac{L_a}{R_c+T/2}\right)
$$

因此：

```text
delta_in > delta_out
```

内侧轮必须转得更“狠”，这样所有轮子的延长线才能尽量交于同一个 ICR，从而降低轮胎横向拖滑。

如果后轮由机械差速器连接，转弯时内、外后轮本来就会有不同转速，机械差速器会允许这种速度差；如果每个轮子由独立电机驱动，则控制器也可以根据各轮转弯半径分别给速度目标。

因此，Ackermann 中的“左右轮速度不同”是**转弯几何的结果**，不是像差速底盘那样用左右轮速度差直接作为主要转向手段。

#### 4.7.2 4 个麦克纳姆轮：为什么可以直接横移

麦克纳姆轮和普通橡胶轮的关键差别在于：轮胎外圈不是连续橡胶面，而是一圈带角度的自由滚子，常见安装角约为 45°。

普通轮的主要约束是：

```text
沿轮子滚动方向可以运动
侧向不能自由运动
```

而麦克纳姆轮的滚子允许某个斜向分量被“释放”。四个轮子的滚子方向经过特定排列后，各轮产生的速度分量可以组合出：

```text
vx：前后移动
vy：左右横移
w ：原地旋转
```

所以它的底盘命令天然是三自由度平面 Twist：

$$
\mathbf u=
\begin{bmatrix}
v_x\\
v_y\\
w
\end{bmatrix}
$$

而不是标准差速底盘只使用：

```text
v_x + w
```

对于一种常见的 X 型滚子排列，定义：

```text
FL：左前轮
FR：右前轮
RL：左后轮
RR：右后轮
r ：轮半径
l_x：机器人中心到前/后轮的 X 向距离
l_y：机器人中心到左/右轮的 Y 向距离
k = l_x + l_y
```

在这一套**特定轮子编号、正方向和滚子安装约定**下，可以写成：

$$
\begin{bmatrix}
\omega_{FL}\\
\omega_{FR}\\
\omega_{RL}\\
\omega_{RR}
\end{bmatrix}
=
\frac{1}{r}
\begin{bmatrix}
1 & -1 & -k\\
1 &  1 &  k\\
1 &  1 & -k\\
1 & -1 &  k
\end{bmatrix}
\begin{bmatrix}
v_x\\
v_y\\
w
\end{bmatrix}
$$

这个矩阵比记住每一项正负号更重要的，是理解三件事：

1. 每个轮子的速度都同时受到 `vx`、`vy`、`w` 影响；
2. 四个轮子的组合可以主动生成 `vy`，所以麦克纳姆可以横移；
3. 改变轮子编号、正转方向或滚子 `/`、`\\` 排列后，矩阵中的正负号会改变。

因此工程中不能从别人的代码复制一个麦克纳姆矩阵就直接使用，必须先确认：

```text
坐标轴定义是否一样？
轮子编号是否一样？
电机正方向是否一样？
滚子安装方向是否一样？
旋转中心是否一样？
```

一个简单的自检方法是分别给纯命令：

```text
(vx > 0, vy = 0, w = 0)   -> 应该只前进
(vx = 0, vy > 0, w = 0)   -> 应该只向左横移
(vx = 0, vy = 0, w > 0)   -> 应该只逆时针原地旋转
```

如果实际运动方向不对，优先检查轮序、正方向和滚子布局，而不是先怀疑 ROS Topic。

#### 4.7.3 3/4 个全向轮：为什么也是矩阵，而不是左右轮公式

全向轮（omni wheel）与麦克纳姆轮的共同点是：轮子自身可以沿一个方向主动滚动，同时依靠小滚子在另一个方向近似自由滑动。

如果把多个全向轮按不同角度布置在底盘周围，就可以让这些不同方向的驱动力组合成任意平面速度：

```text
vx
vy
w
```

以三个相隔 120° 安装的全向轮为例：

```text
轮 1：安装方向 alpha_1
轮 2：alpha_1 + 120°
轮 3：alpha_1 + 240°
```

每个轮子只能直接约束“沿自己驱动方向的速度分量”。因此第 `i` 个轮子的角速度可以写成：

$$
\omega_i
=
\frac{1}{r}
\left(
\sin\alpha_i\,v_x
-
\cos\alpha_i\,v_y
-
R_o\,w
\right)
$$

其中：

```text
r       全向轮半径
R_o     轮子中心到机器人运动学中心的距离
alpha_i 第 i 个轮子的安装/驱动方向角
```

把三个轮子的方程叠在一起，就得到：

$$
\begin{bmatrix}
\omega_1\\
\omega_2\\
\omega_3
\end{bmatrix}
=
\frac{1}{r}
\underbrace{
\begin{bmatrix}
\sin\alpha_1 & -\cos\alpha_1 & -R_o\\
\sin\alpha_2 & -\cos\alpha_2 & -R_o\\
\sin\alpha_3 & -\cos\alpha_3 & -R_o
\end{bmatrix}}
_{H}
\begin{bmatrix}
v_x\\
v_y\\
w
\end{bmatrix}
$$

这就是一个 `3 x 3` 的运动学矩阵。

只要三个轮子的安装方向设计得合理，使矩阵 `H` 满秩，就可以从任意期望的：

```text
(vx, vy, w)
```

唯一算出三只轮子的速度。

如果使用 4 个全向轮，就会得到一个 `4 x 3` 矩阵：

$$
\boldsymbol\omega
=
\frac{1}{r}H\mathbf u
$$

逆运动学仍然很直接：给定底盘速度，矩阵乘法得到每个轮速。

反过来由四个轮速估计：

```text
vx, vy, w
```

由于方程数量多于未知数，通常使用矩阵的伪逆：

$$
\mathbf u
=
rH^{\dagger}\boldsymbol\omega
$$

多出来的轮子并不是“多余”，它可以带来负载分担、机械布置和一定的冗余，但也意味着实际工程中更需要处理：

- 轮径差异；
- 某个轮子离地或打滑；
- 安装角误差；
- 多轮速度不完全一致时的最小二乘估计。

与麦克纳姆一样，全向轮矩阵中的正负号由坐标系、轮子安装方向和电机正方向决定，不能脱离机械图直接死记。

#### 4.7.4 为什么它们都可以接收 Twist，但 Driver 内部算法完全不同

这是 ROS 初学时很容易混淆的一点。

上层都可能使用类似：

```text
geometry_msgs/Twist
```

表达底盘期望运动：

```text
linear.x
linear.y
angular.z
```

但 Twist 只是在描述：

> “希望整个机器人怎样运动。”

它没有规定：

> “底盘必须用什么机械机构实现这个运动。”

所以同样一个：

```text
linear.x = 0.5
linear.y = 0.0
angular.z = 0.3
```

进入不同底盘 Driver 后可能变成完全不同的执行量：

```text
差速底盘
Twist
 -> 左轮角速度
 -> 右轮角速度
```

```text
Ackermann
Twist / 曲率目标
 -> 前轮转向角
 -> 驱动轮速度
```

```text
麦克纳姆
Twist
 -> FL / FR / RL / RR 四个轮速
```

```text
三轮全向
Twist
 -> wheel_1 / wheel_2 / wheel_3 三个轮速
```

因此应该把 ROS 消息和机械运动学分成两个层次理解：

```text
ROS Twist
“整机希望怎么动”
        ↓
具体底盘运动学模型
“这种机械结构要怎么实现”
        ↓
轮速 / 转向角 / 电机目标值
```

这也是为什么不能仅仅看到大家都订阅 `/cmd_vel`，就认为底层控制算法也是一样的。

### 4.8 推荐阅读：想继续看完整运动学推导时看什么

本章只保留 ROS Driver 实际需要的公式和机械直觉。需要继续深入时，可以按主题阅读：

1. **差速底盘完整推导：Introduction to Robotics and Perception — Differential Drive Motion Model**  
   https://www.roboticsbook.org/S52_diffdrive_actions.html

2. **刚体上两个点的速度关系：University of Illinois — Rigid bodies**  
   https://mechref.engr.illinois.edu/dyn/rkg.html

   这正是 4.6 节把任意参考点 `P` 换算到轮轴中点 `C` 所使用的基础关系。

3. **汽车式 / Ackermann 转向：ros2_control — Ackermann Steering Controller / Steering Controllers Library**  
   https://control.ros.org/master/doc/ros2_controllers/ackermann_steering_controller/doc/userdoc.html  
   https://control.ros.org/master/doc/ros2_controllers/steering_controllers_library/doc/userdoc.html

4. **麦克纳姆轮：WPILib — Mecanum Drive Kinematics**  
   https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/mecanum-drive-kinematics.html

5. **三轮/四轮全向轮矩阵：Modern Robotics — Omnidirectional Wheeled Mobile Robots**  
   https://modernrobotics.northwestern.edu/nu-gm-book-resource/13-2-omnidirectional-wheeled-mobile-robots-part-1-of-2/

6. **轮式机器人运动学总览：ROS 2 Control — Wheeled Mobile Robot Kinematics**  
   https://control.ros.org/humble/doc/ros2_controllers/doc/mobile_robot_kinematics.html

这些资料有的来自 ROS 2 或其他机器人软件栈，但这里引用的是刚体与轮式机器人运动学本身，与本项目使用 ROS1 Noetic 并不冲突。

---

## 5. `JointState`：左右轮的位置和速度怎么表达

查看：

```bash
rosmsg show sensor_msgs/JointState
```

结构：

```text
std_msgs/Header header
string[] name
float64[] position
float64[] velocity
float64[] effort
```

### 5.1 name

本项目：

```text
left_wheel_joint
right_wheel_joint
```

代码：

```cpp
msg.name = {left_joint_name_, right_joint_name_};
```

### 5.2 position

对于旋转关节：

```text
position 单位 = rad
```

示例设备用：

```cpp
// 角位置 = 对角速度按时间积分。
data_.left_wheel_position_rad += data_.left_wheel_velocity_rad_s * dt_s;
data_.right_wheel_position_rad += data_.right_wheel_velocity_rad_s * dt_s;
```

因为：

```text
角度变化 = 角速度 * 时间
rad      = rad/s * s
```

例如左轮保持：

```text
2 rad/s
```

持续：

```text
0.5 s
```

累计增加：

```text
1 rad
```

### 5.3 velocity

对于旋转关节：

```text
velocity 单位 = rad/s
```

所以可以直接填：

```cpp
msg.velocity = {
    data.left_wheel_velocity_rad_s,
    data.right_wheel_velocity_rad_s
};
```

### 5.4 effort 为什么留空

`effort` 对旋转关节通常表达力矩，单位 `N*m`。

本章没有力矩传感器，也没有真实电机电流到力矩的标定关系，所以不能因为字段存在就随便填：

```text
0
```

源码选择留空：

```cpp
// 当前示例没有力矩传感器，因此 effort 留空，而不是伪造 0 Nm 测量值。
joint_state_pub_.publish(msg);
```

这就是数据契约的核心原则：**没有数据就不要伪造数据。**

---

## 6. `Imu`：角速度、加速度和 orientation 分别是什么

查看：

```bash
rosmsg show sensor_msgs/Imu
```

主要字段：

```text
orientation
orientation_covariance

angular_velocity
angular_velocity_covariance

linear_acceleration
linear_acceleration_covariance
```

### 6.1 angular_velocity

本章只模拟：

```text
angular_velocity.z
```

单位：

```text
rad/s
```

它表示绕 IMU 自身 Z 轴的角速度。

### 6.2 linear_acceleration

单位必须是：

```text
m/s^2
```

示例假设 `imu_link` 使用常见的 x 前、y 左、z 上方向，底盘水平放置且没有额外线加速度，因此：

```text
x = 0
y = 0
z = 9.80665 m/s^2
```

这里只是教学数据，不代表真实 IMU 输出一定长这样。

### 6.3 orientation 没有数据怎么办

本章没有姿态融合算法，因此没有可靠的：

```text
roll
pitch
yaw orientation estimate
```

ROS `sensor_msgs/Imu` 有明确约定：如果没有某类估计，可以把对应 covariance 的第一个元素设为 `-1`。

所以源码：

```cpp
// 当前示例没有姿态解算结果。
// orientation_covariance[0] = -1 表示不要使用 orientation。
msg.orientation.w = 1.0;
msg.orientation_covariance[0] = -1.0;
```

这里比“随便填一个看起来正常的姿态”更正确。

---

## 7. 从左右轮反馈反推出底盘速度

前面做的是逆运动学：

```text
机器人 v、w
    -> 左右轮速度
```

Odometry 需要反过来：

```text
左右轮速度
    -> 机器人 v、w
```

已知轮轴角速度：

```text
omega_left
omega_right
```

先乘车轮半径得到轮缘线速度：

```text
v_left  = omega_left  * R
v_right = omega_right * R
```

底盘中心线速度：

```text
v = (v_right + v_left) / 2
```

底盘角速度：

```text
w = (v_right - v_left) / L
```

源码：

```cpp
// 正运动学：轮轴角速度 * 轮半径 = 轮缘线速度。
const double left_linear_m_s =
    data.left_wheel_velocity_rad_s * wheel_radius_;

const double right_linear_m_s =
    data.right_wheel_velocity_rad_s * wheel_radius_;

// 差速底盘中心线速度等于左右轮线速度平均值。
const double linear_m_s =
    (right_linear_m_s + left_linear_m_s) / 2.0;

// 左右轮速度差决定绕 Z 轴角速度。
const double angular_rad_s =
    (right_linear_m_s - left_linear_m_s) / wheel_separation_;
```

使用刚才的：

```text
omega_left  = 2 rad/s
omega_right = 4 rad/s
R = 0.1 m
L = 0.5 m
```

得到：

```text
v_left  = 0.2 m/s
v_right = 0.4 m/s
```

再得到：

```text
v = (0.4 + 0.2) / 2 = 0.3 m/s
w = (0.4 - 0.2) / 0.5 = 0.4 rad/s
```

和原始 `/cmd_vel` 一致。

---

## 8. `Odometry`：速度为什么还能变成 x、y、yaw

查看：

```bash
rosmsg show nav_msgs/Odometry
```

它有两大块：

```text
pose
    -> 现在在哪里、朝哪个方向

twist
    -> 现在以什么速度运动
```

本章使用：

```text
header.frame_id = odom
child_frame_id  = base_link
```

官方 `Odometry.msg` 约定：`pose` 使用 `header.frame_id` 表达的坐标系，`twist` 使用 `child_frame_id` 表达的坐标系。

### 8.1 先积分 yaw

已知：

```text
w = angular_rad_s
```

每个周期：

```text
yaw += w * dt
```

### 8.2 再积分 x 和 y

机器人自身前向速度是 `v`。

但在 `odom` 坐标系里，机器人当前已经有一个 `yaw`，所以要把车体前向速度投影到 `odom` 的 X/Y 方向：

```text
x_dot = v * cos(yaw)
y_dot = v * sin(yaw)
```

最简单的离散积分写成：

```text
x   += v * cos(yaw) * dt
y   += v * sin(yaw) * dt
yaw += w * dt
```

源码：

```cpp
// 用当前 yaw 把车体前向速度投影到 odom 坐标系，再进行简单欧拉积分。
odom_x_m_ += linear_m_s * std::cos(odom_yaw_rad_) * dt_s;
odom_y_m_ += linear_m_s * std::sin(odom_yaw_rad_) * dt_s;
odom_yaw_rad_ += angular_rad_s * dt_s;
```

这就是最基础的轮式里程计积分，也叫 dead reckoning（航位推算）：当前结果依赖上一次结果，再不断把新的速度积分进去。

### 8.3 yaw 为什么要变成 Quaternion

`Odometry.pose.pose.orientation` 不是直接存一个 yaw，而是：

```text
geometry_msgs/Quaternion
```

二维底盘只绕 Z 轴旋转，因此：

```text
q.x = 0
q.y = 0
q.z = sin(yaw / 2)
q.w = cos(yaw / 2)
```

源码：

```cpp
// 二维底盘只有 yaw，这里把 yaw 转成绕 Z 轴旋转的四元数。
msg.pose.pose.orientation.z = std::sin(odom_yaw_rad_ / 2.0);
msg.pose.pose.orientation.w = std::cos(odom_yaw_rad_ / 2.0);
```

TF 和完整坐标系关系放到后面的阶段 C 再系统学习。

### 8.4 理想公式为什么到了真实机器人上会越走越偏

前面的公式假设了很多理想条件：轮子纯滚动、参数准确、时间准确、地面平整、传感器没有噪声。

真实机器人并不满足这些条件，因此这条链路中每一层都可能产生误差：

```text
编码器
  |
  | 量化、丢脉冲、噪声
  v
轮子转角/轮速
  |
  | 轮径误差、轮胎变形、打滑
  v
左右轮线速度
  |
  | 轮距误差、skid-steer 侧滑
  v
底盘 v / w
  |
  | dt / 时间戳误差
  v
积分
  |
  v
x / y / yaw
```

这也是为什么轮式 Odometry 是**估计值**，而不是地面真实位置 Ground Truth。

下面把几种最常见的误差和工程处理方式展开。

#### 轮胎打滑

打滑时最典型的问题是：

```text
编码器看到轮子确实转了
!=
车体真的移动了同样的距离
```

例如急加速时驱动轮空转，编码器会认为机器人已经前进，但车体实际位移更小；急转弯、湿滑地面、四轮 skid-steer 原地转向时也很常见。

工程上通常不能只靠修改一个 `R` 或 `L` 参数解决，因为打滑是随工况变化的。常见处理包括：

- 限制速度、加速度和 jerk，减少突然打滑；
- 用 IMU、激光定位、视觉、GNSS 等外部信息和轮速里程计融合；
- 比较轮速、IMU 角速度、外部定位之间的一致性，检测疑似打滑；
- 检测到打滑时提高轮式 Odometry 的不确定度，让融合器少信它。

#### 编码器量化

编码器不是连续输出无限精度的角度，而是一格一格的脉冲。

假设每圈只有有限个 tick，那么低速时可能出现：

```text
这一周期 0 tick
下一周期 1 tick
再下一周期 0 tick
```

直接用很短时间窗求速度就会抖得很厉害。

工程上常见做法：

- 使用更高分辨率编码器；
- 中高速时在固定时间窗内计数 tick；
- 很低速时测相邻脉冲的时间间隔；
- 对速度估计做适当低通滤波或滑动平均；
- 增大统计窗口可以减小量化抖动，但同时会增加延迟，因此要折中。

#### 轮径误差

代码里的：

```text
R = 0.100 m
```

只是一个模型参数。真实有效轮径可能因为轮胎制造误差、气压、载荷、磨损而变成：

```text
左轮 0.099 m
右轮 0.101 m
```

这会导致同样的编码器转角被换算成错误距离。左右轮半径不一致时，即使发直行命令，里程计也会逐渐出现航向偏差。

工程上通常通过定距离直行和多次重复实验标定“有效轮径”，必要时左右轮分别保存参数：

```text
R_left
R_right
```

#### 轮距误差

公式中的 `L` 是决定角速度的关键参数：

```text
w = (v_right - v_left) / L
```

所以 `L` 偏小会把角速度估得偏大，`L` 偏大会把角速度估得偏小。

尤其是四轮 skid-steer 底盘，轮胎转弯时存在明显横向滑动，最合适的运动学参数往往是一个通过实验标定得到的：

```text
effective wheel separation
```

它不一定等于拿尺子量出来的几何轮距。

常见标定方式是让机器人多次原地转固定角度、走固定半径圆弧，再根据实际转角与编码器结果反推更合适的有效轮距。

#### 采样时间误差

积分公式里有：

```text
位置变化 = 速度 * dt
```

所以 `dt` 如果错了，位置一定跟着错。

例如程序“希望”以 20 Hz 运行，并不代表每一帧都严格是：

```text
0.050000 s
```

实际可能是：

```text
0.048 s
0.052 s
0.047 s
0.061 s
...
```

工程上不应该永远把 `dt` 写死成 `0.05`。更可靠的做法是使用真实采样/接收时间戳计算 `dt`，并对异常过大或过小的 `dt` 做保护。

第 13 章会从 TF / tf2 开始正式展开时间语义：先理解带时间戳的坐标变换、Buffer 缓存和查询，再在后续传感器融合中继续使用这些时间契约。

#### 地面不平

本章二维模型默认车体始终在同一个水平面运动。但真实地面可能有坡度、台阶、坑洼和轮胎离地。

这时编码器仍然只能描述轮子自身转了多少，不能直接告诉系统：

```text
车体是不是倾斜了
轮子是不是悬空了
实际水平位移是多少
```

工程上通常结合 IMU 姿态、悬架/接触信息或外部定位；在崎岖路面上也应降低纯轮式 Odometry 的可信度。如果任务本身就是三维地形运动，则需要更完整的 3D 状态估计，而不是继续强行使用纯二维模型解释所有现象。

### 8.5 工程上通常不是“把公式改得更复杂”就结束

面对这些误差，一个比较实用的工程思路是：

```text
系统性误差
-> 标定

高频随机噪声
-> 合理滤波

时变打滑 / 异常工况
-> 检测 + 降低可信度

单一传感器无法观测的问题
-> 多传感器融合

所有剩余不确定性
-> 用 covariance 明确告诉下游
```

因此下一节的 covariance 不是额外附加的数学概念，而是在回答一个非常现实的问题：

> 既然 Odometry 一定有误差，ROS 怎么把“这个结果有多不确定”一起传给后面的定位、融合和导航模块？

---

## 9. covariance：不仅要发布“估计值”，还要发布“对它有多不确定”

`Odometry` 中：

```text
pose.covariance
```

和：

```text
twist.covariance
```

都是：

```text
float64[36]
```

第一次看到 `36` 很容易误以为这是 36 个互不相关、需要一个个手工配置的参数。实际上它只是一个 `6 x 6` 协方差矩阵按行展开后的存储形式。

### 9.1 covariance 到底是干什么的

假设 Driver 发布：

```text
x = 5.03 m
```

这只回答了：

> 当前轮式里程计认为机器人在 `x = 5.03 m`。

但没有回答：

> 这个 5.03 m 到底有多可靠？

如果另外一个定位来源给出：

```text
x = 5.00 m
```

融合器必须知道应该更信哪一个。于是测量除了“值”，还需要携带“不确定度”。

可以先用下面这个入门心智模型理解：

```text
测量值
+
不确定度说明
=
下游能够合理融合的数据
```

严格地说，covariance 不是一个“可信度百分比”，但它承担的工程作用确实是描述误差规模以及不同误差之间的关系。

### 9.2 先理解 variance：一个变量自己的不确定度

假设让机器人重复走同一段 5 m 路程，得到：

```text
5.01 m
4.98 m
5.04 m
4.97 m
5.02 m
...
```

如果测量结果非常集中，说明误差波动小；如果每次结果差异很大，说明不确定性高。

方差 `variance` 用来描述这种波动大小。

对于 x：

```text
variance(x) = sigma_x^2
```

标准差和方差的关系：

```text
sigma_x = sqrt(variance(x))
```

例如：

```text
variance(x) = 0.0025 m^2
```

那么：

```text
sigma_x = sqrt(0.0025)
        = 0.05 m
```

标准差 `0.05 m` 比抽象的 `0.0025 m^2` 更容易建立直觉：误差的典型尺度大约是厘米级，而不是毫米级。

注意单位也会平方：

```text
x / y / z 的单位     m
对应 variance 单位   m^2

yaw 的单位           rad
对应 variance 单位   rad^2
```

### 9.3 covariance：两个误差是否会一起变化

只有方差还不够。

例如机器人转弯时，如果 `yaw` 估计偏大，积分到世界坐标系后 `x` 或 `y` 的误差也可能跟着变化。

这时需要描述：

```text
x_error 和 yaw_error 是否相关
y_error 和 yaw_error 是否相关
x_error 和 y_error 是否相关
```

这就是非对角线 covariance 的用途。

可以先这样区分：

```text
variance
-> 一个变量自己的误差有多大

covariance
-> 两个变量的误差是否会一起变化
```

### 9.4 为什么 ROS 恰好使用 6 x 6 = 36 个数

对于 `pose`，ROS 使用 6 个自由度描述刚体位姿：

```text
x
y
z
roll
pitch
yaw
```

这 6 个变量两两组合，得到一个：

```text
6 x 6 covariance matrix
```

官方 `geometry_msgs/PoseWithCovariance.msg` 也明确说明，它采用 **row-major 的 6x6 covariance matrix**，顺序为：

```text
x, y, z, rotation about X, rotation about Y, rotation about Z
```

对应二维移动机器人常说的：

```text
x, y, z, roll, pitch, yaw
```

![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/7d9d275dc3e34a428f078dd4b90bdc7f.png)


矩阵概念上是：

```text
                 x          y          z        roll       pitch       yaw
x             var(x)     cov(x,y)      ...       ...         ...     cov(x,yaw)
y            cov(y,x)    var(y)        ...       ...         ...     cov(y,yaw)
z               ...        ...       var(z)      ...         ...       ...
roll            ...        ...         ...     var(roll)      ...       ...
pitch           ...        ...         ...       ...      var(pitch)    ...
yaw          cov(yaw,x) cov(yaw,y)     ...       ...         ...     var(yaw)
```

对角线表示“每个变量自己的方差”；非对角线表示“两个变量误差的协方差”。

官方定义：

- https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseWithCovariance.html

### 9.5 36 个数是怎么放进 `float64[36]` 的

ROS 不是把二维矩阵直接存进消息，而是采用 row-major（按行优先）展开：

```text
矩阵第 0 行 -> 数组 [0..5]
矩阵第 1 行 -> 数组 [6..11]
矩阵第 2 行 -> 数组 [12..17]
...
```

因此对角线索引正好是：

```text
[0]  -> x
[7]  -> y
[14] -> z
[21] -> roll
[28] -> pitch
[35] -> yaw
```

源码：

```cpp
// 6x6 covariance 的顺序是 x, y, z, roll, pitch, yaw。
// 按行展开后，对角线索引就是 0, 7, 14, 21, 28, 35。
msg.pose.covariance[0] = odom_pose_xy_variance_;
msg.pose.covariance[7] = odom_pose_xy_variance_;
msg.pose.covariance[14] = unobserved_variance_;
msg.pose.covariance[21] = unobserved_variance_;
msg.pose.covariance[28] = unobserved_variance_;
msg.pose.covariance[35] = odom_pose_yaw_variance_;
```

这里之所以只填对角线，是因为当前教学示例只给每个维度一个独立的经验方差，没有建立 `x/y/yaw` 之间完整的相关误差模型。其余元素保持 0，相当于在这个简化模型中不描述交叉相关性。

`twist.covariance` 也是 `6 x 6`，但六个变量对应的是速度：

```text
linear.x
linear.y
linear.z
angular.x
angular.y
angular.z
```

也就是：

```text
vx, vy, vz, wx, wy, wz
```

### 9.6 这些方差是自己随便填的吗

真实产品里不应该随便填。

本项目配置文件中的数字目前是**教学参数**，目的是让消息结构和参数使用方式可见，不代表某台真实底盘已经经过统计标定。

工程上常见的来源可以分成四类。

#### 来源一：重复实验统计

这是最直观的方法。

例如需要估计 `x` 的方差，可以让机器人在相同条件下重复走：

```text
真实距离 = 5.000 m
```

用更可靠的 Ground Truth（例如高精度定位、运动捕捉、标定场地测量等）记录真实值，同时记录轮式 Odometry：

```text
第 1 次 odom_x = 5.03 m
第 2 次 odom_x = 4.96 m
第 3 次 odom_x = 5.02 m
...
```

先计算每次误差：

```text
error_i = odom_i - ground_truth_i
```

再计算误差样本方差：

```text
variance = sum((error_i - mean_error)^2) / (N - 1)
```

也可以写成：

```text
s^2 = sum((e_i - e_mean)^2) / (N - 1)
```

如果得到：

```text
variance_x = 0.0025 m^2
```

这里还要区分 **bias（系统偏差）** 和 **variance（随机波动）**。

如果：

```text
mean_error = +0.08 m
```

说明每次结果虽然可能很集中，但整体长期偏大约 8 cm。这类系统偏差首先应该通过轮径、轮距、比例因子等标定去消除，而不是指望 covariance 把错误的均值“修正回来”。Covariance 主要描述的是围绕均值的剩余不确定性。

另外，5 m 直行实验得到的统计量只代表相近工况。距离更长、速度更高、转弯更多或地面条件变化时，误差通常会继续累积，因此成熟系统会按运动模型传播 covariance，而不是把所有工况永久固定成同一个常数。

有了这些前提，这个实验才能给出比“凭感觉写一个 0.01”更有依据的：

```cpp
msg.pose.covariance[0] = 0.0025;
```

`yaw` 也一样。可以让机器人重复执行固定角度旋转，例如 `90°`、`360°`，用外部参考获得真实角度，再统计：

```text
yaw_error = odom_yaw - ground_truth_yaw
```

最后得到 `yaw` 误差的方差。

如果同时记录：

```text
x_error
y_error
yaw_error
```

还可以进一步统计 `cov(x,y)`、`cov(x,yaw)`、`cov(y,yaw)`，而不仅仅是三个对角线方差。

#### 来源二：传感器或执行机构的噪声规格

某些传感器的数据手册会给噪声密度、重复性、分辨率等指标。这些指标可以作为建立输入噪声模型的依据。

对轮式底盘，还可能从：

```text
encoder 分辨率
轮径标定误差
轮距标定误差
采样时间不确定性
```

建立输入误差模型。

但数据手册参数通常不能直接原封不动填进 `Odometry.pose.covariance`，因为中间还经过了运动学换算和积分。

#### 来源三：用误差传播模型算出来

如果已经知道输入变量的 covariance：

```text
P_in
```

又知道输出由某个函数：

```text
y = f(x)
```

计算得到，那么一阶近似下常使用 Jacobian 做误差传播：

```text
P_out = J * P_in * J^T
```

这里：

```text
J = f 对输入变量的 Jacobian
```

对于差速底盘，可以把左右轮速度/位移的不确定性传播到：

```text
v
w
x
y
yaw
```

这已经进入“概率机器人学/状态估计”的内容，本章只建立概念，不要求现在推导 Jacobian。

#### 来源四：状态估计器实时维护

EKF、UKF 等状态估计器内部会实时维护状态 covariance：

```text
P
```

随着：

```text
预测
测量更新
传感器失效
长时间没有外部定位
```

这个 `P` 会动态变化。

因此成熟系统里的 covariance 不一定是启动时固定写死的常量。

### 9.7 打滑时为什么可以把 covariance 调大

前面已经看到：轮胎打滑时，编码器和真实车体运动会发生分离。

如果系统能够检测到：

```text
轮速变化很大
但 IMU / 外部定位并不支持相同运动
```

就有理由认为当前轮式 Odometry 比正常状态更不可靠。

一种工程策略是动态提高相关维度的 covariance，让下游融合器降低对这一路测量的权重。

例如概念上：

```text
正常直行
-> wheel odom covariance 较小

高速 / 急转弯
-> 适当增大

明显打滑
-> 显著增大，或者暂时拒绝该测量
```

注意：具体阈值和增大量必须来自系统实验与融合器设计，不能把某个示例数字复制到所有机器人上。

### 9.8 为什么 z、roll、pitch 在本示例里给很大的方差

当前轮式 Odometry 主要估计：

```text
x
y
yaw
```

仅凭二维轮编码器，不能可靠观测：

```text
z
roll
pitch
```

因此示例把这些未观测维度设置成一个很大的方差：

```text
1000000.0
```

它不是说“真实误差一定等于一百万”，而是在当前教学模型里明确表达：

> 这一路数据源对这些维度几乎没有可信观测，不应该依赖它。

不同下游状态估计器可能有自己的配置和测量选择规则，所以真实项目还需要结合具体融合器文档决定哪些维度启用、哪些维度禁用，而不是只依赖一个“大数字”。

### 9.9 当前代码里的 covariance 应该怎样理解

因此，看到：

```cpp
odom.pose.covariance[0] = 0.02;
odom.pose.covariance[7] = 0.02;
odom.pose.covariance[35] = 0.05;
```

应该理解成：

> 这些是为了演示 ROS 消息和参数配置而给出的教学假设，不是“ROS 官方推荐值”，也不是已经针对某台真实底盘测量出来的结果。

真实产品的参数应尽量来自：

```text
实际重复实验统计
+
传感器/执行机构噪声模型
+
误差传播
+
状态估计器运行结果
```

最后可以把这一节压缩成两句话：

```text
Odometry 的数值回答：机器人现在大概在哪里、怎么运动。
Covariance 回答：这些判断分别有多不确定，以及它们的误差之间是否相关。
```

---

## 10. `DiagnosticArray` 本章只做最小状态表达

本章发布：

```text
/diagnostics
```

类型：

```text
diagnostic_msgs/DiagnosticArray
```

其中一个 `DiagnosticStatus` 包含：

```text
level
name
message
hardware_id
values[]
```

示例只表达：

```text
data.online == true
    -> OK

data.online == false
    -> ERROR
```

源码：

```cpp
status.level = data.online
    ? diagnostic_msgs::DiagnosticStatus::OK
    : diagnostic_msgs::DiagnosticStatus::ERROR;
```

这还不是完整的 Driver 健康管理。

真实 Driver 的可靠性专题还需要继续增加：

```text
timeout
watchdog
offline
frequency check
reconnect / reopen
恢复后的状态同步
```

---

## 11. 构建并运行新的第 12 章

因为新增了标准消息依赖，先在 Host 重新构建 Docker Image：

```bash
cd /home/wdfk/share/ros1-docker
docker compose up -d --build
```

进入 Container：

```bash
docker compose exec ros1-dev bash
```

构建 package：

```bash
cd /workspace/ros_ws
catkin build ros1_driver_lab
```

构建成功后：

```bash
source /workspace/ros_ws/devel/setup.bash
```

运行：

```bash
roslaunch ros1_driver_lab chassis_lab.launch
```

这个版本只有一个 C++ Node，不再启动 Python TCP simulator，因此也不存在上一版 `argparse` 无法识别 `__name:=...` / `__log:=...` 的问题。

---

## 12. 发送 `/cmd_vel`，实际观察左右轮结果

另开一个 Container 终端：

```bash
source /workspace/ros_ws/devel/setup.bash
```

发送：

```bash
rostopic pub -r 5 /cmd_vel geometry_msgs/Twist \
  "linear: {x: 0.3, y: 0.0, z: 0.0}
angular: {x: 0.0, y: 0.0, z: 0.4}"
```

Driver 日志应该持续显示接近：

```text
v=0.300 m/s
w=0.400 rad/s
left=2.000 rad/s
right=4.000 rad/s
```

这正是前面手工推导的结果。

查看左右轮：

```bash
rostopic echo /joint_states
```

重点观察：

```text
name
position
velocity
```

其中 `position` 会随着时间持续累计。

查看 IMU：

```bash
rostopic echo /imu/data_raw
```

重点观察：

```text
angular_velocity.z
linear_acceleration.z
orientation_covariance[0]
```

查看 Odometry：

```bash
rostopic echo /odom
```

重点观察：

```text
pose.pose.position.x
pose.pose.position.y
pose.pose.orientation

twist.twist.linear.x
twist.twist.angular.z
```

查看 diagnostics：

```bash
rostopic echo /diagnostics
```

---

## 13. 现在应该怎样理解这个 Driver

第 12 章最后只需要建立下面这条主线：

```text
/cmd_vel
geometry_msgs/Twist
    |
    | linear.x = v
    | angular.z = w
    v
差速底盘逆运动学
    |
    v
左右轮目标角速度

--------------------------------

底层已经取得的 ChassisData
    |
    +--> JointState
    |
    +--> Imu
    |
    +--> 左右轮正运动学
            |
            v
         v / w
            |
            v
         Odometry
    |
    +--> DiagnosticArray
```

到这里，Driver 最核心的数据契约已经建立。

阶段 B 到这里结束。原计划中的时间同步和 Driver 可靠性内容不再各自拆成独立章节，而是在后续真正需要它们的地方按数据消费者视角继续展开。

阶段 C 直接从第 13 章开始，追问第 12 章已经出现但还没有建立完整关系的另一件事：

> `/odom` 写了 `frame_id=odom`、`child_frame_id=base_link`，`/imu/data_raw` 写了 `frame_id=imu_link`，这些名字怎样真正组成 `map -> odom -> base_link -> sensor` 的坐标树？为什么同一份数据在不同时间查询 TF 还可能成功或失败？

这就是第 13 章 TF / tf2 的入口。
