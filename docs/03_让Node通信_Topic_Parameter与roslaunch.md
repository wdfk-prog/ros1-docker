<meta name="referrer" content="no-referrer" />

# 03：让 Node 真正通信——Topic、自定义 `.msg`、Parameter、日志与 roslaunch

> 摘要：从手工启动 Publisher/Subscriber 开始，学习 Topic、标准/自定义 .msg、Parameter、日志与 roslaunch，并用命令观察真实 ROS 图。

[TOC]

## 本章目标

完成本章后，你应该能够：

1. 理解 `roscore`、ROS Master、Parameter Server、`rosout` 的关系；
2. 会使用 `rosrun` 启动单个 Node；
3. 会使用 `rosnode list/info` 查看 Node；
4. 会使用 `rostopic list/info/echo/hz` 查看 Topic；
5. 理解 `std_msgs/String` 与 `std_msgs/msg/String.msg` 的关系；
6. 会定义 `HelloStatus.msg`，并理解 `.msg` 如何生成 C++ 消息类型；
7. 理解 Topic 数据为什么不是由 Master 转发；
8. 理解 private parameter `~publish_rate`；
9. 会使用 `roslaunch` 一次启动多个 Node；
10. 会区分终端日志、`/rosout` 和 `~/.ros/log`。

## 为什么先手工开多个终端

`roslaunch` 很方便，但如果一开始就只会：

```bash
roslaunch ros1_hello hello.launch
```

就很容易把下面这些进程关系当成一个黑盒：

```text
ROS Master
Publisher Node
Subscriber Node
rosout
```

所以本章先用多个终端把系统拆开，确认每一层都能工作，再使用 launch 文件把步骤自动化。

---

## Step 1：准备多个 Container 终端

在 Host 可以重复执行：

```bash
docker compose exec ros1-dev bash
```

或者在 VS Code Remote-SSH 窗口中选择：

```text
Terminal
→ New Terminal
→ ROS1 Container Bash
```

准备至少四个终端：

```text
Terminal A：roscore
Terminal B：hello_node
Terminal C：hello_listener
Terminal D：观察 rosnode / rostopic / rosparam
```

每个新 shell 都应该能够看到：

```bash
echo "$ROS_DISTRO"
```

输出：

```text
noetic
```

如果 `ros1_hello` 刚刚构建完成，但新终端找不到它，可以手工：

```bash
source /workspace/ros_ws/devel/setup.bash
```

---

## Step 2：启动 `roscore`

Terminal A：

```bash
roscore
```

不要关闭这个终端。

另开 Terminal D：

```bash
rosnode list
```

通常能看到：

```text
/rosout
```

### `roscore` 到底启动了什么

可以先把它理解为一个“基础运行时集合”。最重要的组件包括：

```text
ROS Master
    → Node 注册、Topic/Service 名称发现

Parameter Server
    → ROS 参数存储

rosout
    → ROS 日志汇聚
```

ROS Master 最关键的作用是：

```text
注册与发现
```

它不是所有 Topic 数据的中央转发服务器。

---

## Step 3：启动 Publisher `hello_node`

Terminal B：

```bash
rosrun ros1_hello hello_node
```

应该持续看到：

```text
[INFO] ... hello ros1 from docker: 0
[INFO] ... hello ros1 from docker: 1
...
```

### `rosrun` 的两个参数

```bash
rosrun ros1_hello hello_node
```

可以读成：

```text
package:    ros1_hello
executable: hello_node
```

`rosrun` 会根据 ROS package 环境找到 package，再找到它构建出的 executable。

### 验证 Node 已经注册

Terminal D：

```bash
rosnode list
```

应该出现：

```text
/hello_node
/rosout
```

再：

```bash
rosnode info /hello_node
```

你应该能看到它发布的 Topic 信息。

---

## Step 4：查看 `/chatter`

Terminal D：

```bash
rostopic list
```

应该出现：

```text
/chatter
/rosout
/rosout_agg
```

查看 Topic 类型：

```bash
rostopic type /chatter
```

应该是：

```text
std_msgs/String
```

### `std_msgs/String` 背后其实就是 `.msg`

这里第一次正式遇到 ROS 的接口描述文件。执行：

```bash
rosmsg show std_msgs/String
```

可以看到：

```text
string data
```

这份结构来自 `std_msgs` package 中的 `msg/String.msg`。代码里的：

```cpp
#include <std_msgs/String.h>
std_msgs::String msg;
```

并不是 `std_msgs` 的作者手工维护了一份和 `String.msg` 平行的 C++ 协议结构，而是 ROS 根据 `.msg` 生成了对应的语言绑定。

可以先把 `.msg` 理解成一种 **IDL（Interface Definition Language）式的数据契约**：

```text
String.msg
    ↓ 描述字段
string data
    ↓ message_generation / genmsg
生成 C++ / Python 等语言类型
    ↓
std_msgs::String
```

它和 protobuf、Thrift、CORBA IDL 的共同点，是先用语言无关的形式定义跨进程交换的数据结构，再生成具体语言可使用的类型。这里的类比只用于建立接口描述的心智模型，并不表示 ROS `.msg` 与这些系统具有相同的 RPC 或传输协议。

更重要的是，要把 **消息类型** 和 **Topic** 分开：

```text
.msg
    -> 定义“一条消息里有哪些字段、每个字段是什么类型”

Topic
    -> 定义“这种消息通过哪个 ROS 名称进行发布/订阅”
```

因此：

```text
std_msgs/String
```

是消息类型，而：

```text
/chatter
```

是 Topic 名称。`.msg` 本身既不是 Topic，也不负责指定 TCP/UDP 端口。同一种消息类型可以被多个 Topic 复用，例如多个状态 Topic 都可以使用相同的消息类型；Publisher 和 Subscriber 真正建立连接时，则需要对 Topic 使用的消息类型达成一致。

ROS 生成代码还不只是为了“少写一个 struct”。由接口描述生成的类型会进入后续的序列化、反序列化、类型信息与连接协商流程。应用层因此通常不需要自己维护：

```text
字段布局
C++ / Python 对应类型
序列化 / 反序列化入口
ROS 消息类型信息
```

但这并不改变 Topic 的通信语义。Topic 仍然是：

```text
Publisher
   │
   ├──── message ────→ Subscriber A
   ├──── message ────→ Subscriber B
   └──── message ────→ Subscriber C
```

Publisher 表达的是“这里产生了一条数据或事件”，通常不会等待某个 Subscriber 返回一次结果。因此 `/camera/image`、`/imu/data`、`/scan`、`/odom`、`/motor_state` 这类连续数据天然适合 Topic。

直接看消息：

```bash
rostopic echo /chatter
```

典型输出：

```text
data: "hello ros1 from docker: 3"
---
data: "hello ros1 from docker: 4"
---
```

查看频率：

```bash
rostopic hz /chatter
```

默认 `publish_rate=1.0` 时，频率应接近：

```text
1 Hz
```

### 这里发生了什么

Publisher 中：

```cpp
publisher.publish(msg);
```

不是把每条数据先送给 ROS Master，再由 Master 转发。

简化流程更接近：

```text
Publisher 向 Master 注册：
    我发布 /chatter

Subscriber 向 Master 查询：
    谁发布 /chatter？

Master 返回 Publisher 的联系信息

Publisher 与 Subscriber 建立数据连接

后续消息：
Publisher ==================> Subscriber
             TCPROS/UDPROS
```

所以 Master 主要负责控制面和发现，真正消息 payload 通常是 Node 之间直接传输。

---

## Step 5：启动 Subscriber `hello_listener`

Terminal C：

```bash
rosrun ros1_hello hello_listener
```

应该开始看到：

```text
[INFO] ... received: hello ros1 from docker: 10
[INFO] ... received: hello ros1 from docker: 11
...
```

### Subscriber 的关键代码

```cpp
ros::Subscriber subscriber =
    nh.subscribe("chatter", 10, chatterCallback);
```

表示：

```text
订阅 Topic：chatter
接收队列：10
回调函数：chatterCallback
```

然后：

```cpp
ros::spin();
```

持续处理 callback queue。

如果没有 `ros::spin()` 或其它等价的 callback 处理机制，消息即使已经到达底层连接，用户回调也不会被正常持续执行。

### 看 Topic 的 Publisher/Subscriber

Terminal D：

```bash
rostopic info /chatter
```

应该看到：

```text
Publishers:
 * /hello_node

Subscribers:
 * /hello_listener
```

这比只看终端输出更有价值，因为它直接证明 ROS 图中的连接关系。

---

## Step 6：自己定义一个 `.msg`，理解 Topic 的数据契约

前面的 `/chatter` 使用 ROS 已经提供好的 `std_msgs/String`。现在单独增加一个自定义消息实验，但**不修改原 `/chatter` 示例**，这样后续源码阅读仍然可以继续沿用最简单的 `std_msgs/String`。

### 6.1 定义 `HelloStatus.msg`

新增文件：

```text
ros_ws/src/ros1_hello/msg/HelloStatus.msg
```

内容：

```text
int32 sequence
string text
```

这相当于声明一条消息的数据契约：

```text
HelloStatus
├── sequence : int32
└── text     : string
```

`.msg` 没有 Request/Response，也没有 Goal/Result/Feedback。它只描述“一条消息”的字段。

### 6.2 让 catkin 根据 `.msg` 生成代码

`CMakeLists.txt` 中加入消息生成组件：

```cmake
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
)
```

声明要生成的消息：

```cmake
add_message_files(
  FILES
  HelloStatus.msg
)

generate_messages()
```

并把运行期依赖导出：

```cmake
catkin_package(
  CATKIN_DEPENDS message_runtime roscpp std_msgs
)
```

`package.xml` 对应增加：

```xml
<build_depend>message_generation</build_depend>
<build_export_depend>message_runtime</build_export_depend>
<exec_depend>message_runtime</exec_depend>
```

使用生成头文件的 target 还要依赖消息生成 target：

```cmake
add_dependencies(custom_msg_publisher
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
```

Subscriber 同理。

构建：

```bash
cd /workspace/ros_ws
catkin build ros1_hello
source /workspace/ros_ws/devel/setup.bash
```

先检查 ROS 已经认识这个类型：

```bash
rosmsg show ros1_hello/HelloStatus
```

预期：

```text
int32 sequence
string text
```

再找生成的 C++ 头文件：

```bash
find /workspace/ros_ws/devel -path '*/ros1_hello/HelloStatus.h' -print
```

在当前 merge-devel 配置下，通常可以看到：

```text
/workspace/ros_ws/devel/include/ros1_hello/HelloStatus.h
```

因此源码才能写：

```cpp
#include <ros1_hello/HelloStatus.h>

ros1_hello::HelloStatus msg;
msg.sequence = 1;
msg.text = "hello from custom .msg";
```

`HelloStatus.h` 是构建产物，不需要也不应该手工维护。

### 6.3 使用自定义消息发布 `/hello_status`

仓库新增：

```text
src/custom_msg_publisher.cpp
```

核心代码：

```cpp
ros::Publisher publisher =
    nh.advertise<ros1_hello::HelloStatus>("hello_status", 10);

ros1_hello::HelloStatus msg;
msg.sequence = sequence++;
msg.text = "hello from custom .msg";
publisher.publish(msg);
```

启动：

```bash
rosrun ros1_hello custom_msg_publisher
```

另一个终端观察：

```bash
rostopic type /hello_status
rosmsg show ros1_hello/HelloStatus
rostopic echo /hello_status
```

应该能看到类似：

```text
sequence: 3
text: "hello from custom .msg"
---
```

### 6.4 Subscriber 使用的是同一个生成类型

仓库同时新增：

```text
src/custom_msg_subscriber.cpp
```

它的 callback 类型为：

```cpp
static void statusCallback(
    const ros1_hello::HelloStatus::ConstPtr& msg)
```

启动：

```bash
rosrun ros1_hello custom_msg_subscriber
```

Subscriber 会读取相同的两个字段：

```text
received: sequence=4 text=hello from custom .msg
```

到这里可以把 Topic 的类型链整理为：

```text
HelloStatus.msg
    ↓ message_generation
ros1_hello/HelloStatus.h
    ↓
ros1_hello::HelloStatus C++ 对象
    ↓ publish()
/hello_status Topic
    ↓ TCPROS（当前默认情况）
ros1_hello::HelloStatus C++ 对象
    ↓
Subscriber callback
```

`.msg` 解决的是**数据契约、代码生成、序列化/反序列化所需的类型一致性**，不是另外发明一种传输协议。真正的通信模型仍然是 Topic 的异步发布/订阅。

因此，第 03 章先建立第一块接口模型：

```text
HelloStatus.msg
    ↓
定义 payload
    ↓
生成 ros1_hello::HelloStatus
    ↓
publish()/subscribe()
    ↓
Topic 异步数据流
```

阶段 11 再把它扩展为完整的 ROS1 接口描述地图：

```text
.msg
    -> 一条消息的数据契约
    -> Topic 数据流

.srv
    -> Request --- Response
    -> RPC 式请求/响应

.action
    -> Goal --- Result --- Feedback
    -> 再叠加 Cancel/Preempt 与状态机
    -> 可观测、可取消的长任务
```

三者都能减少手工维护跨语言数据结构、序列化类型和接口一致性的工作，但三者之间真正重要的差异不是“谁更省代码”，而是**通信语义不同**。

---

## Step 7：理解 `rosnode` 和 `rostopic` 的边界

初学时很容易把命令混在一起。

### 看 Node

```bash
rosnode list
rosnode info /hello_node
rosnode ping /hello_node
```

关注的是：

```text
谁在运行？
这个 Node 发布/订阅什么？
Node 是否可达？
```

### 看 Topic

```bash
rostopic list
rostopic info /chatter
rostopic echo /chatter
rostopic hz /chatter
```

关注的是：

```text
有哪些数据通道？
消息类型是什么？
谁发布？谁订阅？
数据内容和频率是什么？
```

这两个维度应该同时会看。

---

## Step 8：理解 private parameter `~publish_rate`

Publisher 中：

```cpp
ros::NodeHandle pnh("~");

double publish_rate = 1.0;
pnh.param("publish_rate", publish_rate, 1.0);
```

`"~"` 表示 Node 私有命名空间。

Node 名为：

```text
/hello_node
```

所以：

```text
~publish_rate
```

解析后通常是：

```text
/hello_node/publish_rate
```

在当前手工 `rosrun` 场景里，如果没有设置参数：

```cpp
pnh.param(..., 1.0);
```

会使用默认值 `1.0`。

### 手工设置参数

Terminal D：

```bash
rosparam set /hello_node/publish_rate 5.0
```

但要注意：当前示例只在启动时读取一次参数：

```cpp
pnh.param(...)
```

它不会自动持续监听参数变化。

因此如果 Node 已经启动，再 set 参数，当前 `ros::Rate` 不会自动变成 5 Hz。

停止并重新启动 Publisher 后再观察：

```bash
rostopic hz /chatter
```

---

## Step 9：用 launch 文件把多个进程组织起来

手工终端已经证明通信链路正确，现在再打开：

```text
ros_ws/src/ros1_hello/launch/hello.launch
```

内容：

```xml
<launch>
  <arg name="publish_rate" default="1.0"/>

  <node pkg="ros1_hello"
        type="hello_node"
        name="hello_node"
        output="screen">
    <param name="publish_rate" value="$(arg publish_rate)"/>
  </node>

  <node pkg="ros1_hello"
        type="hello_listener"
        name="hello_listener"
        output="screen"/>
</launch>
```

先看最外层：

```xml
<launch>
...</launch>
```

它表示一个 launch 描述文件。

### `<arg>`

```xml
<arg name="publish_rate" default="1.0"/>
```

是 launch 参数。

启动时可以覆盖：

```bash
roslaunch ros1_hello hello.launch publish_rate:=5.0
```

### `<node>`

```xml
<node pkg="ros1_hello"
      type="hello_node"
      name="hello_node"
      output="screen">
```

可以按字段读：

```text
pkg
    package 名

type
    executable 名

name
    ROS 运行时 Node 名

output="screen"
    把输出显示到 roslaunch 终端
```

### `<param>`

```xml
<param name="publish_rate" value="$(arg publish_rate)"/>
```

因为它写在 `hello_node` 的 `<node>` 内，所以这个参数位于 Node private namespace：

```text
/hello_node/publish_rate
```

正好对应 C++ 中：

```cpp
ros::NodeHandle pnh("~");
pnh.param("publish_rate", ...);
```

---

## Step 10：让 `roslaunch` 启动整个示例

先把之前手工启动的 `hello_node`、`hello_listener`、`roscore` 都 Ctrl+C 停掉。

然后只执行：

```bash
roslaunch ros1_hello hello.launch
```

如果当前没有 ROS Master，`roslaunch` 会处理启动所需的 Master。

另开终端：

```bash
rosnode list
```

应该能看到：

```text
/hello_listener
/hello_node
/rosout
```

再：

```bash
rostopic hz /chatter
```

然后用不同频率：

```bash
roslaunch ros1_hello hello.launch publish_rate:=5.0
```

再次：

```bash
rostopic hz /chatter
```

应接近 5 Hz。

这一步把完整路径串起来了：

```text
命令行 publish_rate:=5.0
        ↓
launch <arg>
        ↓
<param name="publish_rate" ...>
        ↓
/hello_node/publish_rate
        ↓
pnh.param("publish_rate", ...)
        ↓
ros::Rate(5.0)
```

前面自定义 `.msg` 的两个 Node 也提供了独立 launch 文件。已经理解 `roslaunch` 后，可以执行：

```bash
roslaunch ros1_hello custom_msg.launch
```

它只启动：

```text
/custom_msg_publisher
/custom_msg_subscriber
```

并通过 `/hello_status` 交换 `ros1_hello/HelloStatus`。

---

## Step 11：理解 ROS 日志的三层位置

代码中：

```cpp
ROS_INFO("%s", msg.data.c_str());
```

最直观的是终端输出。

但 ROS1 日志还会进入 ROS logging 系统。

### `/rosout`

```bash
rostopic echo /rosout
```

可以看到日志消息流。

### `/rosout_agg`

`rosout` Node 会对日志进行汇聚发布，常见 Topic：

```text
/rosout_agg
```

### `~/.ros/log`

查看：

```bash
ls -l ~/.ros/log
```

通常能看到本次运行生成的 log 目录以及 `latest` 指向最近一次运行。

在 Container 中，这里的 `~` 对 `dev` 用户通常是：

```text
/home/dev
```

所以日志位于 Container 用户 home，不是 Host 仓库中的源码文件。

---

## Step 12：认识 XML-RPC 和 TCPROS 的职责差异

ROS1 常见通信机制可以先分成两层：

```text
控制/发现层
    XML-RPC

数据层
    TCPROS
    UDPROS
```

例如 Node 注册 Publisher：

```text
Node → Master
```

常见是 XML-RPC 调用。

Subscriber 查询 Publisher 地址：

```text
Subscriber → Master
```

也属于发现/协商。

而真正 `/chatter` 的 `std_msgs/String` payload 通常通过 TCPROS 在 Node 之间传输。

不要把“ROS1 用 XML-RPC”误解成“所有消息正文都用 XML-RPC 传”。

---

## Step 13：形成一组最小运行时命令

Node：

```bash
rosnode list
rosnode info /hello_node
rosnode ping /hello_node
```

Topic：

```bash
rostopic list
rostopic info /chatter
rostopic echo /chatter
rostopic hz /chatter
```

Parameter：

```bash
rosparam list
rosparam get /hello_node/publish_rate
rosparam set /hello_node/publish_rate 5.0
```

单个 executable：

```bash
rosrun ros1_hello hello_node
```

多 Node 编排：

```bash
roslaunch ros1_hello hello.launch
```

这些命令是后面排查 ROS1 系统时最常用的基本工具。

---

## 本章常见问题

### `rosrun` 提示找不到 package

先：

```bash
source /workspace/ros_ws/devel/setup.bash
rospack find ros1_hello
```

如果 `rospack find` 也失败，回到：

```bash
catkin build ros1_hello
```

确认构建成功。

### Node 启动了，但 `rostopic echo /chatter` 没数据

按顺序看：

```bash
rosnode list
rosnode info /hello_node
rostopic info /chatter
```

不要只盯着 C++ 代码。

如果 `/hello_node` 都不存在，先处理进程启动问题；如果 Node 存在但 `/chatter` 没 Publisher，再处理 Publisher 创建问题。

### Subscriber 没有回调

看：

```bash
rostopic info /chatter
```

确认 `/hello_listener` 出现在 Subscribers。

然后检查代码是否持续执行：

```cpp
ros::spin();
```

或其它 callback queue 处理方式。

### 改参数后频率没立刻改变

当前示例参数只在 Node 启动时读取一次。

需要动态变化时，要在程序中设计重新读取、dynamic_reconfigure 或其它机制；不是 `rosparam set` 自动修改已经构造好的 `ros::Rate` 对象。

---

## 本章练习

1. 把 launch 默认频率改成 `2.0`，重新运行并用 `rostopic hz` 验证；
2. 不运行 `hello_listener`，观察 `rostopic info /chatter` 中 Subscribers 的变化；
3. 不运行 `hello_node`，只启动 listener，观察它是否仍能存活；
4. 用 `rosnode info` 比较 Publisher 和 Subscriber 的 Publications/Subscriptions；
5. 用 `rostopic type` 和 `rosmsg show std_msgs/String` 查看标准消息定义；
6. 运行 `custom_msg_publisher` / `custom_msg_subscriber`，再用 `rosmsg show ros1_hello/HelloStatus` 与 `rostopic echo /hello_status` 对照自定义 `.msg`。

下一章开始进入 VS Code。重点不是“安装几个插件”，而是明确：

```text
哪些动作发生在 Host？
哪些动作发生在 Container？
F12 在哪里解析头文件？
F5 的 GDB 到底运行在哪里？
```

## 参考资料

- [ROS Wiki: Understanding ROS Nodes](https://wiki.ros.org/ROS/Tutorials/UnderstandingNodes)
- [ROS Wiki: Understanding ROS Topics](https://wiki.ros.org/ROS/Tutorials/UnderstandingTopics)
- [ROS Noetic genmsg: User macro reference](https://docs.ros.org/en/noetic/api/genmsg/html/usermacros.html)
- [ROS Wiki: roslaunch](https://wiki.ros.org/roslaunch)
- [ROS Wiki: Parameter Server](https://wiki.ros.org/Parameter%20Server)
