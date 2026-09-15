<meta name="referrer" content="no-referrer" />

# 02：创建 catkin Workspace、Package 与第一个 ROS1 C++ Node

> 摘要：这一章从空的 catkin workspace 思维出发，逐步理解 `src/`、`package.xml`、`CMakeLists.txt`、`catkin init`、`catkin config` 和 `catkin build`。然后从最小 `main()` 开始，一步步写出 `hello_node` Publisher，并确认它最终被编译成可执行 ELF。

[TOC]

## 本章目标

完成本章后，你应该能够：

1. 解释 workspace、package、Node、CMake target 四个概念的关系；
2. 知道 `package.xml` 和 `CMakeLists.txt` 分别负责什么；
3. 会初始化 catkin_tools workspace；
4. 会创建 ROS1 package，并声明 `roscpp`、`std_msgs` 依赖；
5. 能读懂 `ros::init()`、`NodeHandle`、Publisher、`ros::Rate`；
6. 会把 `.cpp` 文件加入 CMake，生成可执行程序；
7. 会用 `catkin build ros1_hello` 构建单个 package；
8. 知道 `build/`、`devel/`、`logs/` 是构建产物，不是源码。

## 为什么先理解构建链，再写更多 ROS API

对于有 MCU/CMake 经验的人，ROS1 最容易混淆的地方不是 C++，而是“谁在调用谁”。

先把构建链压缩成一张图：

```text
package.xml
    → package 身份与 ROS 依赖

CMakeLists.txt
    → C++ target 与链接规则

catkin
    → ROS1 package/CMake 约定

catkin_tools
    → workspace 级构建编排

CMake
    → 生成底层 build system

GCC/G++
    → 编译、链接出 ELF
```

所以：

```bash
catkin build ros1_hello
```

不是“catkin 自己就是编译器”。它是在 workspace 层组织 package，然后调用 CMake 和底层编译工具。

---

## Step 1：进入业务 workspace

进入 Container：

```bash
docker compose exec ros1-dev bash
```

然后：

```bash
cd /workspace/ros_ws
pwd
```

应该输出：

```text
/workspace/ros_ws
```

当前工程已经提供：

```text
ros_ws/
└── src/
    └── ros1_hello/
```

如果你是在一个完全空的 workspace 中练习，只需要先建立：

```bash
mkdir -p /workspace/ros_ws/src
```

catkin workspace 的源码根目录通常就是：

```text
<workspace>/src
```

注意区分两个 `src`：

```text
/workspace/ros_ws/src/
    → workspace 的 source space

/workspace/ros_ws/src/ros1_hello/src/
    → ros1_hello package 自己的 C++ 源码目录
```

这是 ROS 初学阶段非常重要的路径概念。

---

## Step 2：先 source Noetic underlay

执行：

```bash
source /opt/ros/noetic/setup.bash
```

然后：

```bash
echo "$ROS_DISTRO"
echo "$CMAKE_PREFIX_PATH"
```

`ROS_DISTRO` 应为：

```text
noetic
```

为什么初始化 workspace 前先 source？

因为自己的 workspace 不是凭空存在的。它要建立在已经安装好的 Noetic 之上：

```text
/opt/ros/noetic
    ↓ underlay
/workspace/ros_ws
    ↓ overlay
自己的 package
```

后续 CMake 查找：

```cmake
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
```

就需要知道 Noetic 的安装前缀在哪里。

---

## Step 3：初始化 catkin_tools workspace

在 `/workspace/ros_ws`：

```bash
catkin init
```

检查：

```bash
catkin config
```

这一步主要创建：

```text
ros_ws/.catkin_tools/
```

它保存 catkin_tools 自己的 workspace/profile 配置。

`catkin init` 不会：

- 编译你的 C++；
- 自动生成 Node；
- 自动启动 `roscore`；
- 创建 ROS 网络通信。

它只是告诉 catkin_tools：

```text
这里是一套由我管理的 workspace
```

---

## Step 4：配置 Debug、merged devel 和 compile_commands

执行：

```bash
catkin config \
    --merge-devel \
    --extend /opt/ros/noetic \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

再看：

```bash
catkin config
```

重点确认：

```text
Extending:                    /opt/ros/noetic
Devel Space Layout:          merged
Additional CMake Args:       ... Debug ...
```

### `--extend /opt/ros/noetic`

明确告诉当前 workspace：

```text
把 Noetic 安装空间当作 underlay
```

它不会下载 `roscpp` 源码，只是让当前 workspace 能使用已经安装好的 Noetic package。

### `-DCMAKE_BUILD_TYPE=Debug`

让自己的 Node 带调试信息，后面 F5/GDB 才能看到更完整的源码和变量。

### `-DCMAKE_EXPORT_COMPILE_COMMANDS=ON`

让 CMake 导出编译命令数据库。每个 package 的 build directory 中通常会得到：

```text
compile_commands.json
```

它记录真实编译命令、宏和 `-I` 路径，对分析复杂 C++ 项目很有价值。

### 为什么使用 merged devel

本工程同时在：

```text
Remote-SSH Host
```

和：

```text
Container
```

查看同一份 workspace。

merged devel 会把生成头文件直接放在：

```text
devel/include/...
```

更适合 Host 直接读取，也减少绝对符号链接指向 Container `/workspace/...` 后在 Host 断开的情况。

---

## Step 5：如果从零创建 package，先认识 `catkin_create_pkg`

当前仓库已经有完整的 `ros1_hello`，不需要重复创建。这里先理解从零时会做什么。

在一个空的 `ros_ws/src` 中可以执行：

```bash
cd /workspace/ros_ws/src
catkin_create_pkg ros1_hello roscpp std_msgs
```

命令可以拆成：

```text
catkin_create_pkg
    ros1_hello
    roscpp
    std_msgs
```

含义：

```text
package 名：ros1_hello
依赖：       roscpp
            std_msgs
```

生成后最核心的两个文件是：

```text
ros1_hello/
├── package.xml
└── CMakeLists.txt
```

接下来自己再创建：

```bash
mkdir -p src launch
```

本仓库中最终结构为：

```text
ros1_hello/
├── package.xml
├── CMakeLists.txt
├── README.md
├── src/
│   ├── hello_node.cpp
│   └── hello_listener.cpp
└── launch/
    └── hello.launch
```

---

## Step 6：先读 `package.xml`

打开：

```text
ros_ws/src/ros1_hello/package.xml
```

核心内容：

```xml
<package format="2">
  <name>ros1_hello</name>
  <version>0.0.1</version>
  <description>Beginner ROS1 C++ publisher/subscriber example for Docker learning.</description>
  <maintainer email="dev@example.com">dev</maintainer>
  <license>BSD</license>

  <buildtool_depend>catkin</buildtool_depend>

  <depend>roscpp</depend>
  <depend>std_msgs</depend>
</package>
```

可以把它理解成 package 的“身份和依赖清单”。

### `<name>`

```xml
<name>ros1_hello</name>
```

ROS package 名。

后面：

```bash
rosrun ros1_hello hello_node
```

第一个参数就是它。

### `<buildtool_depend>catkin</buildtool_depend>`

说明这个 package 使用 catkin 作为构建系统。

### `<depend>roscpp</depend>`

C++ Node 需要 ROS1 C++ Client Library。

### `<depend>std_msgs</depend>`

因为代码要使用：

```cpp
std_msgs::String
```

---

## Step 7：从最小 C++ 程序开始写 `hello_node`

打开：

```text
ros_ws/src/ros1_hello/src/hello_node.cpp
```

为了理解它，不要一开始把整份代码当成一个黑盒。按下面顺序拆开。

### 7.1 引入 ROS C++ API

```cpp
#include <ros/ros.h>
```

这个 header 提供常用 roscpp API，例如：

```text
ros::init
ros::NodeHandle
ros::Publisher
ros::Rate
ros::ok
ros::spinOnce
ROS_INFO
```

再引入消息类型：

```cpp
#include <std_msgs/String.h>
```

它提供：

```cpp
std_msgs::String
```

### 7.2 正常的 C++ 入口

```cpp
int main(int argc, char** argv)
{
}
```

ROS Node 最终仍然是一个普通 Linux 可执行程序，所以它有普通 C/C++ `main()`。

### 7.3 初始化 roscpp

```cpp
ros::init(argc, argv, "hello_node");
```

可以先理解为：

```text
初始化 roscpp 进程级状态
解析 ROS remapping 参数
设置默认 Node 名 hello_node
```

后面第 05 篇会继续进入 `ros::init()` 实现源码。

### 7.4 创建 `NodeHandle`

```cpp
ros::NodeHandle nh;
```

很多 ROS1 C++ 通信对象通过 `NodeHandle` 创建，例如：

```text
Publisher
Subscriber
ServiceServer
ServiceClient
Timer
Parameter API
```

这个例子还有：

```cpp
ros::NodeHandle pnh("~");
```

`"~"` 表示当前 Node 的 private namespace。第 03 篇会用它读取 `~publish_rate`。

### 7.5 创建 Publisher

```cpp
ros::Publisher publisher =
    nh.advertise<std_msgs::String>("chatter", 10);
```

先按参数读：

```text
消息类型：std_msgs::String
Topic：   chatter
队列：    10
```

默认命名空间下它最终表现为：

```text
/chatter
```

### 7.6 创建循环频率

```cpp
ros::Rate rate(publish_rate);
```

如果：

```text
publish_rate = 1.0
```

那么循环目标大约是：

```text
1 Hz
```

也就是约 1 秒一次。

### 7.7 构造并发布消息

```cpp
std_msgs::String msg;
std::ostringstream stream;
stream << "hello ros1 from docker: " << count++;
msg.data = stream.str();

ROS_INFO("%s", msg.data.c_str());
publisher.publish(msg);
```

这里做两件独立的事：

```text
ROS_INFO
    → ROS 日志系统

publisher.publish(msg)
    → /chatter Topic 数据
```

日志和 Topic 不是同一个通道。

### 7.8 保持 Node 运行

```cpp
while (ros::ok())
{
    ...
    ros::spinOnce();
    rate.sleep();
}
```

`ros::ok()` 在正常运行时为 true；Ctrl+C、shutdown 等情况会让循环退出。

`rate.sleep()` 用于维持目标循环频率。

`ros::spinOnce()` 处理当前 callback queue 中等待的 callback。这个 Publisher 当前没有订阅回调，但保留这句可以帮助理解 callback queue 模型。

---

## Step 8：对照完整 `hello_node.cpp`

最终代码：

```cpp
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <sstream>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hello_node");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    double publish_rate = 1.0;
    pnh.param("publish_rate", publish_rate, 1.0);

    ros::Publisher publisher =
        nh.advertise<std_msgs::String>("chatter", 10);

    ros::Rate rate(publish_rate);
    int count = 0;

    while (ros::ok())
    {
        std_msgs::String msg;
        std::ostringstream stream;
        stream << "hello ros1 from docker: " << count++;
        msg.data = stream.str();

        ROS_INFO("%s", msg.data.c_str());
        publisher.publish(msg);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
```

现在先不研究 Subscriber 和 launch。先让这个单独的 Node 被编译出来。

---

## Step 9：在 `CMakeLists.txt` 中声明 executable

打开：

```text
ros_ws/src/ros1_hello/CMakeLists.txt
```

首先：

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(ros1_hello)
```

这和普通 CMake 项目一样：

```text
最低 CMake 版本
项目名
```

找到 ROS 依赖：

```cmake
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)
```

把 catkin 导出的 include path 加入当前 CMake：

```cmake
include_directories(
  ${catkin_INCLUDE_DIRS}
)
```

创建 executable：

```cmake
add_executable(hello_node
  src/hello_node.cpp
)
```

这里的关系是：

```text
src/hello_node.cpp
       ↓ compile/link
hello_node
```

最后链接 ROS 库：

```cmake
target_link_libraries(hello_node
  ${catkin_LIBRARIES}
)
```

如果少了它，即使 header 能 include，链接阶段也可能找不到 roscpp 的实现符号。

---

## Step 10：第一次构建 package

回到 workspace 根：

```bash
cd /workspace/ros_ws
```

执行：

```bash
catkin build ros1_hello
```

为什么带 package 名？

因为 workspace 以后会有很多 package：

```text
ros_ws/src/
├── ros1_hello/
├── driver_a/
├── motor_control/
└── diagnostics/
```

只写：

```bash
catkin build
```

会构建 workspace 中需要构建的所有 package。

而：

```bash
catkin build ros1_hello
```

更适合初学阶段，把注意力集中在当前 package。

### 构建成功检查

```bash
ls -l /workspace/ros_ws/devel/lib/ros1_hello/hello_node
```

再用：

```bash
file /workspace/ros_ws/devel/lib/ros1_hello/hello_node
```

你会看到它是 Linux ELF executable。

这时要建立概念：

```text
ROS Node
```

在进程层面仍然是：

```text
Linux executable / process
```

ROS 提供的是通信、命名、发现、参数、日志等框架能力。

---

## Step 11：理解 workspace 构建目录

构建后通常出现：

```text
ros_ws/
├── .catkin_tools/
├── build/
├── devel/
├── logs/
└── src/
```

### `src/`

你的源码。最重要。

### `build/`

CMake/Make/Ninja 等构建过程的中间文件。

### `devel/`

开发空间，里面有：

```text
setup.bash
lib/
include/
...
```

Node executable 就在：

```text
devel/lib/ros1_hello/
```

### `logs/`

catkin_tools 构建日志。

### `.catkin_tools/`

catkin_tools profile、config、build metadata。

这些都可以重新生成，所以 `.gitignore` 不提交它们。

---

## Step 12：为什么构建后要 source `devel/setup.bash`

新开一个 shell 时，只 source：

```bash
source /opt/ros/noetic/setup.bash
```

ROS 只知道系统安装的 Noetic package。

自己的 `ros1_hello` 刚刚才被构建出来，所以要让 shell 认识它：

```bash
source /workspace/ros_ws/devel/setup.bash
```

然后：

```bash
rospack find ros1_hello
```

应输出 package 路径。

本工程的交互式 Container shell 会通过：

```text
/workspace/.devcontainer/ros_env.bash
```

自动按存在性 source overlay，但学习时仍然要理解手工 source 的意义。

---

## Step 13：加入 Subscriber executable

Publisher 能编译后，再看：

```text
src/hello_listener.cpp
```

核心是：

```cpp
static void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("received: %s", msg->data.c_str());
}
```

订阅：

```cpp
ros::Subscriber subscriber =
    nh.subscribe("chatter", 10, chatterCallback);
```

然后：

```cpp
ros::spin();
```

`ros::spin()` 会让当前线程持续处理 callback queue，直到 Node shutdown。

CMake 再增加：

```cmake
add_executable(hello_listener
  src/hello_listener.cpp
)

target_link_libraries(hello_listener
  ${catkin_LIBRARIES}
)
```

重新构建：

```bash
cd /workspace/ros_ws
catkin build ros1_hello
```

检查：

```bash
ls -l devel/lib/ros1_hello/
```

应有：

```text
hello_node
hello_listener
```

---

## 本章常见问题

### `catkin build` 提示 workspace 不存在或路径异常

确认：

```bash
pwd
catkin config
```

你应该在：

```text
/workspace/ros_ws
```

而不是 package 的 `src/ros1_hello` 目录中执行 workspace 构建。

### `find_package(catkin)` 或 `roscpp` 找不到

先检查：

```bash
echo "$CMAKE_PREFIX_PATH"
```

再：

```bash
source /opt/ros/noetic/setup.bash
catkin config --extend /opt/ros/noetic
```

### 改了代码但 executable 没变化

重新执行：

```bash
catkin build ros1_hello
```

不要把“保存 `.cpp`”和“已经重新编译”混为一谈。

### `ros1_hello` 找不到

构建后：

```bash
source /workspace/ros_ws/devel/setup.bash
rospack find ros1_hello
```

如果还是找不到，再看 `catkin build` 是否真的成功。

---

## 本章练习

1. 把 `publish_rate` 默认值从 `1.0` 改为 `2.0`，重新 build；
2. 用 `file` 判断 `hello_node` 的文件类型；
3. 用 `ldd` 看 `hello_node` 依赖哪些动态库；
4. 删除 `build/`、`devel/`、`logs/` 后重新构建，观察哪些文件是可再生成的；
5. 解释 `package.xml` 和 `CMakeLists.txt` 为什么不能互相替代。

下一章不再停留在“编译成功”。我们会启动 ROS Master，运行 Publisher/Subscriber，观察 Topic、Parameter、日志与 `roslaunch`。

## 参考资料

- [catkin_tools documentation](https://catkin-tools.readthedocs.io/)
- [ROS Wiki: Creating a ROS Package](https://wiki.ros.org/ROS/Tutorials/CreatingPackage)
- [ROS Wiki: Building Packages](https://wiki.ros.org/ROS/Tutorials/BuildingPackages)
- [CMake documentation](https://cmake.org/documentation/)
