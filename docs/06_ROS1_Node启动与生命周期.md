<meta name="referrer" content="no-referrer" />

# 06：ROS1 Node 启动与停止流程——从共享库初始化到 `ros::start()` 与 shutdown

> 摘要：围绕 ROS1 roscpp 的 Node 生命周期，说明共享库与 main 前静态初始化、ros::init、NodeHandle、ros::start、shutdown 的关系，并在 Docker 中接入 rqt GUI。

[TOC]

这一章只建立一个整体模型：**一个 roscpp Node 从进程启动、ROS 初始化、内部运行设施启动，到 shutdown 和资源释放，依次发生什么。**

实验继续使用：

```text
ros_ws/src/ros1_hello          # 业务 Node
ros_debug_ws                   # ROS1 Noetic 上游源码 overlay
```

先看整条生命周期，后文只展开需要掌握的节点。

![ROS1 Node 启动与停止主流程](images/06-node-lifecycle.png)

需要先抓住四个边界：

1. `main()` 之前可能已经执行共享库中的全局/静态对象构造；
2. `ros::init()` 只把 roscpp 初始化到 **initialized**；
3. 第一个 `NodeHandle` 在 `NodeHandle::construct()` 中触发 `ros::start()`；
4. `ros::start()` 建立真正的运行设施，shutdown 再按相反方向释放这些资源。

---

## 1. `.so` 在哪里，`main()` 前的代码又是谁执行的

### 1.1 当前工程里的 `.so` 在哪里看

`.so` 是构建产物，不是在 `src/` 目录里阅读的源码文件。

当前 Debug overlay 构建出的 ROS 库主要在：

```text
/workspace/ros_debug_ws/devel/lib/
```

例如：

```bash
ls -lh /workspace/ros_debug_ws/devel/lib/libroscpp.so
ls -lh /workspace/ros_debug_ws/devel/lib/librosconsole*.so
ls -lh /workspace/ros_debug_ws/devel/lib/libxmlrpcpp.so
```

官方 Noetic underlay 自带的库则在：

```text
/opt/ros/noetic/lib/
```

例如：

```bash
ls -lh /opt/ros/noetic/lib/libroscpp.so
ls -lh /opt/ros/noetic/lib/librosconsole*.so
```

如果要知道 `hello_node` **实际声明依赖了哪些共享库**：

```bash
readelf -d /workspace/ros_ws/devel/lib/ros1_hello/hello_node \
    | grep NEEDED
```

如果要看动态链接器最终解析到了哪一份文件：

```bash
ldd /workspace/ros_ws/devel/lib/ros1_hello/hello_node
```

这两条命令的区别是：

```text
readelf -d
  -> 看 ELF 里的 DT_NEEDED 依赖声明

ldd
  -> 看当前环境下这些依赖最终解析到哪个实际路径
```

因此在 Debug overlay 环境中，重点确认类似：

```text
libroscpp.so => /workspace/ros_debug_ws/devel/lib/libroscpp.so
librosconsole.so => /workspace/ros_debug_ws/devel/lib/librosconsole.so
```

而不是意外回到：

```text
/opt/ros/noetic/lib/...
```

程序运行起来之后，还可以直接看进程当前映射了哪些共享库：

```bash
pid=$(pgrep -n hello_node)
grep '\.so' /proc/$pid/maps
```

### 1.2 为什么 `hello_node` 会加载这些 `.so`

业务 package 的 CMake 中：

```cmake
target_link_libraries(hello_node
  ${catkin_LIBRARIES}
)
```

`find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)` 会把 roscpp 等依赖放进 `${catkin_LIBRARIES}`。

链接 `hello_node` 时，链接器把需要的共享库记录到 ELF 的动态段中。Linux 创建进程后，ELF interpreter，也就是动态装载器，会处理这些 `DT_NEEDED`：

```text
hello_node ELF
    ↓
动态装载器
    ↓
加载 libroscpp.so / librosconsole.so / 其它依赖
    ↓
重定位
    ↓
执行共享库初始化代码
    ↓
C Runtime
    ↓
main()
```

所以 `main()` 并不是 Linux 进程真正的第一条指令。对本章只需要区分：

| 层次 | 入口 | 作用 |
| --- | --- | --- |
| ELF | `_start` 一类的进程入口 | C Runtime 启动前的真正 ELF 入口 |
| C/C++ | `main()` | 进入用户应用代码 |
| ROS | `ros::init()` | 进入 roscpp 的显式初始化 |

如果想看 ELF 自己记录的入口地址：

```bash
readelf -h /workspace/ros_ws/devel/lib/ros1_hello/hello_node \
    | grep 'Entry point'
```

VS Code 当前 `launch.json` 中的：

```json
"stopAtEntry": false
```

改成：

```json
"stopAtEntry": true
```

即可让 F5 启动后先停在程序入口附近。正常学习 roscpp 生命周期时，从 `main()` 开始阅读已经足够。

---

## 2. 为什么 `rosconsole.cpp` 会先执行，随后又看到 `master.cpp`、`network.cpp`、`param.cpp`

这里要严格区分四种“顺序”：

```text
CMake 源文件列表
编译/链接顺序
main() 前的 C++ 静态初始化顺序
main() 后 ros::init() 的显式函数调用顺序
```

它们不是同一个东西。

### 2.1 `rosconsole.cpp` 为什么能在 `main()` 前执行

`rosconsole` 是独立共享库。源码中存在：

```cpp
class StaticInit
{
public:
  StaticInit()
  {
    ROSCONSOLE_AUTOINIT;
  }
};

StaticInit g_static_init;
```

`g_static_init` 是全局 C++ 对象。

当动态装载器初始化 `librosconsole.so` 时，会执行这个对象的构造函数，于是进入：

```text
ROSCONSOLE_AUTOINIT
    ↓
ros::console::initialize()
```

所以完全可能看到：

```text
ros::console::initialize()
    ↓
main()
    ↓
ros::init()
```

它不是 `roscore` 调用了 `hello_node`，而是当前 `hello_node` 进程自己的共享库初始化。

### 2.2 `master.cpp`、`network.cpp` 等为什么也会在 `main()` 前出现

这些文件都属于 `libroscpp.so`。

例如它们包含全局对象：

```cpp
// master.cpp
std::string g_host;
std::string g_uri;
ros::WallDuration g_retry_timeout;

// network.cpp
std::string g_host;

// param.cpp
M_Param g_params;
boost::recursive_mutex g_params_mutex;
S_string g_subscribed_params;
```

`std::string`、`std::map`、Boost mutex 等都不是简单的纯零初始化对象，它们需要 C++ 构造过程。因此 `libroscpp.so` 初始化时，也会执行各 translation unit 生成的静态初始化函数。

`XMLRPCManager::instance()` 本身采用函数内 static：

```cpp
static XMLRPCManagerPtr xmlrpc_manager = boost::make_shared<XMLRPCManager>();
```

这一个对象只有第一次真正调用 `XMLRPCManager::instance()` 时才构造；但 `xmlrpc_manager.cpp` 里其它具有动态初始化需求的全局/静态对象仍可能产生初始化代码。

### 2.3 “构建顺序”在哪里看

roscpp 的源码成员直接写在：

```text
ros_debug_ws/src/ros_comm/clients/roscpp/CMakeLists.txt
```

其中：

```cmake
add_library(roscpp
  src/libros/master.cpp
  src/libros/network.cpp
  ...
  src/libros/param.cpp
  ...
  src/libros/xmlrpc_manager.cpp
  ...
  src/libros/init.cpp
  ...
)
```

这说明这些 `.cpp` 最终一起组成 `libroscpp.so`。

每个 `.cpp` 先独立编译成 `.o`。并行构建时，**不存在值得依赖的“master.cpp 必须先编译、network.cpp 再编译”规则**。

当前构建目录下真正的链接命令可以看：

```bash
cat /workspace/ros_debug_ws/build/roscpp/CMakeFiles/roscpp.dir/link.txt
```

只看 `src/libros` 对象文件：

```bash
tr ' ' '\n' \
  < /workspace/ros_debug_ws/build/roscpp/CMakeFiles/roscpp.dir/link.txt \
  | grep 'src/libros'
```

每个源文件对应的对象文件则在：

```text
ros_debug_ws/build/roscpp/CMakeFiles/roscpp.dir/src/libros/
```

### 2.4 “main 前实际初始化顺序”在哪里看

最终顺序已经不是 CMake 源码调用链，而是链接器写进 ELF 的初始化表。

可以看：

```bash
readelf -W -S /workspace/ros_debug_ws/devel/lib/libroscpp.so \
    | grep -E '\.init|\.init_array'

readelf -W -x .init_array \
    /workspace/ros_debug_ws/devel/lib/libroscpp.so
```

也可以检查各 `.o` 是否生成了全局初始化函数：

```bash
nm -aC \
  /workspace/ros_debug_ws/build/roscpp/CMakeFiles/roscpp.dir/src/libros/master.cpp.o \
  | grep -E '_GLOBAL__sub_I_|static_initialization'
```

把 `master.cpp.o` 换成 `network.cpp.o`、`param.cpp.o` 等即可。

这里最重要的结论是：

> **不同 `.cpp` translation unit 之间的全局动态初始化顺序不应作为 ROS API 契约依赖。当前二进制里可以观察它，但业务逻辑不要依赖它。**

因此你在 `main()` 前看到的：

```text
rosconsole.cpp
master.cpp
network.cpp
param.cpp
xmlrpc_manager.cpp
...
```

属于“共享库/translation unit 静态初始化”这一层。

它和下面 `ros::init()` 内真正写死的函数调用顺序是两回事。

---

## 3. `ros::init()`：显式初始化顺序

当前应用调用：

```cpp
ros::init(argc, argv, "hello_node");
```

本章不展开重载形式，只看最终进入核心初始化后做什么。

源码：

```text
ros_debug_ws/src/ros_comm/clients/roscpp/src/libros/init.cpp
```

核心顺序非常明确：

```text
注册 atexit callback
创建 global CallbackQueue
保存 init options
ROSCONSOLE_AUTOINIT
忽略 SIGPIPE
check_ipv6_environment()
network::init(remappings)
master::init(remappings)
this_node::init(name, remappings, options)
  └─ names::init(remappings)
file_log::init(remappings)
param::init(remappings)
g_initialized = true
```

这才是本章真正需要记住的 `ros::init()` 调用顺序。

### `network::init()`

主要决定当前 Node 对外使用的 host 和 TCPROS 相关网络配置。

它会考虑：

```text
__hostname
__ip
__tcpros_server_port
ROS_HOSTNAME
ROS_IP
本机 hostname / 网络接口
```

这里只需要知道它在准备“本 Node 怎样被其它 ROS 进程访问”的网络身份。

### `master::init()`

主要确定 Master URI：

```text
__master
    ↓ 没有
ROS_MASTER_URI
    ↓ 没有
http://localhost:11311
```

随后把 URI 拆成 host/port 保存下来。

注意：这里仍只是**准备 Master 地址**，不是在本节展开注册发现过程；注册发现留到 `07`。

### `this_node::init()` 与 `names::init()`

它负责形成当前 Node 的最终名字和 namespace，并在确定 namespace 后初始化 ROS 名称 remapping。

因此 `ros::init()` 完成以后，`this_node::getName()`、namespace、全局 name remapping 等基础状态已经建立。

### `file_log::init()`

确定当前 Node 的日志目录和日志文件路径。

### `param::init()`

建立参数系统当前进程侧需要的初始化状态，例如参数更新回调绑定等。

所以可以把 `ros::init()` 压缩成一句：

```text
解析启动参数
  + 建立 Node 名称环境
  + 准备网络/Master/日志/参数基础状态
  = initialized
```

此时还**没有执行 `ros::start()`**。

---

## 4. 第一个 `NodeHandle`：从 initialized 进入 started

当前代码：

```cpp
ros::NodeHandle nh;
ros::NodeHandle pnh("~");
```

实现：

```text
ros_debug_ws/src/ros_comm/clients/roscpp/src/libros/node_handle.cpp
```

构造函数内部顺序是：

```text
处理 namespace
    ↓
NodeHandle::construct(...)
    ↓
NodeHandle::initRemappings(remappings)
```

注意：**`initRemappings()` 不在 `construct()` 内部。二者是构造函数依次调用的两个阶段。**

### 4.1 `NodeHandle::construct()`

核心职责：

1. 检查是否已经执行 `ros::init()`；
2. 创建 `NodeHandleBackingCollection`；
3. 解析当前 NodeHandle 的 namespace；
4. 加锁保护全局 `g_nh_refcount`；
5. 若这是第一个 NodeHandle 且 Node 尚未 started，则调用 `ros::start()`；
6. 增加全局 NodeHandle 引用计数。

关键判断：

```cpp
if (g_nh_refcount == 0 && !ros::isStarted())
{
    g_node_started_by_nh = true;
    ros::start();
}
```

因此当前程序的启动边界非常清楚：

```text
ros::init()
    ↓
ros::NodeHandle nh
    ↓
NodeHandle::construct()
    ↓
ros::start()
```

后面再创建 `pnh("~")` 时，Node 已经 started，不会重复 start。

### 4.2 `NodeHandle::initRemappings()`

这个函数只处理**当前 NodeHandle 自己携带的局部 remapping**。

源码逻辑是把每一对映射保存两份：

```text
原始 from/to
  -> unresolved_remappings_

经过当前 NodeHandle namespace 解析后的 from/to
  -> remappings_
```

例如：

```cpp
ros::M_string remaps;
remaps["chatter"] = "/driver/raw";
ros::NodeHandle nh("", remaps);
```

之后这个 `nh` 在解析名称时，先检查自己的局部 remapping；没有命中才继续使用 `ros::names` 的全局 remapping。

当前：

```cpp
ros::NodeHandle nh;
ros::NodeHandle pnh("~");
```

都没有显式传入 remappings，所以 `initRemappings()` 仍会执行，但传入 map 为空。

`pnh("~")` 中的 `~` 会先解析到当前 Node 的 private namespace，因此：

```cpp
pnh.param("publish_rate", ...);
```

位于 `/hello_node` 的 private namespace 下。

---

## 5. `ros::start()`：真正启动运行设施

`ros::start()` 仍位于：

```text
ros_comm/clients/roscpp/src/libros/init.cpp
```

先建立全局运行状态：

```text
g_shutdown_requested = false
g_shutting_down      = false
g_started            = true
g_ok                 = true
```

然后建立 shutdown 入口：

```text
PollManager 注册 checkForShutdown listener
XMLRPCManager 绑定 shutdown callback
```

再初始化内部 timer manager，并依次启动：

```text
TopicManager::start()
ServiceManager::start()
ConnectionManager::start()
PollManager::start()
XMLRPCManager::start()
```

本章对这些 Manager 只建立职责印象：

| 组件 | 这里先理解为 |
| --- | --- |
| `TopicManager` | Topic 发布/订阅状态和相关处理入口 |
| `ServiceManager` | Service server/client 状态 |
| `ConnectionManager` | TCPROS/UDPROS 连接与监听 |
| `PollManager` | 网络 fd 事件轮询和 poll listener |
| `XMLRPCManager` | 当前 Node 自己的 XML-RPC server/client 管理 |

之后 `ros::start()` 继续完成：

```text
安装 SIGINT handler
ros::Time::init()
注册 rosout appender
创建 ~get_loggers / ~set_logger_level 等内部 service
按 /use_sim_time 决定是否订阅 /clock
启动 internal callback queue thread
注册 atexit(ros::shutdown)
启用 global CallbackQueue
```

所以当前阶段只需要形成：

```text
ros::init()
  -> 准备 Node 的基础状态

第一个 NodeHandle
  -> ros::start()

ros::start()
  -> 把 ROS 通信、轮询、XML-RPC、内部 callback 等运行设施真正启动起来
```

后续再分别深入 Master/XML-RPC、Topic/TCPROS、CallbackQueue，不需要现在把 `start()` 中每个 Manager 全部钻完。

---

## 6. rosconsole：三个 backend 到底用哪一个

源码树同时存在：

```text
rosconsole_log4cxx.cpp
rosconsole_glog.cpp
rosconsole_print.cpp
```

它们是三个**构建期可选 backend**，不是运行时三个一起执行。

`rosconsole/CMakeLists.txt` 定义：

```cmake
set(ROSCONSOLE_BACKEND "" CACHE STRING
    "Type of rosconsole backend, one of 'log4cxx', 'glog', 'print'")
```

没有显式指定时，Noetic 的探测顺序是：

```text
log4cxx
  ↓ 找不到
 glog
  ↓ 找不到
 print
```

实际选择结果直接看当前 Debug overlay：

```bash
grep '^ROSCONSOLE_BACKEND:' \
    /workspace/ros_debug_ws/build/rosconsole/CMakeCache.txt
```

也可以看当前产物和依赖：

```bash
ls -lh /workspace/ros_debug_ws/devel/lib/librosconsole*.so

ldd /workspace/ros_debug_ws/devel/lib/librosconsole.so \
    | grep -E 'log4cxx|glog|rosconsole_'
```

如果结果是：

```text
ROSCONSOLE_BACKEND:STRING=log4cxx
```

那么本次构建真正走的是 `rosconsole_log4cxx.cpp`。

只有明确构建成 glog backend 时，`rosconsole_glog.cpp` 中的：

```cpp
google::LogMessage(...)
```

才属于当前运行路径。

### `ROSCONSOLE_FORMAT` 默认格式在哪里

Noetic 的 `rosconsole.cpp` 直接定义：

```cpp
const char* g_format_string = "[${severity}] [${time}]: ${message}";
```

`Formatter::init()` 再解析 `${...}` token。

常用 token：

```text
severity
message
time
walltime
thread
logger
file
line
function
node
```

主要资料：

- ROS Wiki rosconsole：`https://wiki.ros.org/rosconsole`
- Noetic rosconsole 源码：`https://github.com/ros/rosconsole/blob/noetic-devel/src/rosconsole/rosconsole.cpp`
- rosconsole backend 选择：`https://github.com/ros/rosconsole/blob/noetic-devel/CMakeLists.txt`

---

## 7. shutdown 怎样进入，`ros::shutdown()` 又做什么

### Ctrl+C / SIGINT

`ros::start()` 默认安装：

```text
SIGINT
  -> basicSigintHandler()
  -> requestShutdown()
  -> g_shutdown_requested = true
```

`PollManager` 已经注册 `checkForShutdown()` listener，随后进入：

```text
checkForShutdown()
  -> ros::shutdown()
```

### `rosnode kill /hello_node`

另一条入口是：

```bash
rosnode kill /hello_node
```

对应：

```text
XML-RPC shutdown request
  -> shutdownCallback()
  -> requestShutdown()
  -> checkForShutdown()
  -> ros::shutdown()
```

XML-RPC 请求怎样找到 Node 留到 `07`，这一章只关注它最终怎样进入 shutdown。

### 显式调用

应用也可以直接：

```cpp
ros::shutdown();
```

这条路径不需要经过 `requestShutdown()`。

### `ros::shutdown()` 主干

`init.cpp` 中的清理顺序可以压缩为：

```text
设置 g_shutting_down
    ↓
关闭 rosconsole
    ↓
disable + clear global CallbackQueue
    ↓
join internal callback queue thread
    ↓
释放 rosout appender
    ↓
TopicManager::shutdown()
ServiceManager::shutdown()
PollManager::shutdown()
ConnectionManager::shutdown()
XMLRPCManager::shutdown()
    ↓
g_started = false
g_ok = false
    ↓
ros::Time::shutdown()
```

`g_ok = false` 后：

```cpp
while (ros::ok())
```

会在下一次检查时退出。

### 最后一个 `NodeHandle` 析构

`NodeHandle` 用：

```text
g_nh_refcount
```

维护全局引用计数。

如果引用计数降到 0，并且这个 Node 原本是由第一个 NodeHandle 自动启动的，则 `NodeHandle::destruct()` 也会调用 `ros::shutdown()`。

因此生命周期首尾正好对应：

```text
第一个 NodeHandle
  -> ros::start()

最后一个 NodeHandle 析构
  -> ros::shutdown()
```

---

## 8. 把 `rqt_graph` / `rqt_console` 正式加入 Docker 环境

这两个工具都是 GUI：

- `rqt_graph`：Qt GUI，用于查看 ROS computation graph；
- `rqt_console`：Qt GUI，用于显示和过滤 ROS log messages。

因此它们不能只靠一个纯字符终端完成可视化。**需要有可用的图形显示服务。**

当前环境是 Ubuntu 桌面 + Docker，最简单的方式是让 Container 连接 Host 的 X11/XWayland display：

```text
rqt Qt GUI in Container
    ↓ DISPLAY
/tmp/.X11-unix
    ↓
Ubuntu Host Xorg / XWayland
    ↓
桌面窗口
```

如果 Host 的 GNOME 会话使用 Wayland，也通常可以通过 XWayland 兼容层运行这类 Qt/X11 应用。本工程先使用 X11 socket 转发，不额外引入原生 Wayland socket。

### 8.1 Dockerfile

已经把：

```text
ros-noetic-rqt-graph
ros-noetic-rqt-console
```

加入原有 `apt-get install`：

```dockerfile
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ... \
        ros-noetic-rqt-graph \
        ros-noetic-rqt-console \
        ... \
    && rm -rf /var/lib/apt/lists/*
```

只安装这两个具体 package 即可，它们会通过 Debian/ROS package dependency 拉取 `rqt_gui`、`python_qt_binding` 等运行依赖。

### 8.2 compose.yaml

给 `ros1-dev` 增加：

```yaml
environment:
  DISPLAY: ${DISPLAY:-:0}
  QT_X11_NO_MITSHM: "1"
```

并把 Host 的 X11 Unix socket bind mount 到 Container：

```yaml
volumes:
  - type: bind
    source: /tmp/.X11-unix
    target: /tmp/.X11-unix
```

其中：

```text
DISPLAY
  -> 告诉 Qt 应用连接哪个 X display

/tmp/.X11-unix
  -> 本地 X server 的 Unix domain socket

QT_X11_NO_MITSHM=1
  -> 容器场景下不使用 X11 MIT-SHM，减少共享内存相关兼容问题
```

`DISPLAY: ${DISPLAY:-:0}` 表示：

- Compose 启动环境已经有 `DISPLAY`：直接继承；
- 没有：先默认使用常见的 `:0`。

如果 Ubuntu 桌面实际不是 `:0`，以桌面终端执行：

```bash
echo "$DISPLAY"
```

看到的值为准。

### 8.3 Host 授权 Container 连接 X server

在 **Ubuntu 图形桌面里的终端**执行：

```bash
xhost +si:localuser:$(whoami)
```

当前 Dockerfile 会让 Container 用户 UID/GID 与 Host 对齐，因此这个 local-user 授权方式适合当前开发环境。

不要使用：

```bash
xhost +
```

它会直接关闭 X server 的访问控制，范围过大。

如果 Host 没有 `xhost`：

```bash
sudo apt update
sudo apt install x11-xserver-utils
```

### 8.4 重建并启动

修改 Dockerfile 后需要重新 build 镜像：

```bash
cd /home/wdfk/share/ros1-docker

docker compose build ros1-dev
docker compose up -d --force-recreate ros1-dev
```

确认 Container 能看到 display：

```bash
docker compose exec ros1-dev bash

echo "$DISPLAY"
ls -l /tmp/.X11-unix
```

然后先启动 `roscore` 和实验 Node，再执行：

```bash
rqt_graph
```

或：

```bash
rqt_console
```

窗口应该出现在 Ubuntu VM 的桌面环境中，而不是嵌在 VS Code Terminal 里面。

`rqt_graph` 当前主要用来看：

```text
/hello_node
    -> /chatter
```

以及后续 listener 加入后的发布/订阅连接关系。

`rqt_console` 用来看和过滤 ROS 日志，可以按 Node、Severity、Message 等维度观察。

![alt text](images/06-rqt.png)

### 8.5 相关资料

- ROS Wiki `rqt_graph`：`https://wiki.ros.org/rqt_graph`
- ROS Wiki `rqt_console`：`https://wiki.ros.org/rqt_console`
- ROS Wiki `rqt`：`https://wiki.ros.org/rqt`
- `rqt_graph` Noetic 源码：`https://github.com/ros-visualization/rqt_graph/tree/noetic-devel`
- `rqt_console` Noetic 源码：`https://github.com/ros-visualization/rqt_console/tree/noetic-devel`
- X.Org `xhost`：`https://www.x.org/releases/X11R7.0/doc/html/xhost.1.html`
- Docker bind mounts：`https://docs.docker.com/engine/storage/bind-mounts/`

---

## 9. 本章源码阅读主线

按下面的符号顺序阅读即可：

```text
hello_node.cpp
  -> ros::init()
  -> network::init()
  -> master::init()
  -> this_node::init() / names::init()
  -> file_log::init()
  -> param::init()
  -> ros::NodeHandle::NodeHandle()
  -> NodeHandle::construct()
  -> ros::start()
  -> NodeHandle::initRemappings()
  -> shutdownCallback() / basicSigintHandler()
  -> requestShutdown()
  -> checkForShutdown()
  -> ros::shutdown()
  -> NodeHandle::destruct()
```

这里有一个时序细节：构造函数中 `construct()` 先执行，而第一个 `construct()` 会在内部调用 `ros::start()`；返回以后才执行 `initRemappings()`。因此当前源码真实顺序是：

```text
NodeHandle constructor
  -> construct()
      -> first NodeHandle ? ros::start()
  -> initRemappings()
```

本章真正需要建立的是：

```text
进程装载与静态初始化
    ↓
main()
    ↓
ros::init()             initialized
    ↓
第一个 NodeHandle
    ↓
ros::start()            started
    ↓
应用运行
    ↓
shutdown request
    ↓
ros::shutdown()
    ↓
NodeHandle / 进程退出
```

能把这条链解释清楚后，再进入 `07` 的 Master/XML-RPC 注册发现机制。

---

## 参考源码与文档

- roscpp `CMakeLists.txt`：`https://github.com/ros/ros_comm/blob/noetic-devel/clients/roscpp/CMakeLists.txt`
- roscpp `init.cpp`：`https://github.com/ros/ros_comm/blob/noetic-devel/clients/roscpp/src/libros/init.cpp`
- roscpp `node_handle.cpp`：`https://github.com/ros/ros_comm/blob/noetic-devel/clients/roscpp/src/libros/node_handle.cpp`
- roscpp `node_handle.h`：`https://github.com/ros/ros_comm/blob/noetic-devel/clients/roscpp/include/ros/node_handle.h`
- roscpp `master.cpp`：`https://github.com/ros/ros_comm/blob/noetic-devel/clients/roscpp/src/libros/master.cpp`
- roscpp `network.cpp`：`https://github.com/ros/ros_comm/blob/noetic-devel/clients/roscpp/src/libros/network.cpp`
- roscpp `param.cpp`：`https://github.com/ros/ros_comm/blob/noetic-devel/clients/roscpp/src/libros/param.cpp`
- roscpp `xmlrpc_manager.cpp`：`https://github.com/ros/ros_comm/blob/noetic-devel/clients/roscpp/src/libros/xmlrpc_manager.cpp`
- rosconsole `CMakeLists.txt`：`https://github.com/ros/rosconsole/blob/noetic-devel/CMakeLists.txt`
- rosconsole `rosconsole.cpp`：`https://github.com/ros/rosconsole/blob/noetic-devel/src/rosconsole/rosconsole.cpp`
- ROS1 Names：`https://wiki.ros.org/Names`
- ROS1 rosconsole：`https://wiki.ros.org/rosconsole`
- ROS1 rqt_graph：`https://wiki.ros.org/rqt_graph`
- ROS1 rqt_console：`https://wiki.ros.org/rqt_console`
