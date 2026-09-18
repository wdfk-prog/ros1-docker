<meta name="referrer" content="no-referrer" />

# ROS教程11.5：roscore源码阅读——从启动脚本到 Master 注册表与控制面

> 摘要：沿 Noetic 源码解释 roscore、roslaunch、rosmaster、Master 注册表与 Python/C++ 边界，并用 bringup、READY Service 和 Docker Compose 落地整机启动。

![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/123cc8430e2f4e72828fb6822b61ffb7.png)


@[toc]
阶段 06～11 已经从 roscpp、Topic、TCPROS、CallbackQueue、Service 与 Action 的客户端/服务端路径建立了 ROS1 通信模型。本章作为阶段 A 的收束篇，换到 **Master 服务端视角**，把前面多次出现但尚未完整拆开的 `roscore`、`roslaunch`、`rosmaster` 和 Master 注册表串起来。

本章不重复第 07 章已经完成的 roscpp `registerPublisher()` / `registerSubscriber()` 客户端调用，也不重复第 08 章 TCPROS socket 数据面，更不重新展开第 11 章 actionlib 状态机。关注点只有一条：

```text
终端执行 roscore
    -> 到底先执行哪个脚本
    -> roslaunch 为什么参与其中
    -> rosmaster 子进程怎样被创建
    -> Master 内部有哪些核心对象
    -> Node、Topic、Service、Parameter 到达 Master 后怎样进入注册表
    -> Action 为什么在 Master 中没有独立的 Action 注册表
    -> Master 什么时候回调 Node，什么时候退出后续数据路径
```

源码基线固定为 ROS1 Noetic 的 `noetic-devel` 分支。当前 `ros1-docker` 环境使用 `ros:noetic-ros-base-focal`，因此源码路径、Python 版本和运行时模型都以 Noetic 为准。

---

## 1. 先确定一个关键事实：`roscore` 不是 ROS Master 本体

终端执行：

```bash
roscore
```

容易形成下面这种直觉：

```text
roscore == ROS Master
```

从源码看并不是这样。

`roscore` 位于 `ros_comm/tools/roslaunch/scripts/roscore`，它是 `roslaunch` package 安装出来的 Python 入口脚本。脚本解析少量 core 相关参数后，核心动作是进入：

```text
roslaunch.main(... --core ...)
```

真正提供 Master XML-RPC API 的进程稍后才由 roslaunch 创建，它运行的是：

```text
rosmaster --core -p <port> -w <num_workers>
```

所以从进程职责上应该先建立下面的区分：

| 对象 | 本质 | 主要职责 |
| --- | --- | --- |
| `roscore` | Python 启动脚本 | 以 `--core` 模式进入 roslaunch |
| `roslaunch` parent | Python 父进程 | 加载 core 配置、启动/监控 Master 与 core Node |
| `rosmaster` | Python 子进程 | 提供 Master API 与 Parameter Server |
| `/rosout` | 普通 ROS Node | 接收 `/rosout` 日志 Topic |

这也是为什么系统里可以同时看到 `roscore`/`rosmaster`/`rosout` 相关进程，而不是只有一个“万能 roscore 进程”。

---

## 2. `roscore` 是怎么“编译”出来的：这里和 `roscpp` 完全不同

第 05 章为了 F11 进入 `ros::init()`，需要把 `roscpp` 编译成带 Debug 信息的 `libroscpp.so`。`roscore`/`rosmaster` 不能照搬这套理解。

Noetic 中：

```text
roslaunch/CMakeLists.txt
    -> catkin_python_setup()
    -> roslaunch/setup.py
    -> 安装 roslaunch Python package + scripts/roscore 等脚本

rosmaster/CMakeLists.txt
    -> catkin_python_setup()
    -> rosmaster/setup.py
    -> 安装 rosmaster Python package + scripts/rosmaster
```

因此执行：

```bash
catkin build rosgraph rosmaster roslaunch
```

对这几个 package 而言，核心不是“把 Python 编译成一个像 `libroscpp.so` 那样的 ELF”，而是让 catkin 完成 package 配置、Python package/script 的 devel/install 布置以及依赖关系处理。

这会直接改变后面的调试方式：

```text
roscpp
    -> C++
    -> GDB / DWARF / shared library

roslaunch / rosmaster
    -> Python
    -> 直接读 .py
    -> pdb / Python debugger / 日志
```

`rosout` 则是另一个独立 package，属于真正被 roslaunch 启动的 core Node，不应和 Python Master 实现混为一体。

### 2.1 为什么标准 Master 是 Python，而工程 Node 又经常是 C++

看到 `roscore`、`roslaunch`、`rosmaster` 都是 Python 后，容易继续追问：ROS1 是否还有另一套“真正的 C++ Master”？

Noetic 的标准实现不是这种结构。在 `ros_comm` 中，标准 `rosmaster` 就是 Python 实现；C++ 侧对应的是 `roscpp`
客户端库，而不是另一套并行维护的 C++ Master executable。两侧通过 ROS Master XML-RPC API 对接。

因此更准确的边界是：

```text
Python 控制面
    roscore / roslaunch / rosmaster
        -> 进程组织
        -> 名称注册与查询
        -> Parameter Server
        -> XML-RPC 控制请求

C++ Node 侧
    roscpp
        -> ros::init() / NodeHandle
        -> TopicManager / ServiceManager
        -> XmlRpcManager / master::execute()
        -> TCPROS / ROSRPC
        -> CallbackQueue / Spinner
```

例如一个 C++ Publisher 执行 `advertise()` 时，客户端侧会进入 roscpp 的 Topic/Master 调用链，最终通过 XML-RPC
调用 Python Master 的 `registerPublisher()`。Master 返回和推送的是发现信息；Publisher 与 Subscriber 建立 TCPROS
连接后，业务消息不会继续经过 Python Master。

这也是 Python Master 可以成立的重要原因：Master 处在控制面，而不是高频 Topic payload 的数据转发路径。
`roscpp` 中真正值得和本章交叉阅读的文件包括：

```text
clients/roscpp/src/libros/
├── master.cpp
├── xmlrpc_manager.cpp
├── topic_manager.cpp
├── service_manager.cpp
├── connection_manager.cpp
└── transport/transport_tcp.cpp
```

因此调试策略也应该分开：

```text
想看 Master 为什么注册/查询成这样
    -> Python 源码 + pdb/log

想看 C++ Node 怎样调用 Master、怎样建 TCPROS/ROSRPC、怎样分发 callback
    -> roscpp C++ 源码 + GDB/F5
```

这不是“两套 ROS”，而是同一个 ROS1 graph 中不同职责的实现语言边界。

---

## 3. 第一条源码链：`scripts/roscore -> roslaunch.main()`

源码入口：

```text
ros_comm/
└── tools/
    └── roslaunch/
        └── scripts/
            └── roscore
```

`roscore` 先定义 `-p`、`-w`、timeout、Master logger level 等选项，然后进入 `roslaunch.main()`，并主动追加 `--core`。

用等价阅读结构表示：

```text
解析 roscore 自己的命令行参数
    -> 不接受普通 launch 文件参数
    -> import roslaunch
    -> roslaunch.main(["roscore", "--core", ...])
```

此时还没有执行 `ROSMasterHandler.registerPublisher()`，甚至还没有创建 `rosmaster` 子进程。现在只是把“启动 ROS core”这个任务交给 roslaunch 的通用启动框架。

### 3.1 `--core` 到底改变什么

继续进入：

```text
ros_comm/tools/roslaunch/src/roslaunch/__init__.py
```

`roslaunch.main()` 看到 `--core` 后，最重要的行为有三件：

1. 不允许再传普通 launch 文件；
2. 未指定端口时把 Master 端口设为默认的 `11311`；
3. 构造 `ROSLaunchParent(..., is_core=True, ...)`，随后执行 `start()` 和 `spin()`。

因此第一段调用链可以固定为：

```text
roscore
  -> roslaunch.main(--core)
  -> ROSLaunchParent(... is_core=True ...)
  -> ROSLaunchParent.start()
  -> ROSLaunchParent.spin()
```

`spin()` 很重要：父进程不会在 Master 启动后立即退出，而是继续承担 process monitor 的主线程工作，直到整个 core 被关闭。

---

## 4. 为什么启动 `roscore` 时会自动出现 `/rosout`

`ROSLaunchParent._load_config()` 会调用：

```text
roslaunch.config.load_config_default(...)
```

而 `load_config_default()` 会先隐式加载 core 配置，再加载用户指定的 launch 文件。`roscore` 模式没有普通用户 launch 文件，所以这里最重要的是：

```text
get_roscore_filename()
    -> 优先 /etc/ros/roscore.xml
    -> 否则 roslaunch/resources/roscore.xml
```

Noetic 自带 `roscore.xml` 的核心内容只有三项：

```text
/rosversion
/rosdistro
/rosout node (respawn=true)
```

这里有一个容易混淆的点：

> `rosmaster` 并不是 `roscore.xml` 里的 `<node>`。

Master 由 `ROSLaunchRunner._launch_master()` 单独启动；`roscore.xml` 中的 `/rosout` 才作为 core Node 进入普通 Node 启动路径。

所以启动过程实际上分成两条：

```text
Master
    -> roslaunch 内部专门的 _launch_master()

/rosout
    -> roscore.xml
    -> core node
    -> 普通 Node 进程启动逻辑
```

---

## 5. `ROSLaunchParent` 先建立什么基础设施

继续进入：

```text
ros_comm/tools/roslaunch/src/roslaunch/parent.py
```

`ROSLaunchParent.start()` 会先执行 `_start_infrastructure()`。这里不是直接“调用 Master API”，而是在为整个 roslaunch 生命周期搭框架：

```text
_load_config()
    -> 得到 ROSLaunchConfig

_start_pm()
    -> 创建 ProcessMonitor

_start_server()
    -> 启动 roslaunch parent 自己的 XML-RPC server

_start_remote()
    -> 如果有远程 machine，再建立远程启动基础设施
```

对于本项目当前 Docker + host network 的本地学习场景，最需要抓住的是 `ProcessMonitor`：后续 `rosmaster` 和 `/rosout` 都由 roslaunch 纳入进程生命周期管理。

随后创建 `ROSLaunchRunner`，进入 runner 的 setup/launch 流程。

---

## 6. `ROSLaunchRunner._setup()`：Master 为什么一定先于 `/rosout`

源码：

```text
ros_comm/tools/roslaunch/src/roslaunch/launch.py
```

`ROSLaunchRunner._setup()` 的核心顺序是：

```text
_launch_master()
    -> 如果这次确实新启动了 Master
       -> _launch_core_nodes()

_launch_setup_executables()
_load_parameters()
launch_nodes()
```

对 `roscore` 来说，这意味着：

```text
先把 Master 拉起来并等它可用
    -> 在 _launch_master() 内先初始化 /run_id 等核心参数
    -> 再启动 /rosout
    -> 最后上传 ROSLaunchConfig 中其余参数
```

这个顺序是必须的，因为 `/rosout` 自己也是普通 ROS Node。它启动后同样需要向 Master 注册自己的 Publisher/Subscriber 等图状态。如果 Master 还没有可用，后面的 Node 注册发现链就无法正常建立。

### 6.1 已经有 Master 时会怎样

`_launch_master()` 会先检查配置中的 Master URI 是否已经有 Master 在线。

普通 roslaunch 可以复用已经存在的 Master；但 `roscore` 以 `is_core=True` 运行时，如果目标 Master 已经在线，会直接报错，避免再次启动第二套 core。

这解释了常见现象：

```text
终端 A 已经运行 roscore
终端 B 再运行 roscore
    -> 报已有 roscore/master 正在运行
```

这不是端口绑定失败后才偶然出现，而是 roslaunch 有显式的 core 冲突检查。

---

## 7. `create_master_process()`：这里才真正拼出 `rosmaster --core`

继续进入：

```text
ros_comm/tools/roslaunch/src/roslaunch/nodeprocess.py
```

`create_master_process()` 构造的关键命令等价于：

```text
rosmaster --core -p <port> -w <num_workers>
```

其中：

- package 是 `rosmaster`；
- 默认端口最终是 `11311`；
- `-w` 对应 Master 用于异步通知的 worker 数量；
- timeout/logger level 等选项会按 roscore 参数继续向下传递。

随后该 `LocalProcess` 被注册到 `ProcessMonitor`，再真正 `start()`。

因此从终端命令到 Master 子进程的完整入口已经可以写成：

```text
roscore
  -> roslaunch.main(--core)
  -> ROSLaunchParent.start()
  -> ROSLaunchRunner._setup()
  -> ROSLaunchRunner._launch_master()
  -> create_master_process()
  -> LocalProcess.start()
  -> rosmaster --core -p 11311 -w 3
```

到这里才真正跨越了 **进程边界**。

### 7.1 启动 `roscore` 期间到底向 Master 发送了什么

把“启动进程”和“发送 ROS 控制请求”分开看，`roscore` 启动阶段的网络交互会更清楚。`ROSLaunchRunner._launch_master()` 在创建 Master 前后会通过 Master XML-RPC 做一组探测和初始化。

第一步是在线探测。`roslaunch.core.Master.is_running()` 实际调用：

```text
Master.getPid('/roslaunch')
```

启动前调用一次，用于判断目标 `ROS_MASTER_URI` 是否已经有 Master；启动 `rosmaster` 子进程后又循环调用，直到 Master 能正常响应或超时。也就是说，终端刚执行 `roscore` 时，最早出现的一类 Master XML-RPC 请求不是 Topic 注册，而是 `getPid` 健康探测。

Master 可用后，roslaunch 会初始化本次 launch session：

```text
hasParam('/run_id')
    -> 不存在：setParam('/run_id', run_id)
    -> 已存在：getParam('/run_id') 并校验是否一致
```

如果 roslaunch parent 自己的 XML-RPC server URI 已建立，还会写入类似：

```text
/roslaunch/uris/<host>__<port>
```

用于记录当前 roslaunch parent 的 URI。

接着 `_launch_core_nodes()` 会先对 core node 执行：

```text
lookupNode('/roslaunch', '/rosout')
```

如果 Master 返回“没有这个 Node”，才真正启动 `/rosout`。因此 `/rosout` 是否需要创建，也是通过 Master 的控制面查询决定的。

core node 启动后，`_load_parameters()` 使用 XML-RPC MultiCall 把 launch config 中的参数批量送到 Parameter Server。对默认 `roscore.xml`，其中包括：

```text
/rosversion
/rosdistro
```

所以从 `roscore` 启动到 core ready，典型的控制请求可以概括为：

```text
getPid                 -> Master 在线探测
hasParam/getParam      -> 检查 /run_id
setParam               -> 写 /run_id、roslaunch URI 和 core 参数
lookupNode             -> 检查 /rosout 是否已存在

/rosout 启动以后
    -> 再像普通 Node 一样进行自己的 Topic/Service 注册
```

这里没有任何 Topic payload 被 `roscore` 转发。启动阶段这些交互仍然属于 XML-RPC 控制面。

---

## 8. 第二条源码链：`scripts/rosmaster -> rosmaster_main()`

Master 子进程入口：

```text
ros_comm/tools/rosmaster/scripts/rosmaster
```

这个脚本本身极薄，只负责导入 `rosmaster` package 并进入主函数。真正逻辑位于：

```text
ros_comm/tools/rosmaster/src/rosmaster/main.py
```

`rosmaster_main()` 依次完成：

```text
解析 --core / -p / -w / timeout / logger level
    -> 配置 master.log
    -> 选择端口（默认 11311）
    -> Master(port, num_workers)
    -> master.start()
    -> while master.ok(): sleep(0.1)
    -> 收到退出后 master.stop()
```

这里的循环不是 XML-RPC 请求处理循环。XML-RPC server 会在 `Master.start()` 内部另外启动线程；`rosmaster_main()` 主线程主要维持进程生命周期并等待 shutdown。

---

## 9. `Master.start()`：真正把“网络入口”和“业务处理器”装配起来

源码：

```text
ros_comm/tools/rosmaster/src/rosmaster/master.py
```

`Master` 这个类很薄。它的价值是把两个核心对象装起来：

```text
ROSMasterHandler
    -> Master API 真正的业务逻辑和状态

XmlRpcNode
    -> XML-RPC server 生命周期和网络监听
```

等价阅读结构：

```text
handler = ROSMasterHandler(num_workers)
master_node = XmlRpcNode(port, handler)
master_node.start()
等待 master_node.uri 就绪
保存 handler / master_node / uri
```

这里形成一个非常重要的职责分工：

```text
XmlRpcNode
    -> “请求怎么从网络进来”

ROSMasterHandler
    -> “registerPublisher 进来以后该做什么”
```

这两个对象不能混成一个“Master 类”来理解。

---

## 10. `XmlRpcNode` 与线程模型：Master API 在什么线程执行

`XmlRpcNode` 来自：

```text
ros_comm/tools/rosgraph/src/rosgraph/xmlrpc.py
```

`XmlRpcNode.start()` 会启动一条 Python thread 执行 server `run()`。初始化阶段创建的是：

```text
ThreadingXMLRPCServer
```

它继承 Python `socketserver.ThreadingMixIn` 和 `SimpleXMLRPCServer`，因此允许多个 XML-RPC 请求并发进入。

这意味着 Master 不是：

```text
一个主线程串行处理所有 registerPublisher/registerSubscriber
```

而更接近：

```text
rosmaster main thread
    -> 维持生命周期

XmlRpcNode server thread
    -> accept / serve

XML-RPC request worker threads
    -> 执行 ROSMasterHandler 的 public API
```

正因为 XML-RPC API 可能并发进入，`ROSMasterHandler` 在修改图状态时需要 `ps_lock`。`RegistrationManager` 源码也明确说明其自身不是线程安全容器，调用方需要负责外部加锁。

除此之外还有第二类线程：`MarkedThreadPool`。它不是用来接收 Master API 的，而是用来执行 Master 主动回调其他 Node 的任务，例如 `publisherUpdate` 和同名 Node 替换时的 `shutdown`。

所以本章后续遇到异步路径时，必须区分：

```text
XML-RPC request thread
    -> 收到 Node -> Master 请求

MarkedThreadPool worker
    -> Master -> Node 的异步回调
```

---

## 11. Master 内部类关系：先把对象所有权看清

到这里再看整体对象关系，已经不会把类名当成抽象架构名词。

![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/c2f4a8878a0e4508b050cab690e0a499.png)


关键对象可以压缩成下面这张表：

| 类/对象 | 所在 package | 持有什么 | 主要职责 |
| --- | --- | --- | --- |
| `ROSLaunchParent` | `roslaunch` | config、runner、ProcessMonitor | 组织整个 launch/core 生命周期 |
| `ROSLaunchRunner` | `roslaunch` | launch config、process monitor | 启动 Master、core Node、普通 Node、参数 |
| `Master` | `rosmaster` | `ROSMasterHandler`、`XmlRpcNode` | 装配并启动/停止 Master |
| `XmlRpcNode` | `rosgraph` | `ThreadingXMLRPCServer`、handler | 建立 XML-RPC 网络入口 |
| `ROSMasterHandler` | `rosmaster` | 注册管理器、参数字典、线程池、Topic type 表 | 实现 Master/Parameter XML-RPC API |
| `RegistrationManager` | `rosmaster` | `nodes` + 四类 `Registrations` | 维护 Node 与注册关系 |
| `NodeRef` | `rosmaster` | 某 Node 的 API 与注册 key 列表 | 表示某个 `caller_id` 当前拥有哪些注册 |
| `Registrations` | `rosmaster` | `map`、Service 时额外有 `service_api_map` | 建立 Topic/Service/Param key 到 Node 的索引 |
| `ParamDictionary` | `rosmaster` | 参数树 + `RegistrationManager` | Parameter Server 数据存取和订阅更新 |
| `MarkedThreadPool` | `rosmaster` | 异步任务 | 向 Node 发 `publisherUpdate`、`paramUpdate`、`shutdown` 等回调 |

这一层对象关系解释了后面几乎所有 Master 行为。

---

## 12. Node 到底怎样“注册到 Master”：并不存在 `registerNode()`

这是理解 Master 注册表最关键的一点。

从 ROS 图概念看会自然认为：

```text
Node 启动
    -> registerNode(node_name, node_uri)
    -> 然后再注册 Topic/Service
```

Noetic Master API 并没有这种独立 `registerNode()` 流程。

Node 是在注册下面这些资源时，**作为副作用进入 `RegistrationManager.nodes`**：

```text
registerPublisher
registerSubscriber
registerService
subscribeParam
```

它们最终都会走向类似：

```text
RegistrationManager._register(...)
    -> _register_node_api(caller_id, caller_api)
    -> NodeRef.add(type, key)
    -> 对应 Registrations.register(...)
```

所以 Master 对一个 Node 的核心身份实际上是：

```text
caller_id
    -> Node 名称，例如 /talker

caller_api
    -> 这个 Node 自己的 XML-RPC URI
       例如 http://host:random_port/
```

### 12.1 `NodeRef` 记录什么

一个 `NodeRef` 维护：

```text
id                  -> caller_id
api                 -> caller_api
param_subscriptions -> 订阅了哪些 parameter key
topic_subscriptions -> 订阅了哪些 topic
topic_publications  -> 发布了哪些 topic
services            -> 提供了哪些 service
```

也就是说，`NodeRef` 不是 Node 的进程对象，不持有 PID，不负责执行业务 callback。它只是 Master 图状态中的一条“这个 Node 当前登记了什么”的记录。

### 12.2 Node 什么时候从 `nodes` 删除

正常 unregister 时，`RegistrationManager._unregister()` 会从对应 `Registrations` 删除资源，同时从 `NodeRef` 移除 key。

当：

```text
param_subscriptions
+ topic_subscriptions
+ topic_publications
+ services
```

全部为空时，`NodeRef.is_empty()` 为真，Master 才会删除 `nodes[caller_id]`。

因此 Node 的 Master 侧存在性是由注册资源驱动的，不是一个独立“Node 在线状态机”。

---

## 13. 遇到同名 Node 怎么办：为什么后启动的会顶掉前一个

`RegistrationManager._register_node_api()` 里专门处理：

```text
相同 caller_id
但 caller_api 不同
```

这表示 Master 看到：

```text
同一个 Node 名称
    -> 现在从另一个 Node XML-RPC URI 又来注册了
```

处理逻辑是：

```text
找到旧 NodeRef
    -> caller_api 相同：继续复用旧 NodeRef
    -> caller_api 不同：
         1. 把 shutdown(old_api, reason) 放入 MarkedThreadPool
         2. 创建新的 NodeRef
         3. 替换 nodes[caller_id]
         4. 清理旧 caller_id 的 Publisher/Subscriber/Service/Param 注册
         5. 再写入本次新注册
```

这里有一个明确的异步边界：

```text
当前注册请求线程
    -> 只负责发现冲突、更新 Master 内部图状态、提交 shutdown task

MarkedThreadPool worker
    -> 再通过旧 Node 的 XML-RPC API 调用 shutdown
```

所以“同名 Node 后启动会让前一个退出”不是 roslaunch 特有行为。即使两个进程不是由同一个 launch 文件启动，只要它们向同一个 Master 使用相同 `caller_id`、但暴露不同 `caller_api`，Master 的注册逻辑就会触发替换。

---

## 14. Topic 到达 Master 后执行什么：Publisher 与 Subscriber 是两张注册表

`ROSMasterHandler.__init__()` 通过 `RegistrationManager` 持有：

```text
publishers
subscribers
services
param_subscribers
```

其中 Topic 不是一张“Topic 对象表”，而是至少要同时理解：

```text
publishers.map
    topic -> [(caller_id, caller_api), ...]

subscribers.map
    topic -> [(caller_id, caller_api), ...]

topics_types
    topic -> datatype
```

### 14.1 Subscriber 先启动：`registerSubscriber()` 做什么

一个 roscpp Subscriber 在第 07 章客户端侧最终会向 Master 发：

```text
registerSubscriber(caller_id, topic, topic_type, caller_api)
```

Master 侧执行顺序可以压缩为：

```text
XML-RPC request thread
    -> 获取 ps_lock
    -> RegistrationManager.register_subscriber(...)
       -> _register_node_api(...)
       -> NodeRef.add(TOPIC_SUBSCRIPTIONS, topic)
       -> subscribers.register(...)
    -> 如果 topic type 尚未知，则记录类型
    -> 查询当前 publishers.get_apis(topic)
    -> 释放锁
    -> 把当前 Publisher Node XML-RPC URI 列表作为返回值给 Subscriber
```

所以 Subscriber 注册的返回值不是 TCPROS 地址，而是：

```text
当前已经发布这个 Topic 的 Node XML-RPC API URI 列表
```

Subscriber 后续再直接联系这些 Publisher Node。

### 14.2 Publisher 后启动：`registerPublisher()` 做什么

Publisher 向 Master 注册后，Master 会：

```text
获取 ps_lock
    -> register_publisher(...)
    -> 更新 topics_types
    -> 查询该 Topic 当前全部 Publisher URI
    -> 查询该 Topic 当前全部 Subscriber URI
    -> _notify_topic_subscribers(...)
    -> 返回当前 Subscriber URI 列表
```

这里最值得继续进入的是 `_notify_topic_subscribers()`。

它不会在当前 XML-RPC request thread 中同步逐个连接所有 Subscriber，而是把通知任务交给 `MarkedThreadPool`。worker 随后对每个 Subscriber 的 `caller_api` 调用：

```text
publisherUpdate('/master', topic, pub_uris)
```

于是形成：

```text
Publisher Node
    -> Master.registerPublisher()

Master
    -> 更新自己的注册表
    -> 异步通知 Subscriber Node.publisherUpdate()

Subscriber Node
    -> 根据新的 Publisher API URI 列表
    -> 对新增 Publisher 调 requestTopic()
```

到 `requestTopic()` 以后，Master 就退出 Topic 建链主路径。TCPROS/UDPROS 的协议协商和消息字节传输属于第 08 章的数据面。

### 14.3 Master 在 Topic 中到底存什么、不存什么

Master 存：

```text
Topic 名称
Topic datatype
Publisher caller_id + Node XML-RPC URI
Subscriber caller_id + Node XML-RPC URI
```

Master 不存：

```text
Publisher 的业务消息队列
Subscriber callback queue
TCPROS Connection Header
序列化后的消息 payload
Publication / Subscription 对象
```

这些都属于具体 Node 进程内部或后续点对点数据连接。

---

## 15. Service 到达 Master 后执行什么：这里存的是 `service_api`

Service 和 Topic 在 Master 中有一个本质差异。

`registerService()` 参数同时包含：

```text
caller_api
    -> Service Server 所在 Node 的 XML-RPC API

service_api
    -> 真正 Service ROSRPC endpoint
       rosrpc://host:port
```

Master 会把 Node 身份写入 `NodeRef`，同时把 Service 名称写入 `services` 注册表。

`Registrations` 对 Service 还有一个特殊约束：

```text
同一个 service key 只允许一个当前有效 provider
```

因此 Service 注册表额外维护 `service_api_map`。

### 15.1 Client 查询 Service 时 Master 返回什么

Service Client 调用：

```text
lookupService(caller_id, service)
```

Master 直接从 `services.get_service_api(service)` 取出并返回：

```text
rosrpc://host:port
```

这和 Topic 的发现链不同：

```text
Topic
    Master 返回 Publisher 的 Node XML-RPC URI
    -> Subscriber 再 requestTopic()
    -> 协商实际 Transport

Service
    Master lookupService() 直接返回 Service 的 ROSRPC URI
    -> Client 直接连接 Service Server
```

因此 Service 请求体和响应体也不经过 Master。Master 仍然只承担控制面发现。

---

## 16. Action 到 Master 后怎么办：Master 根本没有 Action 注册表

阶段 11 已经确认 ActionClient/ActionServer 最终建立五类 Topic：

```text
<action>/goal
<action>/cancel
<action>/status
<action>/feedback
<action>/result
```

从 Master 源码可以进一步得到一个更强的结论：

> ROS1 Master 不认识“Action”这种独立 graph entity。

直接看 `getSystemState()` 的系统状态结构即可：

```text
[publishers, subscribers, services]
```

没有：

```text
actions
```

`RegistrationManager` 的注册类型也只有：

```text
TOPIC_SUBSCRIPTIONS
TOPIC_PUBLICATIONS
SERVICE
PARAM_SUBSCRIPTIONS
```

同样没有 `ACTION`。

因此 Action 到达 Master 后的真实过程是：

| Action 端 | actionlib 创建的 ROS 接口 | Master 实际看到的行为 |
| --- | --- | --- |
| Client | publish `goal` | 普通 `registerPublisher()` |
| Client | publish `cancel` | 普通 `registerPublisher()` |
| Client | subscribe `status` | 普通 `registerSubscriber()` |
| Client | subscribe `feedback` | 普通 `registerSubscriber()` |
| Client | subscribe `result` | 普通 `registerSubscriber()` |
| Server | subscribe `goal` / `cancel` | 普通 Subscriber 注册 |
| Server | publish `status` / `feedback` / `result` | 普通 Publisher 注册 |

Goal ID、状态迁移、Cancel/Preempt、Feedback/Result 关联都由 actionlib 客户端和服务端实现，不进入 `rosmaster` 的数据模型。

这也解释了为什么：

```text
rostopic list
```

能直接看到 Action 展开的 Topic，而 Master 不需要额外提供一个 `registerAction()`。

---

## 17. Parameter Server 在哪里：就在同一个 `rosmaster` 进程里

`ROSMasterHandler.__init__()` 同时创建：

```text
ParamDictionary(self.reg_manager)
```

所以 Noetic 的默认架构不是：

```text
rosmaster process
parameter_server process
```

而是：

```text
同一个 rosmaster 进程
    -> Master API
    -> Parameter Server API
```

二者共用同一个 XML-RPC server 入口。

### 17.1 `ParamDictionary` 存什么

其核心状态是：

```text
parameters = {}
```

按 ROS namespace 形成嵌套参数树，并通过自己的 `RLock` 保护读写。

普通：

```text
getParam
setParam
deleteParam
hasParam
searchParam
getParamNames
```

最终都操作这个参数字典。

### 17.2 `subscribeParam()` 为什么又回到 `RegistrationManager`

参数订阅不能只存一个字典值，还必须记录：

```text
哪些 Node 想在某个参数变化时收到 paramUpdate
```

所以 `param_subscribers` 也属于 `RegistrationManager`。

当参数被修改或删除时，`ParamDictionary` 计算受影响的 subscriber，再由 Master 异步回调对应 Node 的：

```text
paramUpdate(...)
```

因此 Parameter 和 Topic 在控制面上有相似结构：

```text
Node 先把“我关心什么”登记到 Master
    -> Master 内部状态发生变化
    -> Master 再根据登记结果主动回调 Node XML-RPC API
```

但 Parameter 没有 TCPROS 数据面，值本身就是通过 XML-RPC API 读写/推送。

---

## 18. `ROSMasterHandler` 到底是怎样的数据中心

把 `ROSMasterHandler.__init__()` 的核心字段放在一起，可以形成 Master 的最小心智模型：

```text
ROSMasterHandler
├── thread_pool
│   └── Master -> Node 的异步 XML-RPC callback
│
├── ps_lock
│   └── 保护 graph registration 相关共享状态
│
├── reg_manager
│   ├── nodes
│   ├── publishers
│   ├── subscribers
│   ├── services
│   └── param_subscribers
│
├── topics_types
│   └── topic -> datatype
│
└── param_server
    └── ParamDictionary
```

看到这里后，Master API 就不再是几十个彼此独立的函数。它们基本都属于下面几类操作：

| API 类型 | 典型函数 | 实际目的 |
| --- | --- | --- |
| 注册 | `registerPublisher`、`registerSubscriber`、`registerService` | 修改 `RegistrationManager` |
| 注销 | `unregisterPublisher` 等 | 删除注册关系，必要时删除空 `NodeRef` |
| 查询 | `lookupNode`、`lookupService`、`getSystemState` | 从注册表读取发现信息 |
| Topic 通知 | `publisherUpdate` 的发起侧逻辑 | Publisher 集合变化后通知 Subscriber |
| Parameter | `getParam`、`setParam`、`subscribeParam` | 访问 `ParamDictionary` 和参数订阅表 |
| 生命周期 | `shutdown`、`getPid` | Master 自身管理/诊断 |

---

## 19. `lookupNode()`、`lookupService()`、`getSystemState()` 分别回答什么

这三个 API 很适合用来反推 Master 保存的数据边界。

### 19.1 `lookupNode(caller_id, node_name)`

回答：

```text
这个 Node 的 XML-RPC API 在哪里？
```

数据来源：

```text
RegistrationManager.nodes[node_name].api
```

返回的是：

```text
http://host:node_xmlrpc_port/
```

它不是 Topic 的 TCPROS 地址，也不是 Service 的 ROSRPC 地址。

### 19.2 `lookupService(caller_id, service_name)`

回答：

```text
这个 Service Server 的 ROSRPC endpoint 在哪里？
```

数据来源：

```text
services.service_api_map
```

返回的是：

```text
rosrpc://host:service_port
```

### 19.3 `getSystemState(caller_id)`

回答：

```text
当前 ROS graph 有哪些 Publisher、Subscriber、Service provider？
```

返回结构是：

```text
[
  publishers,
  subscribers,
  services
]
```

这是一张 **控制面快照**，不是运行时数据连接快照。它不能告诉某个 TCPROS socket 当前发送到第几个字节，也不能告诉某个 callback 是否正在执行。

---

## 20. 把控制面和数据面重新放到一张表里

完成 Master 源码后，阶段 06～11 中出现的几个通信机制可以统一成下面的边界：

| 机制 | Master 保存/处理什么 | Master 返回或通知什么 | 后续业务数据是否经过 Master |
| --- | --- | --- | --- |
| Node | `caller_id -> NodeRef(caller_api)` | `lookupNode()` 返回 Node XML-RPC URI | 不适用 |
| Topic | Publisher/Subscriber 注册 + Topic type | Publisher URI 列表、`publisherUpdate` | 否，后续 `requestTopic -> TCPROS/UDPROS` 点对点 |
| Service | Service provider + `service_api` | `lookupService()` 返回 ROSRPC URI | 否，Client 直接连 Server |
| Action | 没有 Action 表；只看到五类 Topic | 按普通 Topic 规则发现 | 否，仍是 Topic 数据面 |
| Parameter | 参数字典 + param subscriber | XML-RPC 返回值或 `paramUpdate` | Parameter 本身就走 XML-RPC |

这张表可以把 Master 的角色压缩成一句工程判断：

> ROS1 Master 是名称注册、发现和参数控制面的中心，不是 Topic/Service/Action 业务 payload 的中心转发器。

---

## 21. 在当前 `ros1-docker` 中怎样准备 Master 源码

第 05 章已经使用 `ros_debug_ws` 阅读 `ros_comm`。如果源码已经存在，不要重复 clone。

Container 中确认：

```bash
ls /workspace/ros_debug_ws/src/ros_comm/tools/roslaunch
ls /workspace/ros_debug_ws/src/ros_comm/tools/rosmaster
ls /workspace/ros_debug_ws/src/ros_comm/tools/rosgraph
```

如果还没有 `ros_comm`：

```bash
cd /workspace/ros_debug_ws/src

git clone --branch noetic-devel --single-branch \
    https://github.com/ros/ros_comm.git
```

本章需要的主要源码都在这一个仓库里，不需要为了 Master 再 clone roscpp_core。

---

## 22. 怎样构建自己的 `roslaunch/rosmaster` overlay

进入 Debug workspace：

```bash
cd /workspace/ros_debug_ws
source /opt/ros/noetic/setup.bash
```

如果该 workspace 还没配置：

```bash
catkin init

catkin config \
    --merge-devel \
    --extend /opt/ros/noetic \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

然后构建：

```bash
catkin build rosgraph rosmaster roslaunch
```

这里保留 `Debug` 配置是为了和现有 debug workspace 一致；但要再次强调：

```text
它不会让 rosmaster.py 获得“DWARF 调试信息”
```

因为 Master 主体是 Python。

构建后：

```bash
source /workspace/ros_debug_ws/devel/setup.bash
```

先不要只看 `which roscore`，因为 shell PATH 中仍可能首先出现 `/opt/ros/noetic/bin/roscore`。判断 Python package 是否真的来自 overlay，更可靠的是：

```bash
python3 -c 'import roslaunch; print(roslaunch.__file__)'
python3 -c 'import rosmaster; print(rosmaster.__file__)'
python3 -c 'import rosgraph; print(rosgraph.__file__)'
```

期望路径优先落在：

```text
/workspace/ros_debug_ws/...
```

而不是只有：

```text
/opt/ros/noetic/lib/python3/dist-packages/...
```

这样才能证明当前 Python import 的确使用了 checkout 的源码 overlay。

---

## 23. 最适合这套 Python 源码的动态阅读方式

### 23.1 第一遍：直接运行完整 `roscore`

先观察完整进程关系：

```bash
source /workspace/.devcontainer/ros_env.bash
roscore --master-logger-level debug
```

另一个终端：

```bash
ps -ef | grep -E 'roscore|rosmaster|rosout' | grep -v grep
```

预期能够区分：

```text
roscore / roslaunch parent
rosmaster --core ...
rosout
```

这一步的目标不是背 PID，而是确认源码阅读里的三个运行角色确实存在。

### 23.2 第二遍：人为触发 Master API

保持 roscore 运行，再启动阶段 02/03 已有的 Node，例如 Publisher 和 Subscriber，然后观察：

```bash
rosnode list
rostopic list
rostopic info /hello
rosservice list
rosparam list
```

这些命令看到的是 Master/Node API 汇总出来的控制面信息。与此同时可以读取本次 ROS session 的 `master.log`，确认 `registerPublisher`、`registerSubscriber`、lookup 等 API 调用顺序。

### 23.3 第三遍：单独调 `rosmaster`，减少 roslaunch 干扰

如果目标是逐行理解 `ROSMasterHandler`，可以暂时不运行完整 roscore，而是直接启动 Master：

```bash
source /workspace/.devcontainer/ros_env.bash

python3 -m pdb \
    /workspace/ros_debug_ws/src/ros_comm/tools/rosmaster/scripts/rosmaster \
    --core -p 11311 -w 3
```

这样能直接在 Python debugger 中进入：

```text
rosmaster_main()
Master.start()
ROSMasterHandler.__init__()
registerPublisher()
registerSubscriber()
registerService()
lookupService()
getSystemState()
```

此时没有完整 roscore 自动启动的 `/rosout`，但 Master 和 Parameter Server 本身已经可以用于研究注册逻辑。若实验需要 `/rosout`，可在另一个终端手工运行：

```bash
rosrun rosout rosout
```

这种“先隔离 Master，再补 core Node”的方式，比一上来同时调 roslaunch、rosmaster、rosout 三个进程更容易建立调用链。

---

## 24. 源码应该按什么顺序读：不要从 `master_api.py` 随机翻函数

本章最推荐的真实阅读顺序是：

```text
1. roslaunch/scripts/roscore
   -> 先回答 roscore 到底是什么

2. roslaunch/src/roslaunch/__init__.py
   -> 找 main()、--core、ROSLaunchParent.start()/spin()

3. roslaunch/src/roslaunch/config.py
   -> 找 load_config_default()、get_roscore_filename()

4. roslaunch/resources/roscore.xml
   -> 明确 /rosout 与两个 core 参数从哪里来

5. roslaunch/src/roslaunch/parent.py
   -> 看 _start_infrastructure() 和 runner 初始化

6. roslaunch/src/roslaunch/launch.py
   -> 看 _launch_master()、_launch_core_nodes()、_setup()

7. roslaunch/src/roslaunch/nodeprocess.py
   -> 看 create_master_process() 实际拼出的 rosmaster 命令

8. rosmaster/scripts/rosmaster
   -> 进入真正 Master 子进程入口

9. rosmaster/src/rosmaster/main.py
   -> 看 rosmaster_main() 的生命周期

10. rosmaster/src/rosmaster/master.py
    -> 看 Master.start() 如何装配 handler + XmlRpcNode

11. rosgraph/src/rosgraph/xmlrpc.py
    -> 看 XmlRpcNode / ThreadingXMLRPCServer 的网络与线程入口

12. rosmaster/src/rosmaster/master_api.py
    -> 再读 register/lookup/parameter API

13. rosmaster/src/rosmaster/registrations.py
    -> 把 NodeRef、Registrations、RegistrationManager 的数据结构闭合

14. rosmaster/src/rosmaster/paramserver.py
    -> 最后补 Parameter Server 与 param subscriber

15. rosmaster/src/rosmaster/threadpool.py
    -> 跟 publisherUpdate / shutdown / paramUpdate 的异步回调线程
```

这个顺序的核心原则是：

```text
先弄清“谁把谁启动起来”
    -> 再弄清“请求怎样进进程”
    -> 再弄清“API 改哪张表”
    -> 最后看异步通知和边界
```

如果一开始就打开 800 多行的 `master_api.py` 从 `registerPublisher` 往下看，很容易知道单个函数在做什么，却不知道它为什么会在这个进程、这个线程、这个生命周期阶段被调用。

---

## 25. 四条典型事件重新走一遍

### 25.1 一个新 Publisher 出现

```text
roscpp TopicManager
    -> Master.registerPublisher(caller_id, topic, type, caller_api)

ROSMasterHandler
    -> ps_lock
    -> RegistrationManager.register_publisher()
    -> _register_node_api()
    -> NodeRef 记录 topic publication
    -> publishers.map 记录 topic -> Node
    -> topics_types 记录 datatype
    -> 查询当前 Publisher/Subscriber URI
    -> 提交 publisherUpdate task

MarkedThreadPool
    -> 调 Subscriber Node.publisherUpdate()

Subscriber Node
    -> 对新增 Publisher Node.requestTopic()
    -> 进入 TCPROS/UDPROS 数据面
```

### 25.2 一个新 Subscriber 出现

```text
roscpp TopicManager
    -> Master.registerSubscriber(...)

ROSMasterHandler
    -> RegistrationManager.register_subscriber()
    -> NodeRef 记录 topic subscription
    -> subscribers.map 记录 topic -> Node
    -> 返回当前 Publisher Node XML-RPC URI 列表

Subscriber Node
    -> 对列表中的 Publisher requestTopic()
```

### 25.3 一个 Service Server 出现

```text
ServiceManager
    -> Master.registerService(caller_id, service, service_api, caller_api)

ROSMasterHandler
    -> RegistrationManager.register_service()
    -> NodeRef 记录 service
    -> services.map 记录 provider
    -> service_api_map 记录 rosrpc://host:port

Service Client
    -> Master.lookupService(service)
    -> 拿到 service_api
    -> 直接连接 Service Server
```

### 25.4 一个 Action Server 出现

```text
ActionServer
    -> subscribe goal
    -> subscribe cancel
    -> advertise status
    -> advertise feedback
    -> advertise result

这些操作分别进入普通：
    registerSubscriber()
    registerPublisher()

Master
    -> 不创建任何 Action 对象
    -> 不维护 Goal 状态
    -> 不处理 Cancel/Preempt

Action 状态机
    -> 完全留在 actionlib Node 内部
```

---

## 26. 从 roscore 源码回到真实工程：整套 ROS1 系统怎样启动

前 25 节解决的是“ROS core 内部怎么工作”。到这里还需要把源码认知落回工程，否则仍然会留下三个实际问题：

```text
项目部署后，谁来启动 roscore？
谁来把所有业务 Node 一起启动？
Node A 依赖 Node B 时，怎样保证 B 真正 READY？
```

当前仓库新增一套最小实现，把这三个问题直接落到可运行文件中：

```text
compose.runtime.yaml
ros_ws/src/ros1_bringup/launch/system.launch
ros_ws/src/ros1_hello/src/ready_server.cpp
ros_ws/src/ros1_hello/src/ready_client.cpp
```

这套实现不改变原来的开发模式。`compose.yaml` 仍然让 `ros1-dev` 执行 `sleep infinity`，方便 VS Code、GDB、
手工 `roscore/roslaunch` 和源码阅读；`compose.runtime.yaml` 才代表“设备运行态”。

![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/ea12de45c9fa44dab7d20492e5ec554e.png)


### 26.1 实际工程通常不需要人工先敲 `roscore`

开发阶段为了观察每一个进程，可以手工执行：

```bash
roscore
rosrun ros1_hello hello_node
rosrun ros1_hello hello_listener
```

但部署时更常见的是只保留一个顶层入口：

```bash
roslaunch ros1_bringup system.launch
```

普通 roslaunch 运行时会检查目标 Master。对于当前工程默认的本机 Master 配置，如果 Master 尚未运行，
`ROSLaunchRunner` 会进入 `_launch_master()`，最终调用前文已经读过的 `create_master_process()`：

```text
roslaunch ros1_bringup system.launch
    -> ROSLaunchRunner.launch()
    -> _setup()
    -> _launch_master()
    -> create_master_process()
    -> rosmaster --core ...
```

因此工程入口不需要再额外包装一条“先执行 `roscore`”命令。`roscore` 适合独立学习和手工调试；顶层
`roslaunch` 更适合把 Master、core Node 和业务 Node 组织成一个系统启动入口。

这里的“自动启动 Master”有明确边界：普通 `roslaunch` 只有在目标 Master 地址属于本机时才会自动创建新的 Master。
如果 `ROS_MASTER_URI` 指向另一台机器，而远程 Master 又无法联系，Noetic `validate_master_launch()` 会直接报错，
不会在本机偷偷启动一个 Master 来替代远程 Master。当前仓库使用 Host network 和默认本机 Master，因此走的是前一种路径。

### 26.2 为什么新增 `ros1_bringup`，而不是继续把所有 launch 塞进 `ros1_hello`

业务 package 和系统启动 package 的职责不同：

```text
ros1_hello
    -> 提供可执行 Node、消息和局部 launch

ros1_bringup
    -> 决定整套系统由哪些 package/Node 组成
    -> 提供顶层 system.launch
```

当前新增目录：

```text
ros_ws/src/
├── ros1_hello/
│   └── src/
│       ├── ready_server.cpp
│       └── ready_client.cpp
│
└── ros1_bringup/
    ├── CMakeLists.txt
    ├── package.xml
    └── launch/
        └── system.launch
```

这种拆法的价值在于：驱动、感知、控制 package 可以各自维护自己的实现，而 bringup package 只负责系统装配。
后续阶段 B 增加 CAN/UART/Ethernet Driver Node 时，也可以继续沿这个边界扩展，而不需要把所有实现放进同一个 package。

### 26.3 `system.launch` 怎样一次启动工程 Node

当前 `ros1_bringup/launch/system.launch` 先复用第 03 章已有的 `/chatter` 示例，然后再启动 READY 依赖实验：

```xml
<launch>
  <arg name="publish_rate" default="1.0"/>
  <arg name="driver_startup_delay" default="3.0"/>
  <arg name="ready_timeout" default="10.0"/>

  <include file="$(find ros1_hello)/launch/hello.launch">
    <arg name="publish_rate" value="$(arg publish_rate)"/>
  </include>

  <node pkg="ros1_hello"
        type="ready_client"
        name="ready_client"
        output="screen"
        required="true">
    <param name="wait_timeout" value="$(arg ready_timeout)"/>
  </node>

  <node pkg="ros1_hello"
        type="ready_server"
        name="ready_server"
        output="screen"
        required="true">
    <param name="startup_delay" value="$(arg driver_startup_delay)"/>
  </node>
</launch>
```

这里故意把依赖方 `ready_client` 写在提供方 `ready_server` 前面。这个顺序只用于把问题暴露得更直观：
**launch 文件中的书写位置不是业务 READY 契约。**

即使某个进程先被 roslaunch 创建，也只能说明操作系统已经开始运行这个进程，不能推出下面这些条件已经成立：

```text
CAN 设备已经打开
串口已经配置完成
相机已经完成初始化
Service 已经 advertise
Action Server 已经 start
TF 已经发布
控制器已经进入 RUNNING
```

进程“存在”和业务“可用”必须分开建模。

### 26.4 `ready_server`：初始化完成以后才发布 READY Service

`ready_server.cpp` 用 3 秒延时模拟真实驱动初始化。延时只是为了让实验过程可见，不是依赖管理方案。
真正的契约是：**初始化完成前不 advertise READY Service；初始化完成后才注册 `/demo_driver/ready`。**

核心代码：

```cpp
double startup_delay = 3.0;
pnh.param("startup_delay", startup_delay, 3.0);

ROS_INFO("[ready_server] simulating driver initialization for %.1f s", startup_delay);
if (startup_delay > 0.0)
{
  ros::WallDuration(startup_delay).sleep();
}

ros::ServiceServer ready_service =
    nh.advertiseService("/demo_driver/ready", handleReady);

ROS_INFO("[ready_server] READY: service /demo_driver/ready is available");
ros::spin();
```

这条路径和本章前半部分的 Master 服务端源码可以直接对应：

```text
ready_server
    -> advertiseService("/demo_driver/ready")
    -> roscpp ServiceManager
    -> Master.registerService(...)
    -> ROSMasterHandler.registerService()
    -> RegistrationManager.services
```

此时 Master 才能够通过 `lookupService()` 返回这个 Service 的 `rosrpc://...` endpoint。

### 26.5 `ready_client`：等待 READY，而不是 `sleep(5)` 猜时间

依赖方启动后立刻等待：

```cpp
const std::string ready_service_name = "/demo_driver/ready";

if (!ros::service::waitForService(
        ready_service_name,
        ros::Duration(wait_timeout)))
{
  ROS_ERROR("[ready_client] timeout waiting for %s", ready_service_name.c_str());
  return 1;
}
```

`ros::service::waitForService()` 还不是单纯“看 Master 注册表里有没有这个名字”。Noetic `service.cpp` 中，
`waitForService()` 会循环调用 `service::exists()`；`exists()` 先经 `ServiceManager::lookupService()` 获取 Service
host/port，再建立同步 `TransportTCP` 连接并发送 `probe=1` 的 Service header。也就是说，这个等待条件不仅要求 Master
已经知道 Service provider，还要求对应 ROSRPC endpoint 能够接受连接。

READY Service 可用后，再真正调用一次：

```cpp
ros::ServiceClient ready_client =
    nh.serviceClient<std_srvs::Trigger>(ready_service_name);

std_srvs::Trigger ready_request;
if (!ready_client.call(ready_request) || !ready_request.response.success)
{
  ROS_ERROR("[ready_client] readiness check failed");
  return 1;
}
```

只有这一步成功后，才进入业务路径：

```cpp
ros::Subscriber chatter_sub =
    nh.subscribe("/chatter", 10, chatterCallback);

ROS_INFO("[ready_client] business path enabled; subscribed to /chatter");
ros::spin();
```

所以这里建立的因果关系是：

```text
ready_client 进程已经启动
    !=
业务已经开始

/demo_driver/ready 可发现并调用成功
    -> dependency READY
    -> 才建立 /chatter 业务订阅
```

这比下面这种写法可靠：

```text
启动 A
sleep 5
启动 B
```

固定延时只能假设“5 秒通常够用”，不能表达真实状态。机器负载、USB 枚举、CAN/串口初始化、网络连接和设备
固件启动时间变化后，这种假设很容易失效。

### 26.6 构建并运行完整 bringup

在开发 Container 中先构建新增 package：

```bash
cd /workspace/ros_ws
source /opt/ros/noetic/setup.bash

catkin build ros1_hello ros1_bringup
source /workspace/ros_ws/devel/setup.bash
```

这里不需要单独启动 `roscore`。直接执行：

```bash
roslaunch ros1_bringup system.launch
```

默认 `ready_server` 会模拟 3 秒初始化，因此日志顺序应该能够观察到类似阶段：

```text
ready_client: waiting for /demo_driver/ready
ready_server: simulating driver initialization for 3.0 s
...
ready_server: READY: service /demo_driver/ready is available
ready_client: dependency ready: demo_driver is ready
ready_client: business path enabled; subscribed to /chatter
```

另开终端可以观察：

```bash
rosnode list
rosservice list | grep /demo_driver/ready
rostopic info /chatter
```

也可以调大初始化时间，让等待过程更明显：

```bash
roslaunch ros1_bringup system.launch \
  driver_startup_delay:=8.0 \
  ready_timeout:=15.0
```

如果把 `ready_timeout` 设置得比初始化时间短，`ready_client` 会在依赖未就绪时退出，而不是假装系统已经可以工作。
当前 `system.launch` 把 `ready_client` 和 `ready_server` 都标记为 `required="true"`，因此任一关键 Node 退出都会让
roslaunch 关闭当前进程组；运行在 `compose.runtime.yaml` 下时，外层 Docker restart policy 再决定是否重新拉起整套运行态。

### 26.7 运行态 Docker：Container 启动后直接执行顶层 roslaunch

开发 Compose 继续保持：

```yaml
# compose.yaml
command: sleep infinity
```

因为开发 Container 的目标是长期存在，方便 VS Code attach、GDB、多个终端和手工实验。

运行态另建 `compose.runtime.yaml`：

```yaml
services:
  ros1-runtime:
    image: ros1-noetic-dev:local
    container_name: ros1-runtime
    network_mode: host
    init: true
    restart: unless-stopped

    volumes:
      - type: bind
        source: .
        target: /workspace

    working_dir: /workspace

    command:
      - bash
      - -lc
      - |
        set -e
        if [ ! -f /workspace/ros_ws/devel/setup.bash ]; then
          echo "ros_ws/devel/setup.bash not found; build ros1_hello and ros1_bringup first" >&2
          exit 2
        fi
        source /workspace/.devcontainer/ros_env.bash
        exec roslaunch ros1_bringup system.launch
```

这里有两个刻意保留的工程边界。

第一，运行态 Container **不在每次开机时自动重新编译 workspace**。源码构建仍然属于开发/CI/部署阶段；运行阶段只消费已经存在的
`ros_ws/devel`。这可以避免设备每次启动都把编译器、源码变化和启动时序混进运行生命周期。

第二，`restart: unless-stopped` 不是“Compose 文件放在磁盘上就会自己创建 Container”。第一次部署仍然需要创建运行态：

```bash
docker compose -f compose.runtime.yaml up -d --build
```

之后，只要 Docker daemon 随 Host 启动，并且这个 Container 没有被人工 stop/remove，Docker 的 restart policy 会在
Docker daemon 重启后恢复该 Container。`roslaunch` 又是 Container 内的主业务进程，因此完整恢复链是：

```text
Host 启动
    -> Docker daemon
    -> ros1-runtime Container
    -> roslaunch ros1_bringup system.launch
    -> Master（若不存在）+ /rosout + 工程 Node
    -> READY 条件逐步成立
    -> 业务进入 RUNNING
```

查看运行态：

```bash
docker compose -f compose.runtime.yaml ps
docker compose -f compose.runtime.yaml logs -f
```

停止运行态：

```bash
docker compose -f compose.runtime.yaml down
```

开发和运行两个 Compose 都使用 Host network，因此不要在两个 Container 中同时启动两套默认端口的 ROS Master。
开发 Container 可以继续保持 `sleep infinity`；需要手工运行 `roscore/roslaunch` 前，应先确认运行态 Master 是否已经占用
默认 `11311`。

### 26.8 Docker、roslaunch、Node READY 分别负责哪一层

实际项目中最容易混淆的是三个不同生命周期：

| 层级 | 当前工程机制 | 解决的问题 | 不解决的问题 |
| --- | --- | --- | --- |
| Host/Container | Docker `restart: unless-stopped` | Host/Docker 重启后恢复运行态 Container | 不判断某个 ROS Node 的业务资源是否 READY |
| ROS 进程组 | `roslaunch` | 启动 Master/core Node/业务 Node，并监控 ROS 进程 | 不应把 `<node>` 书写顺序当成业务初始化依赖 |
| 业务状态 | READY Service / 状态机 | 表达驱动、Service、Action、设备等是否真正可用 | 不负责创建 Docker Container |

当前示例对两个 READY Node 使用 `required="true"`：关键 Node 退出时让 roslaunch 结束整套进程组，再由 Docker
restart policy 负责外层恢复。`required` 仍然只是进程级失败策略，它不表达业务 READY。

如果以后改用 `respawn="true"`，它也只属于第二层：Node 进程退出后让 roslaunch 重新创建进程；它不能证明
重启后的 Node 已经恢复业务 READY。`required` 与 `respawn` 也不能同时用于同一个 `<node>`。

对于更复杂的驱动系统，READY 条件可以进一步升级为明确状态机：

```text
INIT
    -> DEVICE_OPEN
    -> CONFIGURED
    -> COMM_READY
    -> RUNNING
```

控制 Node 依赖的是某个可观察状态，而不是“另一个进程已经存在了几秒”。

### 26.9 从开机到业务 RUNNING，再把整条链走一遍

至此可以把本章的源码链和工程链合并：

```text
Host / Docker
    -> 启动 ros1-runtime

Container command
    -> source ros_env.bash
    -> exec roslaunch ros1_bringup system.launch

roslaunch
    -> 检查 Master
    -> 必要时 create_master_process()
    -> rosmaster --core
    -> 启动 /rosout
    -> 启动 hello_node / hello_listener / ready_client / ready_server

ready_server
    -> 完成模拟驱动初始化
    -> registerService(/demo_driver/ready)

Master
    -> RegistrationManager 记录 Service provider

ready_client
    -> waitForService(/demo_driver/ready)
    -> lookupService / ROSRPC readiness check
    -> dependency READY
    -> subscribe(/chatter)
    -> 进入业务 callback
```

这条链说明工程里“自启动”和“有顺序”不是一件事：

```text
自启动
    -> 谁创建 Container / ROS 进程

依赖顺序
    -> 谁等待哪个明确的 READY 条件
```

只有把这两层分开，系统启动才不会退化成一串不可证明的 `sleep N`。

---

## 27. 阶段 A 到这里真正闭合了什么

阶段 06～11 的学习顺序现在可以闭合成一套完整 ROS1 运行模型：

```text
Node 启动与 ros::start()
    -> Node 自己建立 XML-RPC、TCPROS 等基础设施

Master 控制面
    -> 保存名称和注册关系
    -> 返回或推送“去哪里找对端”

Topic
    -> Master 只完成发现
    -> Node 之间再 requestTopic + TCPROS

Service
    -> Master 返回 ROSRPC endpoint
    -> Client/Server 直接请求响应

Action
    -> actionlib 把长任务语义展开为五类 Topic
    -> Master 仍只按普通 Topic 处理

Parameter
    -> 和 Master 同进程
    -> 通过 XML-RPC 读写与 paramUpdate

CallbackQueue / Spinner
    -> 决定 Node 收到数据以后业务 callback 在何处执行
```

因此进入阶段 B 的“ROS 驱动 Node 软件架构”之前，控制面、数据面、进程、线程、注册发现和上层通信语义已经能够相互对应。后续再设计 CAN/UART/Ethernet Driver 时，就可以明确判断：

```text
哪些状态属于 ROS graph
哪些数据属于 Transport
哪些 callback 属于 ROS 线程
哪些 I/O 应该留在 Driver 自己的线程/状态机
```

这正是阶段 A 从“会写 Node”过渡到“能设计驱动 Node”的边界。

---

## 参考源码与官方资料

源码基线：ROS1 Noetic `noetic-devel`。

- `roslaunch/scripts/roscore`：<https://github.com/ros/ros_comm/blob/noetic-devel/tools/roslaunch/scripts/roscore>
- `roslaunch/__init__.py`：<https://github.com/ros/ros_comm/blob/noetic-devel/tools/roslaunch/src/roslaunch/__init__.py>
- `roslaunch/config.py`：<https://github.com/ros/ros_comm/blob/noetic-devel/tools/roslaunch/src/roslaunch/config.py>
- `roslaunch/resources/roscore.xml`：<https://github.com/ros/ros_comm/blob/noetic-devel/tools/roslaunch/resources/roscore.xml>
- `roslaunch/parent.py`：<https://github.com/ros/ros_comm/blob/noetic-devel/tools/roslaunch/src/roslaunch/parent.py>
- `roslaunch/launch.py`：<https://github.com/ros/ros_comm/blob/noetic-devel/tools/roslaunch/src/roslaunch/launch.py>
- `roslaunch/nodeprocess.py`：<https://github.com/ros/ros_comm/blob/noetic-devel/tools/roslaunch/src/roslaunch/nodeprocess.py>
- `rosmaster/scripts/rosmaster`：<https://github.com/ros/ros_comm/blob/noetic-devel/tools/rosmaster/scripts/rosmaster>
- `rosmaster/main.py`：<https://github.com/ros/ros_comm/blob/noetic-devel/tools/rosmaster/src/rosmaster/main.py>
- `rosmaster/master.py`：<https://github.com/ros/ros_comm/blob/noetic-devel/tools/rosmaster/src/rosmaster/master.py>
- `rosmaster/master_api.py`：<https://github.com/ros/ros_comm/blob/noetic-devel/tools/rosmaster/src/rosmaster/master_api.py>
- `rosmaster/registrations.py`：<https://github.com/ros/ros_comm/blob/noetic-devel/tools/rosmaster/src/rosmaster/registrations.py>
- `rosmaster/paramserver.py`：<https://github.com/ros/ros_comm/blob/noetic-devel/tools/rosmaster/src/rosmaster/paramserver.py>
- `rosgraph/xmlrpc.py`：<https://github.com/ros/ros_comm/blob/noetic-devel/tools/rosgraph/src/rosgraph/xmlrpc.py>
- `rosmaster/CMakeLists.txt`：<https://github.com/ros/ros_comm/blob/noetic-devel/tools/rosmaster/CMakeLists.txt>
- `rosmaster/setup.py`：<https://github.com/ros/ros_comm/blob/noetic-devel/tools/rosmaster/setup.py>
- `roslaunch/CMakeLists.txt`：<https://github.com/ros/ros_comm/blob/noetic-devel/tools/roslaunch/CMakeLists.txt>
- `roslaunch/setup.py`：<https://github.com/ros/ros_comm/blob/noetic-devel/tools/roslaunch/setup.py>
- `actionlib/client/action_client.h`：<https://github.com/ros/actionlib/blob/noetic-devel/actionlib/include/actionlib/client/action_client.h>
- `roslaunch/launch.py`：<https://github.com/ros/ros_comm/blob/noetic-devel/tools/roslaunch/src/roslaunch/launch.py>
- `roscpp/master.cpp`：<https://github.com/ros/ros_comm/blob/noetic-devel/clients/roscpp/src/libros/master.cpp>
- `roscpp/service.cpp`：<https://github.com/ros/ros_comm/blob/noetic-devel/clients/roscpp/src/libros/service.cpp>
- Docker Compose `restart`：<https://docs.docker.com/reference/compose-file/services/#restart>
- `actionlib/server/action_server.h`：<https://github.com/ros/actionlib/blob/noetic-devel/actionlib/include/actionlib/server/action_server.h>
