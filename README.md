# ROS1 Noetic + Docker + VS Code 初学者学习工程

> 面向已经会使用 Linux、C/C++，但刚开始系统学习 ROS1 的开发者。工程把 Ubuntu 24.04 Host、Docker、ROS1 Noetic、catkin、VS Code Remote-SSH、Dev Container、F12 源码阅读和 GDB/F5 调试放在同一套可重复的学习环境中。

ROS1 Noetic 已于 2025-05-31 结束官方支持。本工程适合仍需维护 ROS1 项目、理解遗留系统或学习 ROS1 机制的场景；它不改变 Noetic 已 EOL 的事实。官方说明见 [ROS 1 End of Life](https://www.ros.org/blog/noetic-eol/)。

## 你会完成什么

按照 `docs/` 的顺序学习后，你会亲手完成下面这条链路：

```text
Ubuntu 24.04 Host
        |
        | Docker / Compose
        v
Ubuntu 20.04 Focal + ROS1 Noetic Container
        |
        | catkin build
        v
ros1_hello package
        |
        +--> hello_node      发布 /chatter
        |
        +--> hello_listener  订阅 /chatter
        |
        +--> hello.launch    一次启动两个 Node
        |
        +--> VS Code F12     阅读 roscpp 源码
        |
        +--> VS Code F5/GDB  调试自己的 Node 和 ros::init()
```

工程始终只使用一个仓库根目录：

```text
Host:      /home/wdfk/share/ros1-docker
Container: /workspace
```

`ros_ws` 是自己的业务 workspace；`ros_debug_ws` 用于放 ROS1 上游源码和 Debug overlay。VS Code 根目录始终保持为整个仓库，而不是单独打开 `ros_ws`。

## 工程目录

```text
ros1-docker/
├── README.md
├── Dockerfile
├── compose.yaml
├── .env.example
├── .gitignore
│
├── .devcontainer/
│   ├── devcontainer.json
│   └── ros_env.bash
│
├── .vscode/
│   ├── c_cpp_properties.json
│   ├── extensions.json
│   ├── launch.json
│   ├── settings.json
│   └── tasks.json
│
├── docs/
│   ├── 01_搭建ROS1_Noetic_Docker开发环境.md
│   ├── 02_创建catkin工作空间_Package与第一个Node.md
│   ├── 03_让Node通信_Topic_Parameter与roslaunch.md
│   ├── 04_使用VSCode_RemoteSSH与DevContainer.md
│   ├── 05_阅读roscpp源码并使用F12_F5调试.md
│
├── ros_ws/
│   └── src/
│       └── ros1_hello/
│           ├── CMakeLists.txt
│           ├── package.xml
│           ├── README.md
│           ├── launch/hello.launch
│           └── src/
│               ├── hello_node.cpp
│               └── hello_listener.cpp
│
└── ros_debug_ws/
    └── src/.gitkeep
```

## 推荐学习顺序

教程采用“目标 → 动机 → 分步操作 → 检查点 → 原理 → 问题排查 → 练习”的节奏。不要一次把所有命令复制完；每完成一个检查点，再进入下一步。

写作节奏参考 ROS-Industrial Training 的练习式组织方式：先说明为什么做，再逐步创建、构建、运行和验证，最后给出练习或挑战；本文档内容仍针对 ROS1 Noetic、Docker 和本仓库结构重新编写。参考：[Creating Packages and Nodes](https://industrial-training-master.readthedocs.io/en/humble/_source/session1/3-Creating-a-ROS-Package-and-Node.html)。

| 顺序 | 文章 | 学完后应该会什么 |
| --- | --- | --- |
| 1 | `01_搭建ROS1_Noetic_Docker开发环境.md` | 理解 Image/Container/Bind Mount，并启动稳定的 ROS1 开发 Container |
| 2 | `02_创建catkin工作空间_Package与第一个Node.md` | 理解 workspace/package/CMake/catkin，并构建第一个 ROS1 C++ Node |
| 3 | `03_让Node通信_Topic_Parameter与roslaunch.md` | 会运行 Node、查看 Topic、使用参数和 roslaunch |
| 4 | `04_使用VSCode_RemoteSSH与DevContainer.md` | 理解 Host IDE 与 Container IDE 的边界，建立稳定的 IntelliSense/F12 工作流 |
| 5 | `05_阅读roscpp源码并使用F12_F5调试.md` | 能阅读 `ros::init()` 源码，并用 GDB/F5 进入 roscpp |

## 第一次启动

以下命令在 Ubuntu 24.04 Host 执行。

### 1. 进入仓库

```bash
cd /home/wdfk/share/ros1-docker
```

如果你的仓库放在其它目录，也可以使用自己的路径。后续 Container 内路径仍固定为 `/workspace`。

### 2. 写入 Host UID/GID

```bash
printf 'LOCAL_UID=%s\nLOCAL_GID=%s\n' \
    "$(id -u)" "$(id -g)" > .env
```

查看：

```bash
cat .env
```

这样 Dockerfile 创建的 `dev` 用户会尽量与 Host 用户保持相同 UID/GID，减少 bind mount 文件变成 `root:root` 的问题。

### 3. 构建并启动 Container

```bash
docker compose up -d --build
```

检查：

```bash
docker compose ps
```

应该看到 `ros1-dev` 处于 `Up` 状态。

### 4. 进入 ROS Container

```bash
docker compose exec ros1-dev bash
```

检查环境：

```bash
echo "$ROS_DISTRO"
g++ --version
gdb --version
```

`ROS_DISTRO` 应输出：

```text
noetic
```

### 5. 初始化并构建业务 workspace

在 Container 中执行：

```bash
cd /workspace/ros_ws
source /opt/ros/noetic/setup.bash
catkin init
catkin config \
    --merge-devel \
    --extend /opt/ros/noetic \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
catkin build ros1_hello
```

检查：

```bash
catkin config
```

应看到：

```text
Devel Space Layout:          merged
```

再检查可执行文件：

```bash
ls -l /workspace/ros_ws/devel/lib/ros1_hello/
```

应至少看到：

```text
hello_node
hello_listener
```

### 6. 手工运行第一个 ROS1 程序

打开三个 Container 终端。

Terminal A：

```bash
roscore
```

Terminal B：

```bash
source /workspace/ros_ws/devel/setup.bash
rosrun ros1_hello hello_node
```

Terminal C：

```bash
source /workspace/ros_ws/devel/setup.bash
rosrun ros1_hello hello_listener
```

再开一个终端查看 Topic：

```bash
rostopic echo /chatter
```

如果能持续看到字符串消息，最小 ROS1 Publisher/Subscriber 链路已经跑通。

## 日常 Docker 命令

这些命令建议先手敲熟悉，不需要额外脚本包装：

```bash
# 创建/启动 Container，后台运行
docker compose up -d

# 查看服务状态
docker compose ps

# 查看日志
docker compose logs

# 持续跟随日志
docker compose logs -f

# 进入 ROS Container
docker compose exec ros1-dev bash

# 停止但保留 Container
docker compose stop

# 再次启动同一个 Container
docker compose start

# 停止并删除 Container/Compose 网络
docker compose down
```

`docker compose down` 不会删除 Host 源码。`ros_ws`、`ros_debug_ws` 和整个仓库都在 Host bind mount 上。

## VS Code 的两个工作位置

本工程有两个明确用途不同的 VS Code 模式。

### Remote-SSH Host 窗口

日常主窗口：

```text
Windows VS Code
    ↓ Remote-SSH
Ubuntu 24.04 Host
```

适合：

- 编辑 Host 仓库文件；
- Git；
- Docker/Compose 管理；
- 普通 Host Terminal；
- `ROS1 Container Bash` Terminal；
- Host F5，通过 `pipeTransport` 调用 Container 内 GDB。

### Reopen in Container

需要精确看到 Container 文件系统和 ROS 安装环境时：

```text
VS Code 左下角 Remote 指示器
→ Reopen in Container
```

此时 VS Code Extension Host 运行在 `ros1-dev` Container 内，能够直接访问：

```text
/workspace
/opt/ros/noetic
/usr/include
/usr/lib
```

它更适合深入 ROS C++ IntelliSense、查看 Container 系统头文件和直接使用 GDB。

## C++ IntelliSense 的基本原则

`.vscode/c_cpp_properties.json` 是本工程唯一主要 C/C++ IntelliSense 配置。

重要路径分成三类：

```text
源码 header
    ros/ros.h
    ros/time.h
    ros/console.h
    → 来自 ros_debug_ws/src 中的上游仓库

catkin 生成 header
    ros/common.h
    std_msgs/String.h
    rosgraph_msgs/Clock.h
    → 来自 ros_debug_ws/devel/include

Host 系统 header
    boost/shared_ptr.hpp
    boost/function.hpp
    → 由 Host /usr/bin/g++ 的系统 include 路径提供
```

Remote-SSH Host 下，为了让：

```cpp
#include <ros/ros.h>
```

能够稳定 F12 到 roscpp 源码，本工程显式给出 roscpp 的 include 根：

```text
${workspaceFolder}/ros_debug_ws/src/ros_comm/clients/roscpp/include
```

同时保留：

```text
${workspaceFolder}/ros_debug_ws/src/**
```

用于广泛源码索引。两者用途不同。

## 准备 roscpp 源码阅读环境

上游 ROS 仓库不直接打包进本仓库。需要源码阅读/F5 进入 roscpp 时，在 Host 执行：

```bash
cd /home/wdfk/share/ros1-docker/ros_debug_ws/src

git clone --branch noetic-devel --single-branch \
    https://github.com/ros/ros_comm.git

git clone --branch noetic-devel --single-branch \
    https://github.com/ros/roscpp_core.git

git clone --branch noetic-devel --single-branch \
    https://github.com/ros/rosconsole.git

git clone --branch noetic-devel --single-branch \
    https://github.com/ros/std_msgs.git

git clone --branch noetic-devel --single-branch \
    https://github.com/ros/ros_comm_msgs.git
```

然后在 Container 中：

```bash
cd /workspace/ros_debug_ws
source /opt/ros/noetic/setup.bash
catkin init
catkin config \
    --merge-devel \
    --extend /opt/ros/noetic \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
catkin build roscpp std_msgs rosconsole rosgraph_msgs
```

完整步骤和验证方法见第 05 篇。

## 遇到问题时先做什么

不要先连续修改配置。先收集证据。

C/C++ IntelliSense：

```text
Ctrl+Shift+P
→ C/C++: Log Diagnostics
```

catkin：

```bash
catkin config
catkin list --unformatted
```

文件是否真的存在：

```bash
test -f <path> && echo OK || echo MISSING
ls -l <path>
```

动态库到底加载哪一份：

```bash
ldd /workspace/ros_ws/devel/lib/ros1_hello/hello_node
```


## 参考资料

- [ROS 1 Noetic EOL](https://www.ros.org/blog/noetic-eol/)
- [catkin_tools documentation](https://catkin-tools.readthedocs.io/)
- [Docker Compose documentation](https://docs.docker.com/compose/)
- [VS Code C++ configuration](https://code.visualstudio.com/docs/cpp/customize-cpp-settings)
- [VS Code Dev Containers](https://code.visualstudio.com/docs/devcontainers/containers)
- [ros_comm](https://github.com/ros/ros_comm)
- [roscpp_core](https://github.com/ros/roscpp_core)
