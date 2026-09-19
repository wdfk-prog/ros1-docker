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
├── compose.runtime.yaml
├── Doxyfile
├── .env.example
├── .gitignore
│
├── .github/
│   └── workflows/
│       ├── ci.yml
│       ├── pages-doxygen.yml
│       └── publish-image.yml
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
│   ├── doxygen-mainpage.md
│   ├── 01_搭建ROS1_Noetic_Docker开发环境.md
│   ├── 02_创建catkin工作空间_Package与第一个Node.md
│   ├── 03_让Node通信_Topic_Parameter与roslaunch.md
│   ├── 04_使用VSCode_RemoteSSH与DevContainer.md
│   ├── 05_阅读roscpp源码并使用F12_F5调试.md
│   ├── ROS教程11.5：roscore源码阅读——从启动脚本到Master注册表与控制面.md
│   └── 12_ROS消息与驱动数据契约_差速底盘Driver.md
│
├── ros_ws/
│   └── src/
│       ├── ros1_hello/
│       │   ├── CMakeLists.txt
│       │   ├── package.xml
│       │   ├── README.md
│       │   ├── msg/HelloStatus.msg
│       │   ├── launch/
│       │   │   ├── hello.launch
│       │   │   └── custom_msg.launch
│       │   └── src/
│       │       ├── hello_node.cpp
│       │       ├── hello_listener.cpp
│       │       ├── custom_msg_publisher.cpp
│       │       ├── custom_msg_subscriber.cpp
│       │       ├── ready_server.cpp
│       │       └── ready_client.cpp
│       │
│       ├── ros1_bringup/
│       │   ├── CMakeLists.txt
│       │   ├── package.xml
│       │   ├── README.md
│       │   └── launch/system.launch
│       │
│       └── ros1_driver_lab/
│           ├── CMakeLists.txt
│           ├── package.xml
│           ├── README.md
│           ├── config/chassis_lab.yaml
│           ├── launch/chassis_lab.launch
│           └── src/chassis_driver_node.cpp
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
| 3 | `03_让Node通信_Topic_Parameter与roslaunch.md` | 会运行 Node、查看 Topic、定义自定义 `.msg`、使用参数和 roslaunch |
| 4 | `04_使用VSCode_RemoteSSH与DevContainer.md` | 理解 Host IDE 与 Container IDE 的边界，建立稳定的 IntelliSense/F12 工作流 |
| 5 | `05_阅读roscpp源码并使用F12_F5调试.md` | 能阅读 `ros::init()` 源码，并用 GDB/F5 进入 roscpp |
| 12 | `12_ROS消息与驱动数据契约_差速底盘Driver.md` | 理解 Twist、JointState、Imu、Odometry、covariance，并完成差速底盘正/逆运动学 |

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

### 整机运行 Compose

`compose.yaml` 保持开发用途：Container 长期执行 `sleep infinity`，由开发者手工运行 ROS 命令。
阶段 A 最后的工程实验另外提供 `compose.runtime.yaml`，让 Container 直接执行顶层 `roslaunch`。

先在开发 Container 中完成一次 workspace 构建：

```bash
cd /workspace/ros_ws
catkin build ros1_hello ros1_bringup
```

回到 Host 后启动运行态：

```bash
# 首次创建/启动运行态 Container
# --build 仅构建 Docker Image，不替代上面的 catkin build。
docker compose -f compose.runtime.yaml up -d --build

# 查看整套 ROS 系统日志
docker compose -f compose.runtime.yaml logs -f

# 查看运行状态
docker compose -f compose.runtime.yaml ps

# 停止并删除运行态 Container
docker compose -f compose.runtime.yaml down
```

`compose.runtime.yaml` 使用 `restart: unless-stopped`。运行态 Container 首次创建后，只要 Docker daemon 随系统启动、
Container 没有被人工停止或删除，Host 重启后 Docker 会按 restart policy 恢复该 Container。

完整启动链和 READY 依赖实验见 `docs/ROS教程11.5：roscore源码阅读——从启动脚本到Master注册表与控制面.md`。

## CI/CD、在线文档与 GHCR 镜像

这个仓库除了本地开发环境，还包含三条 GitHub Actions 自动化流程。初学时可以把它们理解成：

```text
提交代码
   |
   +--> CI：重新编译 ROS1 示例，确认代码还能构建
   |
   +--> Docs CD：重新生成 Doxygen 网站并发布到 GitHub Pages
   |
   +--> Image CD：重新构建 Docker 镜像并发布到 GHCR
```

对应文件如下：

| 工作流 | 作用 | 主要触发条件 |
| --- | --- | --- |
| `.github/workflows/ci.yml` | 在 ROS1 Noetic Docker 环境中执行真实 `catkin build ros1_hello` | Pull Request、`main` 相关代码更新、手工运行 |
| `.github/workflows/pages-doxygen.yml` | 把 Markdown、源码和 Doxygen 注释生成 HTML，并发布到 GitHub Pages | `main` 的文档/源码更新、手工运行 |
| `.github/workflows/publish-image.yml` | 构建开发镜像并推送到 GitHub Container Registry | `main`、`v*` Release tag、手工运行 |

### ROS1 CI 实际检查什么

CI 不会在 GitHub Runner 的 Host 系统中直接安装 ROS1 Noetic。它先使用仓库自己的 `Dockerfile` 构建 Ubuntu 20.04 + ROS1 Noetic 镜像，再把仓库映射到容器 `/workspace` 中执行：

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

最后还会检查下面两个可执行文件是否真的生成：

```text
ros_ws/devel/lib/ros1_hello/hello_node
ros_ws/devel/lib/ros1_hello/hello_listener
```

因此，GitHub 上绿色的 ROS1 CI 表示“这一 commit 已经在仓库定义的 ROS1 Docker 环境中完成实际编译”，而不只是做了 YAML 或源码静态检查。

### Doxygen 在线文档

文档站目标地址：

```text
https://wdfk-prog.github.io/ros1-docker/
```

Doxygen 同时读取：

```text
docs/                       Markdown 学习文章
ros_ws/src/ros1_hello/      ROS1 示例源码和代码注释
```

网站中可以查看 01~05 学习文档，也可以进入 File List、Globals 和源码浏览页面查看 C/C++ 文件与 Doxygen 注释。

第一次启用 GitHub Pages 时，需要在仓库网页手工设置一次：

```text
Settings
→ Pages
→ Build and deployment
→ Source
→ GitHub Actions
```

完成后，`.github/workflows/pages-doxygen.yml` 会负责后续自动生成和部署，不需要提交 `build/doxygen/html` 生成物。

如果想在本机开发 Container 中提前生成一次文档，可以执行：

```bash
docker compose exec ros1-dev bash
cd /workspace
rm -rf build/doxygen
mkdir -p build/doxygen
doxygen Doxyfile
```

生成首页位于：

```text
/workspace/build/doxygen/html/index.html
```

`build/doxygen/` 已加入 `.gitignore`，因为它属于可重复生成的临时产物。

### GHCR Docker 镜像

镜像发布地址：

```text
ghcr.io/wdfk-prog/ros1-docker
```

`main` 分支发布时会生成：

```text
ghcr.io/wdfk-prog/ros1-docker:main
ghcr.io/wdfk-prog/ros1-docker:sha-<commit前7位>
```

例如以后创建正式 tag：

```bash
git tag v1.0.0
git push origin v1.0.0
```

工作流会发布：

```text
ghcr.io/wdfk-prog/ros1-docker:v1.0.0
ghcr.io/wdfk-prog/ros1-docker:1.0.0
ghcr.io/wdfk-prog/ros1-docker:latest
```

GitHub Container Registry 第一次发布 Container package 时，默认可见性是 **Private**。因此工作流第一次成功 push 镜像后，还要根据你的使用方式选择：

- 保持 Private：拉取镜像前需要先登录 GHCR，并使用具有 package 读取权限的 GitHub 凭据；
- 改成 Public：任何人都可以匿名 `docker pull`，更适合公开学习仓库。

如果希望公开镜像，需要在 GitHub 网页手工执行一次：

```text
个人主页
→ Packages
→ ros1-docker
→ Package settings
→ Danger Zone
→ Change visibility
→ Public
```

GitHub 会明确提示：**package 一旦改成 Public，就不能再改回 Private**。确认确实希望公开之后再执行这个操作。

当 package 已设为 Public，或者当前 Docker 客户端已经登录并拥有读取权限时，可以拉取：

```bash
docker pull ghcr.io/wdfk-prog/ros1-docker:main
```

需要注意：GHCR 中预构建镜像的 `dev` 用户固定使用 UID/GID `1000:1000`，这样镜像 tag 的内容保持稳定。如果你的 Host UID/GID 不是 1000，并且要把本机源码 bind mount 进去开发，仍推荐使用本仓库原来的方式：

```bash
printf 'LOCAL_UID=%s\nLOCAL_GID=%s\n' \
    "$(id -u)" "$(id -g)" > .env

docker compose up -d --build
```

这样会针对你的 Host UID/GID 本地重建开发镜像，避免 bind mount 文件权限问题。

### CI 与 CD 的区别

在本仓库中可以先这样理解：

```text
CI
= 验证这次改动是否还能正确构建
= 不发布正式产物

CD
= 在验证之外，把可使用的结果发布出去
= GitHub Pages 文档站 / GHCR Docker 镜像
```

Pull Request 只执行 ROS1 CI，不会向 GHCR 发布镜像，也不会把未合并代码部署为正式文档站。

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
