<meta name="referrer" content="no-referrer" />

# 01：搭建 ROS1 Noetic Docker 开发环境

> 摘要：这一章只做一件事——把 Ubuntu 24.04 Host、Docker、Ubuntu 20.04 Focal 和 ROS1 Noetic 组合成一个稳定、可重复的开发环境。完成后，你应该能解释 Dockerfile、Image、Container、Compose、bind mount、`sleep infinity` 和 `/ros_entrypoint.sh` 各自负责什么，并能手工启动、进入、停止开发 Container。

[TOC]

## 本章目标

完成本章后，你应该能够：

1. 解释为什么 Host 可以是 Ubuntu 24.04，而 ROS1 Noetic 仍运行在 Ubuntu 20.04 Focal userspace 中；
2. 区分 Dockerfile、Image、Container 和 Compose；
3. 理解为什么整个仓库 bind mount 到 Container 的 `/workspace`；
4. 理解为什么开发 Container 使用普通用户 `dev`；
5. 理解 `network_mode: host`、`init: true`、`sleep infinity` 的作用；
6. 用 `docker compose` 手工完成 build、up、ps、exec、logs、stop、down；
7. 知道官方 ROS Image 的 `/ros_entrypoint.sh` 为什么会自动 source `/opt/ros/noetic/setup.bash`。

## 为什么先搭环境，而不是直接写 ROS 代码

ROS1 Noetic 的目标平台是 Ubuntu 20.04 Focal。你的 Host 可以运行更现代的 Ubuntu，但如果直接在 Host 上混装 Noetic、旧版依赖、调试工具和业务代码，后续会出现两个问题：

- Host 系统升级会影响 ROS 开发环境；
- 不同开发者机器之间很难保持一致。

Docker 的价值不是“把 ROS 变成虚拟机”，而是把 ROS 所需要的 userspace 固定下来。

本工程采用：

```text
Ubuntu 24.04 Host
    |
    | Linux Kernel
    v
Docker Container
    |
    +-- Ubuntu 20.04 Focal userspace
    +-- ROS1 Noetic
    +-- GCC / CMake / catkin_tools
    +-- GDB
```

Container 和 Host 共享 Linux Kernel，但文件系统、进程视图、系统库和安装的软件由 Container 自己的 userspace 提供。

因此：

```text
Host 是 Ubuntu 24.04
```

和：

```text
Container 里是 Ubuntu 20.04 + Noetic
```

并不矛盾。

---

## Step 1：先看清工程根目录

Host 上进入仓库：

```bash
cd /home/wdfk/share/ros1-docker
```

这一章最重要的几个文件是：

```text
ros1-docker/
├── Dockerfile
├── compose.yaml
├── .env.example
├── .env                 # 第一次启动时生成，不提交 Git
├── ros_ws/
└── ros_debug_ws/
```

先记住一个路径映射：

```text
Host:
/home/wdfk/share/ros1-docker

Container:
/workspace
```

后续看到：

```text
/workspace/ros_ws
```

就要立刻想到它实际上对应 Host 的：

```text
/home/wdfk/share/ros1-docker/ros_ws
```

### 检查点

```bash
pwd
ls -la
```

确认当前目录就是仓库根目录，并能看到 `Dockerfile` 和 `compose.yaml`。

---

## Step 2：理解 Dockerfile——它负责“镜像怎么做出来”

打开：

```text
Dockerfile
```

第一行是：

```dockerfile
FROM ros:noetic-ros-base-focal
```

它告诉 Docker：不要从一个空 Linux 文件系统开始，而是以官方 ROS1 Noetic `ros-base` Focal Image 为基础。

这个基础 Image 已经提供：

- Ubuntu 20.04 Focal userspace；
- `/opt/ros/noetic`；
- ROS1 基础命令；
- `/ros_entrypoint.sh`；
- `ROS_DISTRO=noetic` 等基础环境。

接下来 Dockerfile 安装：

```text
build-essential
cmake
git
gdb
gdbserver
doxygen
graphviz
python3-catkin-tools
...
```

这些是“开发镜像”需要的工具，而不是每次进入 Container 后再手工安装。

### 为什么把工具写进 Dockerfile

如果你进入 Container 后手工执行：

```bash
sudo apt install gdb
```

然后未来重新 build Image，这个手工改动不会自动成为 Dockerfile 的一部分。

而写进 Dockerfile：

```dockerfile
RUN apt-get update && apt-get install ...
```

意味着：

```text
Dockerfile
   ↓ docker build
Image
   ↓ docker compose up
Container
```

每次从 Dockerfile 重建，都能得到同一组开发工具。

---

## Step 3：为什么创建 `dev` 用户，而不是一直使用 root

Docker Container 默认很容易以 root 运行。如果整个仓库 bind mount 进去后，root 在 Container 中创建：

```text
ros_ws/build/
ros_ws/devel/
```

Host 上可能看到：

```text
root root
```

这会让普通 Host 用户后续删除、编辑或 Git 操作变麻烦。

本工程通过 build args 把 Host UID/GID 传进 Dockerfile：

```yaml
args:
  USER_UID: ${LOCAL_UID:-1000}
  USER_GID: ${LOCAL_GID:-1000}
```

Dockerfile 再创建：

```text
dev
```

用户。

### 在 Host 生成 `.env`

执行：

```bash
printf 'LOCAL_UID=%s\nLOCAL_GID=%s\n' \
    "$(id -u)" "$(id -g)" > .env
```

查看：

```bash
cat .env
```

典型结果：

```text
LOCAL_UID=1000
LOCAL_GID=1000
```

### 检查 Compose 是否读到了变量

```bash
docker compose config --environment
```

重点确认 `LOCAL_UID`、`LOCAL_GID` 有值。

---

## Step 4：理解 `compose.yaml`——它负责“Container 怎么运行”

Dockerfile 与 Compose 不要混为一谈。

```text
Dockerfile
    → Image 怎么构建

compose.yaml
    → Container 怎么启动、挂载、联网、保持运行
```

本工程只有一个服务：

```yaml
services:
  ros1-dev:
```

所以日常命令都会围绕：

```text
ros1-dev
```

展开。

### `image` 与 `container_name`

```yaml
image: ros1-noetic-dev:local
container_name: ros1-dev
```

含义分别是：

```text
Image 名：     ros1-noetic-dev:local
Container 名： ros1-dev
```

Image 更像“模板”；Container 是这个模板的一次运行实例。

---

## Step 5：理解 bind mount——为什么源码不放进 Image

Compose 中：

```yaml
- type: bind
  source: .
  target: /workspace
```

表示：

```text
Host 仓库根目录
      ↓ bind mount
Container /workspace
```

因此你在 Host 修改：

```text
ros_ws/src/ros1_hello/src/hello_node.cpp
```

Container 中立刻看到：

```text
/workspace/ros_ws/src/ros1_hello/src/hello_node.cpp
```

不需要重新 build Image。

### 为什么源码留在 Host

这样可以同时满足：

- Host Git 正常工作；
- Remote-SSH VS Code 直接编辑 Host 文件；
- Container 使用同一份源码编译；
- 删除 Container 不会删除源码；
- Image 只负责工具链，不承载你的工作区历史。

### 验证 bind mount

先启动 Container 后，在 Host：

```bash
touch /home/wdfk/share/ros1-docker/.bind_mount_test
```

Container 中：

```bash
ls -l /workspace/.bind_mount_test
```

能看到同一个文件就说明 bind mount 正常。

验证完删除：

```bash
rm /workspace/.bind_mount_test
```

---

## Step 6：构建并启动开发 Container

Host 执行：

```bash
docker compose up -d --build
```

这里有两个动作：

```text
--build
    先根据 Dockerfile 构建/更新 Image

up -d
    创建并在后台启动 Container
```

查看状态：

```bash
docker compose ps
```

应该能看到：

```text
ros1-dev ... Up
```

如果 `docker compose up` 不加 `-d`，终端会一直占着。这不是卡死，而是 Compose 前台附着到了长期运行的服务。

---

## Step 7：理解 `sleep infinity`——为什么 Container 不立刻退出

Container 的生命周期与主进程有关。

如果主进程结束：

```text
PID 1 退出
    ↓
Container 停止
```

本工程在 Compose 中设置：

```yaml
command: sleep infinity
```

它的意思就是：

```text
一直等待，不主动退出
```

因此开发 Container 可以保持 `Up`，你再通过：

```bash
docker compose exec ros1-dev bash
```

进入它执行 `catkin build`、`roscore`、`rosrun`、`gdb`。

这种模式特别适合“长期开发环境 Container”，因为 ROS 进程不会被隐藏在 Compose 的启动命令里。

---

## Step 8：理解 `init: true`

Compose 中还有：

```yaml
init: true
```

它会在 Container 内放一个轻量 init 进程作为 PID 1。

主要职责：

1. 转发 `SIGINT`、`SIGTERM` 等信号；
2. 回收已经退出但父进程没有 `wait()` 的僵尸子进程。

这对 `roslaunch` 很有意义，因为 `roslaunch` 可能启动多个子进程。

要特别注意：

```text
init: true
```

不是：

```text
启动 systemd
```

也不是：

```text
自动启动 roscore
```

ROS 进程仍然由你自己手工运行。

---

## Step 9：为什么学习阶段使用 `network_mode: host`

ROS1 的 Master、Node XML-RPC URI、TCPROS 连接都涉及网络地址。

如果一开始就使用 Docker bridge/NAT，还需要同时理解：

```text
Container IP
Host IP
端口映射
ROS_IP
ROS_HOSTNAME
```

会增加很多与 ROS 基础无关的变量。

本工程先使用：

```yaml
network_mode: host
```

让 Container 与 Host 使用同一网络 namespace，简化单机学习。

这不是说所有生产 ROS1 系统都应该使用 Host Network，而是学习阶段先减少变量。

---

## Step 10：GDB 为什么需要 `SYS_PTRACE`

Compose 中：

```yaml
cap_add:
  - SYS_PTRACE
```

Linux 调试器会用 `ptrace` 一类机制观察/控制被调试进程。

Docker 默认限制很多能力，因此调试和 attach 场景可能需要：

```text
CAP_SYS_PTRACE
```

本工程没有使用：

```yaml
privileged: true
```

因为它的权限范围过大。

同时：

```yaml
security_opt:
  - seccomp=unconfined
```

用于减少不同 Docker/Kernel 组合下 GDB syscall 过滤造成的兼容性问题。

它属于开发便利配置，会降低隔离强度，不应该不加判断地搬到生产环境。

---

## Step 11：进入 Container，确认 ROS Noetic 环境

Host：

```bash
docker compose exec ros1-dev bash
```

Container：

```bash
whoami
pwd
echo "$ROS_DISTRO"
```

预期类似：

```text
dev
/workspace
noetic
```

再检查：

```bash
ls -ld /workspace
ls -ld /opt/ros/noetic
```

这时要建立一个很重要的概念：

```text
/workspace
    → Host bind mount 进来的工程

/opt/ros/noetic
    → Image 自己的 ROS 安装目录
```

两者来源完全不同。

---

## Step 12：理解 `/ros_entrypoint.sh`

在 Container 中：

```bash
cat /ros_entrypoint.sh
```

核心通常类似：

```bash
source "/opt/ros/$ROS_DISTRO/setup.bash" --
exec "$@"
```

第一句让 ROS 环境进入当前 shell/process 环境，例如：

```text
ROS_DISTRO
ROS_PACKAGE_PATH
PATH
LD_LIBRARY_PATH
PYTHONPATH
```

第二句：

```bash
exec "$@"
```

表示用真正要运行的命令替换 shell 进程。

本工程交互式 bash 又会 source：

```text
/workspace/.devcontainer/ros_env.bash
```

它按照：

```text
/opt/ros/noetic
    ↓
ros_debug_ws/devel（如果已经构建）
    ↓
ros_ws/devel（如果已经构建）
```

形成 overlay 环境。

---

## Step 13：把日常 Docker 命令练熟

先不要依赖插件按钮。建议至少能手工写出：

```bash
# 后台启动
docker compose up -d

# 查看状态
docker compose ps

# 进入 Container
docker compose exec ros1-dev bash

# 查看日志
docker compose logs

# 实时跟随日志
docker compose logs -f

# 停止但保留 Container
docker compose stop

# 重新启动
docker compose start

# 停止并删除 Container
docker compose down
```

理解 `stop` 和 `down` 的区别：

```text
stop
    Container 还在，只是停止

down
    Compose 创建的 Container 和网络被删除
```

但 bind mount 的 Host 仓库不会因此被删除。

---

## 本章常见问题

### 现象 1：`docker compose up` 一直不返回

如果没有 `-d`，Compose 在前台运行是正常的。

改用：

```bash
docker compose up -d
```

再通过：

```bash
docker compose ps
```

确认状态。

### 现象 2：Host 出现 root:root 文件

先确认 `.env`：

```bash
cat .env
id -u
id -g
```

如果改变 UID/GID 后需要重新创建 Image：

```bash
docker compose down
docker compose build --no-cache
docker compose up -d
```

### 现象 3：Container 一启动就退出

先看：

```bash
docker compose ps -a
docker compose logs
```

然后检查 Compose 中是否仍有：

```yaml
command: sleep infinity
```

### 现象 4：Container 中看不到 Host 文件

检查：

```bash
docker inspect ros1-dev
```

重点看 `Mounts`，确认 Host 仓库映射到 `/workspace`。

---

## 本章练习

不看本文，尝试回答：

1. 删除 Container 后，`ros_ws/src` 为什么还在？
2. Dockerfile 和 `compose.yaml` 各自负责什么？
3. `dst=/workspace` 或 `target: /workspace` 的含义是什么？
4. `sleep infinity` 为什么适合开发 Container？
5. `init: true` 和 systemd 有什么区别？
6. 为什么本工程不直接使用 `privileged: true`？

如果这些问题能自己解释，继续下一章。

下一章会从：

```text
/workspace/ros_ws
```

开始，创建 catkin workspace、ROS package 和第一个 C++ Node。

## 参考资料

- [Docker: What is a container?](https://docs.docker.com/get-started/docker-concepts/the-basics/what-is-a-container/)
- [Docker Compose](https://docs.docker.com/compose/)
- [Docker bind mounts](https://docs.docker.com/engine/storage/bind-mounts/)
- [ROS Docker Official Image](https://hub.docker.com/_/ros)
