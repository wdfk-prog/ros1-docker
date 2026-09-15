<meta name="referrer" content="no-referrer" />

# 04：使用 VS Code Remote-SSH 与 Dev Container 建立 ROS1 开发工作流

> 摘要：这一章不把 VS Code 当成“装几个插件就结束”。我们会先确定 IDE 到底运行在哪一层，再建立两套互补工作流：Remote-SSH Host 负责日常编辑、Git、Docker 和 Host→Container GDB；Dev Container 负责最准确的 ROS1 C++ IntelliSense、`/opt/ros/noetic` 浏览和 Container 内调试。

[TOC]

## 本章目标

完成本章后，你应该能够：

1. 解释 Windows VS Code、Remote-SSH Host、Docker Container 三层关系；
2. 知道 VS Code Extension Host 在不同模式下运行在哪里；
3. 会使用 `ROS1 Container Bash` Terminal Profile；
4. 会使用 Container Tools 查看 Container 文件；
5. 会 `Reopen in Container`，并理解它与普通 `docker exec` 的区别；
6. 理解为什么 VS Code 根目录保持 `/workspace`；
7. 理解 `.vscode/c_cpp_properties.json` 中各类 include path 的作用；
8. 会使用 `C/C++: Log Diagnostics` 判断 IntelliSense 实际加载了什么。

## 为什么先确定“VS Code 在哪一层”

当前环境不是一个单层 Linux 桌面，而是：

```text
Windows
   |
   | VS Code Remote-SSH
   v
Ubuntu 24.04 Host
   |
   | Docker
   v
ros1-dev Container
Ubuntu 20.04 + ROS1 Noetic
```

如果不先分清这三层，就很容易产生下面的误解：

```text
“Container 里明明有 /opt/ros/noetic，为什么 F12 找不到？”

“Host F5 为什么能调 Container 里的程序？”

“普通 VS Code Terminal 和 Container Terminal 为什么路径不一样？”
```

答案都取决于：

```text
当前 VS Code Extension Host 在哪里运行？
```

---

## Step 1：把整个仓库作为 Remote-SSH Workspace 打开

推荐主工作方式：

```text
Windows VS Code
    ↓ Remote-SSH
Ubuntu 24.04 Host
    ↓ 打开
/home/wdfk/share/ros1-docker
```

不要只打开：

```text
/home/wdfk/share/ros1-docker/ros_ws
```

因为整个仓库里还包括：

```text
Dockerfile
compose.yaml
.devcontainer/
.vscode/
ros_debug_ws/
```

它们都是完整开发环境的一部分。

### 检查点

VS Code Explorer 顶层应该能同时看到：

```text
.devcontainer
.vscode
Dockerfile
compose.yaml
docs
ros_ws
ros_debug_ws
```

---

## Step 2：安装 Host 侧基础扩展

仓库的：

```text
.vscode/extensions.json
```

推荐：

```text
ms-vscode.cpptools
ms-azuretools.vscode-containers
docker.docker
ms-vscode-remote.remote-containers
```

它们分别用于：

```text
C/C++
    → IntelliSense、F12、cppdbg/GDB

Container Tools
    → Container/Image/Volume/Logs/Files

Docker DX
    → Dockerfile / Compose 编辑辅助

Dev Containers
    → Reopen in Container / Attach
```

ROS1 专用扩展放在 Dev Container 配置中：

```text
Ranch-Hand-Robotics.rde-ros-1
```

这样 ROS 插件真正运行在有 Noetic 环境的 Container Extension Host 中，而不是强迫 Ubuntu24 Host 模拟完整 Noetic 安装环境。

---

## Step 3：普通 Host Terminal 与 ROS Container Terminal 不要混

Remote-SSH Host 窗口里新建普通终端：

```bash
pwd
```

通常得到：

```text
/home/wdfk/share/ros1-docker
```

这是 Host shell。

仓库在 `.vscode/settings.json` 中定义了另一个 Terminal Profile：

```text
ROS1 Container Bash
```

在 VS Code：

```text
Terminal
→ New Terminal
→ 右侧下拉
→ ROS1 Container Bash
```

再：

```bash
pwd
```

应该是：

```text
/workspace
```

再检查：

```bash
echo "$ROS_DISTRO"
```

应该是：

```text
noetic
```

### 这个 Profile 本质上是什么

配置核心等价于：

```bash
docker compose \
    --project-directory <workspace> \
    exec ros1-dev bash
```

它不是 ROS 插件，也不是新的 shell 类型，只是 VS Code 帮你固定了一条 `docker compose exec` 命令。

### 为什么使用 `--project-directory`

这样 Compose 始终从整个仓库寻找：

```text
compose.yaml
```

不依赖终端当前 `cwd`。

因此你即使在：

```text
ros_ws/src/ros1_hello
```

打开这个 Profile，也不会因为当前目录变化而找不到 Compose 项目。

---

## Step 4：用 Container Tools 看“Container 里真实存在的文件”

Container Tools 适合回答：

```text
ros1-dev 是否运行？

Image 是哪个？

Container 日志是什么？

/opt/ros/noetic 里到底有哪些文件？
```

例如 Host Explorer 并不能直接把：

```text
/opt/ros/noetic
```

当作 Host 目录浏览，因为它属于 Container 文件系统。

此时可以用 Container Tools 的 Files 功能查看：

```text
/opt/ros/noetic/include
/opt/ros/noetic/lib
/ros_entrypoint.sh
/usr/include
```

这和 bind mount 的 `/workspace` 不一样。

```text
/workspace
    → Host 文件映射进 Container

/opt/ros/noetic
    → Container Image 自己的文件
```

这个区别对后面 F12 非常重要。

---

## Step 5：理解 Reopen in Container

点击 VS Code 左下角 Remote 指示器：

```text
Reopen in Container
```

Dev Containers 会读取：

```text
.devcontainer/devcontainer.json
```

核心配置：

```jsonc
"dockerComposeFile": "../compose.yaml",
"service": "ros1-dev",
"workspaceFolder": "/workspace",
"remoteUser": "dev"
```

这不是重新复制一份代码。

因为 `/workspace` 已经是 Host 仓库 bind mount 进来的同一份文件。

变化的是：

```text
VS Code UI 仍在 Windows

但 Extension Host、Terminal、cpptools、ROS 插件
进入 ros1-dev Container
```

因此此时 IntelliSense 可以直接访问：

```text
/opt/ros/noetic/include
/usr/include
/usr/include/c++/9
```

这就是 Dev Container 模式比 Host 模式更接近真实编译环境的原因。

---

## Step 6：为什么 Dev Container 根目录仍然是 `/workspace`

`devcontainer.json` 中：

```jsonc
"workspaceFolder": "/workspace"
```

不要改成：

```text
/workspace/ros_ws
```

因为你需要同时看到：

```text
/workspace/ros_ws
/workspace/ros_debug_ws
/workspace/.vscode
/workspace/.devcontainer
/workspace/compose.yaml
```

后面阅读 `roscpp` 源码时，业务代码和上游源码会同时参与导航和调试。

代价是：某些 ROS 插件只会把“VS Code 根目录”当作 catkin workspace 检测目标。

当前根目录是：

```text
/workspace
```

而真正的 catkin workspace 是：

```text
/workspace/ros_ws
```

所以插件可能显示：

```text
Build tool NOT detected
```

这不等于 ROS 环境坏了。

本工程对构建使用显式 Task：

```text
ROS1 [DEV CONTAINER]: Build ros1_hello
```

让任务明确 `cd /workspace/ros_ws` 再执行：

```bash
catkin build ros1_hello
```

---

## Step 7：理解 `.devcontainer/ros_env.bash`

文件内容很短：

```bash
source /opt/ros/noetic/setup.bash

if [ -f /workspace/ros_debug_ws/devel/setup.bash ]; then
    source /workspace/ros_debug_ws/devel/setup.bash
fi

if [ -f /workspace/ros_ws/devel/setup.bash ]; then
    source /workspace/ros_ws/devel/setup.bash
fi
```

顺序非常重要：

```text
Noetic underlay
    ↓
ros_debug_ws overlay
    ↓
ros_ws overlay
```

最终业务 workspace 在最上层。

为什么不直接把 ROS 插件配置成：

```text
/workspace/ros_ws/devel/setup.bash
```

因为第一次打开工程时 `ros_ws` 可能还没构建，`devel/setup.bash` 还不存在。

而 `ros_env.bash` 可以：

```text
有 overlay 就 source
没有就跳过
```

所以它是稳定入口。

---

## Step 8：先理解 Host IntelliSense 的边界

回到 Remote-SSH Host 窗口。

此时 `ms-vscode.cpptools` 运行在：

```text
Ubuntu 24.04 Host
```

而不是 Container。

因此：

```text
Host 能直接访问：
/home/wdfk/share/ros1-docker/...
/usr/include
/usr/bin/g++

Host 不能直接访问：
/opt/ros/noetic
Container /usr/include/c++/9
```

这就是为什么 Host IntelliSense 需要：

```text
ros_debug_ws
```

把需要阅读的 ROS 上游源码放到 Host 可见的 bind-mount 目录中。

---

## Step 9：阅读 `c_cpp_properties.json`

打开：

```text
.vscode/c_cpp_properties.json
```

先看编译器：

```jsonc
"compilerPath": "/usr/bin/g++"
```

在 Remote-SSH Host 中，这个路径指的是：

```text
Ubuntu24 Host 的 g++
```

C/C++ 扩展会询问它，并自动得到 Host 系统 include：

```text
/usr/include/c++/...
/usr/include/x86_64-linux-gnu
/usr/include
```

所以不要手工堆：

```jsonc
"/usr/include/**",
"/usr/include/x86_64-linux-gnu/**"
```

如果 Boost 已安装，Host g++ 的系统 include 就应该能找到它。

---

## Step 10：理解四组 ROS include path

配置中最重要的是：

```jsonc
"includePath": [
    "${workspaceFolder}/ros_ws/src/**",

    "${workspaceFolder}/ros_ws/devel/include",
    "${workspaceFolder}/ros_ws/devel/include/**",

    "${workspaceFolder}/ros_debug_ws/src/ros_comm/clients/roscpp/include",
    "${workspaceFolder}/ros_debug_ws/src/**",

    "${workspaceFolder}/ros_debug_ws/devel/include",
    "${workspaceFolder}/ros_debug_ws/devel/include/**",

    "/opt/ros/noetic/include"
]
```

按职责理解，不要死记路径。

### 10.1 `ros_ws/src/**`

自己的 package 源码。

### 10.2 `ros_ws/devel/include` 与 `/**`

自己的 workspace 生成 header，例如以后自己定义 message 后产生的：

```text
my_pkg/MyMessage.h
```

### 10.3 roscpp 精确 include root

```text
ros_debug_ws/src/ros_comm/clients/roscpp/include
```

这是一个标准 C/C++ include root。

因为：

```cpp
#include <ros/ros.h>
```

要求编译器/IntelliSense 能找到：

```text
<某个 include root>/ros/ros.h
```

当前源码实际位置：

```text
ros_comm/clients/roscpp/include/ros/ros.h
```

所以 include root 应该是它上一级：

```text
ros_comm/clients/roscpp/include
```

### 10.4 `ros_debug_ws/src/**`

用于广泛递归索引其它上游源码：

```text
roscpp_core
rosconsole
ros_comm_msgs
...
```

但要注意：

```text
递归索引范围
```

和：

```text
C/C++ 的精确 include root
```

不是完全相同的概念。

在 Remote-SSH Host 下，`src/**` 可以帮助 cpptools“看到”很多文件，但 `<ros/ros.h>` 的 F12 导航仍需要 roscpp 真实 include root 才稳定。

### 10.5 `ros_debug_ws/devel/include`

用于 catkin 生成 header：

```text
ros/common.h
std_msgs/String.h
rosgraph_msgs/Clock.h
```

第 05、06 篇会专门讲这三类 header 的差异。

---

## Step 11：第一次做 F12 前，先准备 roscpp 源码

如果：

```text
ros_debug_ws/src/ros_comm
```

还不存在，在 Host：

```bash
cd /home/wdfk/share/ros1-docker/ros_debug_ws/src

git clone --branch noetic-devel --single-branch \
    https://github.com/ros/ros_comm.git
```

然后确认：

```bash
test -f \
    ros_comm/clients/roscpp/include/ros/ros.h \
    && echo "ros.h OK"
```

再回 `hello_node.cpp`：

```cpp
#include <ros/ros.h>
```

把光标放到：

```text
ros/ros.h
```

按 F12。

目标：

```text
/home/wdfk/share/ros1-docker/
ros_debug_ws/src/ros_comm/clients/roscpp/include/ros/ros.h
```

这一步只证明：

```text
Header 导航已经能到源码
```

它还不证明运行时真正加载了自己编译的 Debug `libroscpp.so`。后者属于第 05 篇。

---

## Step 12：使用 `C/C++: Log Diagnostics` 看事实

如果 F12 或红线不正常，不要第一反应继续添加 includePath。

执行：

```text
Ctrl+Shift+P
→ C/C++: Log Diagnostics
```

重点看：

```text
Current Configuration
Compiler Path
Language
Include Paths
System Include Paths
IntelliSense Mode
```

例如你应该能确认：

```text
Compiler Path: /usr/bin/g++
Language: C++
```

以及：

```text
include (recursive): .../ros_debug_ws/src
include: .../ros_comm/clients/roscpp/include
```

如果配置文件里写了路径，但 Diagnostics 里没有，就说明：

```text
当前 Translation Unit 没有真正使用那套配置
```

这比继续猜路径更可靠。

---

## Step 13：理解 Reset、Rescan 和 Reload 的区别

修改 C/C++ 配置或重新生成大量 header 后，可以按顺序：

```text
C/C++: Reset IntelliSense Database
```

清理 cpptools 数据库。

然后：

```text
C/C++: Rescan Workspace
```

重新扫描 workspace。

最后必要时：

```text
Developer: Reload Window
```

重新加载 VS Code 窗口和扩展。

不要把这三个动作当成“万能修复”。它们适合在路径已经正确、但缓存状态可能过期时使用。

---

## Step 14：Remote-SSH Host F5 的总体结构

本工程的 Host Debug 配置使用：

```text
VS Code cppdbg
    运行在 Host
        ↓ pipeTransport
Docker CLI
        ↓ docker exec
Container /usr/bin/gdb
        ↓
Container 中的 hello_node
```

所以 Host 不需要自己运行 Noetic 的 GDB target。

`.vscode/launch.json` 中：

```jsonc
"pipeProgram": "docker"
```

让 cppdbg 调用 Docker。

```jsonc
"debuggerPath": "/usr/bin/gdb"
```

这个路径是 Container 里的 GDB。

```jsonc
"sourceFileMap": {
    "/workspace": "${workspaceFolder}"
}
```

负责把 DWARF 中记录的 Container 源码路径映射回 Host 文件路径。

第 05 篇会实际使用这条链路。

---

## Step 15：Reopen in Container 后 F5 为什么更简单

如果已经 Reopen in Container：

```text
cpptools
GDB
ELF
源码路径
```

都在同一个 Container namespace。

因此 Debug 配置可以直接：

```jsonc
"miDebuggerPath": "/usr/bin/gdb"
```

不需要：

```text
pipeTransport
sourceFileMap /workspace → Host
```

这也是为什么本工程保留两套 launch config：

```text
[HOST]
    → 日常 Remote-SSH 主窗口使用

[DEV CONTAINER]
    → 深入 Container 内调试时使用
```

不要把两套字段混在同一个配置里。

---

## 本章常见问题

### VS Code 能搜索到 `ros.h`，但 `#include <ros/ros.h>` F12 不跳

“文件搜索能找到”不等于“它被识别成正确 include root”。

确认配置中存在：

```text
${workspaceFolder}/ros_debug_ws/src/ros_comm/clients/roscpp/include
```

再用：

```text
C/C++: Log Diagnostics
```

确认当前 TU 真的加载了它。

### Host 上 `/opt/ros/noetic/include` 不存在

这是正常的。

Remote-SSH Host 模式下主要依靠：

```text
ros_debug_ws/src
ros_debug_ws/devel/include
```

要直接访问 Container `/opt/ros/noetic`，使用：

```text
Reopen in Container
```

或 Container Tools Files。

### ROS1 插件能用环境，但检测不到 catkin workspace

当前 VS Code 根目录是：

```text
/workspace
```

真正 catkin workspace 是：

```text
/workspace/ros_ws
```

所以插件根目录自动检测不一定成功。

使用仓库提供的 Task：

```text
ROS1 [DEV CONTAINER]: Build ros1_hello
```

而不是为了迎合插件，把整个 VS Code 根目录改成 `/workspace/ros_ws`。

---

## 本章练习

1. 在 Remote-SSH Host 普通 Terminal 和 `ROS1 Container Bash` 中分别执行 `pwd`；
2. 在 Container Tools 中找到 `/ros_entrypoint.sh`；
3. Reopen in Container 后执行 `which g++`、`echo $ROS_DISTRO`；
4. 回 Remote-SSH Host，对 `<ros/ros.h>` 做 F12；
5. 执行 `C/C++: Log Diagnostics`，找出 Host g++ 的 system include path；
6. 解释为什么 `ros_debug_ws/src/**` 和 roscpp 精确 include root 不能简单看成同一件事。

下一章会继续沿着 `hello_node.cpp` 往下走：

```text
F12 到 ros/ros.h
    ↓
找到 ros::init 声明
    ↓
找到 init.cpp 实现
    ↓
自己编译 Debug libroscpp.so
    ↓
让业务 Node 真正加载这份库
    ↓
F5/F11 进入 roscpp
```

## 参考资料

- [VS Code Remote Development](https://code.visualstudio.com/docs/remote/remote-overview)
- [VS Code Dev Containers](https://code.visualstudio.com/docs/devcontainers/containers)
- [VS Code C++ configuration](https://code.visualstudio.com/docs/cpp/customize-cpp-settings)
- [VS Code C++ debugging](https://code.visualstudio.com/docs/cpp/cpp-debug)
