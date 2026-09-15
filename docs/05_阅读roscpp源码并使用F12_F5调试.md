<meta name="referrer" content="no-referrer" />

# 05：阅读 roscpp 源码，并用 F12、F5、GDB 进入 `ros::init()`

> 摘要：这一章把“能看 ROS 头文件”和“运行时真的进入 ROS 源码”分成两条独立链路。你会准备 `ros_debug_ws`，构建带 Debug 信息的 `libroscpp.so`，让业务 `ros_ws` 链接这份 overlay，再用 `ldd`、`readelf`、纯 GDB 和 VS Code F5 逐层证明调试链路正确。

[TOC]

## 本章目标

完成本章后，你应该能够：

1. 区分 F12 源码导航和 F5/GDB 运行时调试；
2. 知道 `ros::init()` 声明与实现分别在哪里；
3. 把 `ros_comm`、`roscpp_core`、`rosconsole`、`std_msgs`、`ros_comm_msgs` 放入 Debug workspace；
4. 使用 merged devel 构建 Debug `roscpp`；
5. 理解源码 header、生成 header、系统 header 三种来源；
6. 用 `readelf` 判断 `libroscpp.so` 是否包含 Debug 信息；
7. 让业务 workspace `--extend /workspace/ros_debug_ws/devel`；
8. 用 `ldd` 证明 `hello_node` 运行时加载的是 Debug overlay；
9. 用纯 GDB 设置 `ros::init` 断点；
10. 用 VS Code F5/F11 进入 `roscpp` 实现源码。

## 为什么 F12 能跳，并不代表 F11 一定能进入

先建立两条完全不同的链路。

### 源码导航链

```text
hello_node.cpp
    ↓ F12
ros/ros.h
    ↓ declaration/navigation
ros/init.h
    ↓ search implementation
ros_comm/clients/roscpp/src/libros/init.cpp
```

这条链依赖：

```text
源码文件在不在
IntelliSense include path 对不对
符号数据库是否正确
```

### 运行时调试链

```text
hello_node ELF
    ↓ dynamic linker
libroscpp.so
    ↓ DWARF Debug info
init.cpp
    ↓ GDB breakpoint / step
ros::init()
```

这条链依赖：

```text
实际加载哪一份 .so
那份 .so 是否带 Debug 信息
GDB 是否在正确环境运行
源码路径是否能映射
```

所以：

```text
F12 成功
```

只能证明源码导航链成功，不能证明运行时加载了自己编译的 `libroscpp.so`。

---

## Step 1：先找到 `ros::init()` 的声明

打开：

```text
ros_ws/src/ros1_hello/src/hello_node.cpp
```

代码：

```cpp
ros::init(argc, argv, "hello_node");
```

在 Dev Container 或已经配置好的 Remote-SSH Host 中，对 `ros::init` 做 F12。

声明来自 roscpp header，核心文件：

```text
ros_comm/clients/roscpp/include/ros/init.h
```

但真正实现不在 header 中。

实现位于：

```text
ros_comm/clients/roscpp/src/libros/init.cpp
```

这就是为什么只安装 `/opt/ros/noetic/include` 只能很好地看到声明，却不能完整阅读实现源码。

---

## Step 2：在 Host 准备上游源码

进入：

```bash
cd /home/wdfk/share/ros1-docker/ros_debug_ws/src
```

依次 clone：

```bash
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

为什么不是只 clone `ros_comm`？

因为 `roscpp` 的依赖并不全部在一个 Git repository 中。

例如：

```text
ros/ros.h
    → ros_comm / roscpp

ros/time.h
    → roscpp_core / rostime

ros/console.h
    → rosconsole

std_msgs/String.msg
    → std_msgs

rosgraph_msgs/Clock.msg
    → ros_comm_msgs
```

### 检查点

Host：

```bash
test -f ros_comm/clients/roscpp/include/ros/ros.h \
    && echo "ros.h OK"

test -f ros_comm/clients/roscpp/src/libros/init.cpp \
    && echo "init.cpp OK"

test -f roscpp_core/rostime/include/ros/time.h \
    && echo "time.h OK"

test -f rosconsole/include/ros/console.h \
    && echo "console.h OK"
```

如果某个文件不存在，先处理源码仓库问题，不要去改 IntelliSense。

---

## Step 3：确认 Container 能看到同一份源码

进入 Container：

```bash
docker compose exec ros1-dev bash
```

检查：

```bash
test -f \
    /workspace/ros_debug_ws/src/ros_comm/clients/roscpp/src/libros/init.cpp \
    && echo "roscpp source visible"
```

因为整个仓库 bind mount 到 `/workspace`，所以 Host clone 的源码应该直接出现在 Container。

这一步证明的是：

```text
Host 源码
    ↓ bind mount
Container 可见
```

不需要 `docker cp`，也不需要额外的 Host SDK 副本。

---

## Step 4：认识三类 header

在真正 build Debug workspace 前，先把后面最容易混淆的头文件分成三类。

### 4.1 源码 header

Git 仓库中原本就存在：

```text
ros/ros.h
ros/init.h
ros/time.h
ros/console.h
```

来源：

```text
ros_debug_ws/src/...
```

如果这类文件缺失，通常说明对应源码仓库没准备好。

### 4.2 catkin 生成 header

源码仓库里不一定直接存在最终 `.h`：

```text
ros/common.h
std_msgs/String.h
rosgraph_msgs/Clock.h
```

例如 `std_msgs/String.h` 来自：

```text
std_msgs/msg/String.msg
    ↓ message generation
std_msgs/String.h
```

`ros/common.h` 则由 roscpp 的 CMake 配置阶段从模板生成。

这类文件要在：

```text
ros_debug_ws/devel/include
```

找。

### 4.3 Host 系统 header

例如：

```text
boost/shared_ptr.hpp
boost/function.hpp
boost/math/special_functions/round.hpp
```

它们不是 ROS package 源码。

Remote-SSH Host IntelliSense 运行在 Ubuntu24 Host 上，所以需要 Host 的系统开发头文件存在。

检查：

```bash
test -f /usr/include/boost/shared_ptr.hpp \
    && echo "Boost OK" \
    || echo "Boost MISSING"
```

如果缺少，Host 可以安装普通 C++ Boost header：

```bash
sudo apt update
sudo apt install libboost-dev
```

这不是在 Host 安装 ROS1。

---

## Step 5：初始化 `ros_debug_ws`

Container：

```bash
cd /workspace/ros_debug_ws
source /opt/ros/noetic/setup.bash
catkin init
```

配置：

```bash
catkin config \
    --merge-devel \
    --extend /opt/ros/noetic \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

检查：

```bash
catkin config
```

重点：

```text
Extending:                    /opt/ros/noetic
Devel Space Layout:          merged
```

### 为什么 Debug workspace 使用 merged devel

Container 路径是：

```text
/workspace/ros_debug_ws
```

Host 路径是：

```text
/home/wdfk/share/ros1-docker/ros_debug_ws
```

我们希望生成 header 在两边都能直接读取：

```text
ros_debug_ws/devel/include/...
```

merged devel 更适合这种“同一个 build output 既要给 Container 编译，也要给 Host IntelliSense 看”的模式。

---

## Step 6：确认 workspace 识别到了关键 package

```bash
catkin list --unformatted | grep -E \
'^(roscpp|rostime|std_msgs|rosconsole|rosgraph_msgs)$'
```

至少应该看到：

```text
roscpp
rostime
std_msgs
rosconsole
rosgraph_msgs
```

这里使用：

```bash
catkin list --unformatted
```

是因为普通格式可能带状态标记、路径或其它格式化内容，不适合做精确名称匹配。

如果 `roscpp` 不在列表中，先回去检查：

```text
ros_debug_ws/src/ros_comm
```

是否存在。

---

## Step 7：构建 Debug roscpp 和需要的消息包

执行：

```bash
catkin build roscpp std_msgs rosconsole rosgraph_msgs
```

catkin_tools 会根据 package dependency graph 自动决定顺序。

你可能看到类似：

```text
cpp_common
std_msgs
rostime
roscpp_traits
xmlrpcpp
roscpp_serialization
rosconsole
rosgraph_msgs
roscpp
```

真正重要的是最后：

```text
All ... packages succeeded
```

### 检查生成 header

```bash
test -f /workspace/ros_debug_ws/devel/include/ros/common.h \
    && echo "ros/common.h OK"

test -f /workspace/ros_debug_ws/devel/include/std_msgs/String.h \
    && echo "std_msgs/String.h OK"

test -f /workspace/ros_debug_ws/devel/include/rosgraph_msgs/Clock.h \
    && echo "rosgraph_msgs/Clock.h OK"
```

再看：

```bash
ls -l /workspace/ros_debug_ws/devel/include/std_msgs/String.h
```

我们希望它是 Host 也能读取的正常生成结果，而不是一个指向其它 Container-only 绝对路径的失效链接。

### Host 再检查一次

Host：

```bash
cd /home/wdfk/share/ros1-docker

test -f ros_debug_ws/devel/include/ros/common.h \
    && echo "Host common OK"

test -f ros_debug_ws/devel/include/std_msgs/String.h \
    && echo "Host String OK"
```

这一步把“构建成功”和“Host IntelliSense 真能读”区分开了。

---

## Step 8：确认 `libroscpp.so` 包含 Debug 信息

Container：

```bash
readelf -S \
    /workspace/ros_debug_ws/devel/lib/libroscpp.so \
    | grep -E 'debug_info|debug_line'
```

如果能看到类似：

```text
.debug_info
.debug_line
```

说明这份共享库包含 DWARF Debug section。

只看到源代码文件并不够。

真正 F11 进入 `ros::init()` 需要运行时加载的那份库本身带调试信息。

---

## Step 9：让业务 workspace 以 Debug workspace 为 underlay

现在切到业务 workspace：

```bash
cd /workspace/ros_ws
```

先 source Debug overlay：

```bash
source /opt/ros/noetic/setup.bash
source /workspace/ros_debug_ws/devel/setup.bash
```

再配置业务 workspace：

```bash
catkin config \
    --merge-devel \
    --extend /workspace/ros_debug_ws/devel \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

然后重新构建：

```bash
catkin build ros1_hello
```

为什么要重新 build？

因为业务 Node 在链接阶段需要重新解析：

```text
roscpp 到底来自哪个 prefix
```

如果它早已链接到了：

```text
/opt/ros/noetic/lib/libroscpp.so
```

仅仅后来 source 一个 overlay，并不能保证旧 ELF 的依赖关系自动改变。

---

## Step 10：用 `ldd` 证明运行时库来源

这是进入 ROS 内部调试前最重要的证据之一。

执行：

```bash
ldd \
    /workspace/ros_ws/devel/lib/ros1_hello/hello_node \
    | grep roscpp
```

目标是看到：

```text
/workspace/ros_debug_ws/devel/lib/libroscpp.so
```

而不是：

```text
/opt/ros/noetic/lib/libroscpp.so
```

如果仍然是 `/opt/ros/noetic`，先不要开 VS Code F11。

因为那时你调试的仍然是系统安装版 roscpp。

### 也可以看环境顺序

```bash
echo "$LD_LIBRARY_PATH" | tr ':' '\n'
```

应该让：

```text
/workspace/ros_ws/devel/lib
/workspace/ros_debug_ws/devel/lib
```

位于 `/opt/ros/noetic/lib` 前面。

---

## Step 11：先用纯 GDB 证明 `ros::init` 能断住

复杂 IDE 调试前，先用最小工具验证。

Terminal A：

```bash
roscore
```

Terminal B：

```bash
source /workspace/.devcontainer/ros_env.bash

gdb -q \
    /workspace/ros_ws/devel/lib/ros1_hello/hello_node
```

进入 GDB：

```gdb
set breakpoint pending on
rbreak ^ros::init
run
```

`ros::init` 有重载，所以：

```gdb
rbreak ^ros::init
```

比一开始猜某个精确签名更方便。

如果 GDB 在 `init.cpp` 中停住，说明：

```text
ELF
  ↓
Debug libroscpp.so
  ↓
DWARF
  ↓
ros::init source
```

这条运行时链已经成立。

---

## Step 12：再回到 VS Code Remote-SSH Host F5

确认 Container 正在运行：

```bash
docker compose ps
```

确认 GDB：

```bash
docker exec -i ros1-dev sh -c '/usr/bin/gdb --version'
```

如果要测试 GDB/MI：

```bash
printf '1-gdb-version\n2-gdb-exit\n' \
    | docker exec -i ros1-dev \
      sh -c '/usr/bin/gdb --interpreter=mi2 --quiet'
```

然后 VS Code Run and Debug 选择：

```text
ROS1 [HOST] Debug hello_node in Docker
```

### 为什么没有 `preLaunchTask`

本工程故意不让 F5 自动：

```text
catkin build
roscore
rosrun
roslaunch
```

因为学习阶段应该明确每一步是什么。

推荐顺序：

```text
1. 手工 catkin build
2. 手工 roscore
3. 确认动态库来源
4. 再 F5
```

这样 F5 失败时变量更少。

---

## Step 13：先在自己的代码上练习 F5/F10/F11

在：

```cpp
ros::init(argc, argv, "hello_node");
```

前后设置断点。

先熟悉：

```text
F5
    Start / Continue

F10
    Step Over

F11
    Step Into

Shift+F11
    Step Out
```

建议先在自己的：

```text
hello_node.cpp
```

中确认：

```text
断点能停
变量能看
源码路径正确
```

再尝试进入 roscpp。

---

## Step 14：F11 进入 `ros::init()`

当执行到：

```cpp
ros::init(argc, argv, "hello_node");
```

按 F11。

目标源码：

```text
ros_debug_ws/src/ros_comm/clients/roscpp/src/libros/init.cpp
```

如果 Host F5 使用 `pipeTransport`，GDB 的 DWARF 中记录的是：

```text
/workspace/...
```

而 VS Code Explorer 打开的是：

```text
/home/wdfk/share/ros1-docker/...
```

所以 `launch.json` 中：

```jsonc
"sourceFileMap": {
    "/workspace": "${workspaceFolder}"
}
```

把 Container 源码路径映射回 Host。

---

## Step 15：阅读 `init.cpp` 时先观察什么

不要一进入 ROS 源码就试图理解所有全局变量和线程。

建议按三个层次读。

### 第一层：`ros::init()` 的入口

看它接收：

```text
argc / argv
Node name
init options
```

以及它如何处理 remapping 参数和全局初始化状态。

### 第二层：`NodeHandle` 创建后的变化

自己的代码：

```cpp
ros::NodeHandle nh;
```

是另一个重要观察点。

可以比较：

```text
只执行 ros::init()
```

和：

```text
已经构造 NodeHandle
```

之后 roscpp 的内部状态变化。

### 第三层：Manager 对象

随着学习深入，再看：

```text
XMLRPCManager
TopicManager
ServiceManager
PollManager
ConnectionManager
```

不要第一天就把它们全部展开。

先把：

```text
init → NodeHandle → advertise → publish
```

这条主线走通。

---

## Step 16：为什么有时 F11 会进入 STL/Boost

调试 C++ 库时，F11 不知道你“只想看 ROS”。

它只知道：

```text
下一次函数调用在哪里
```

所以可能进入：

```text
std::string
std::map
boost::shared_ptr
pthread
libstdc++
```

在 Container 中编译的 Noetic/Focal 代码可能记录：

```text
/usr/include/c++/9/...
```

而 Ubuntu24 Host 使用：

```text
/usr/include/c++/13/...
```

这时 Host 模式不一定能打开 Container 的每一份系统 header 源文件。

默认处理：

```text
Shift+F11 Step Out
```

回到你真正关心的 roscpp。

如果目标是深入系统库源码，切到 Dev Container 会更准确，因为它直接拥有 Focal Container 的 `/usr/include`。

---

## Step 17：F12 和 F5 最终应该形成的工作法

源码阅读：

```text
hello_node.cpp
    ↓ F12
ros/ros.h
    ↓
ros/init.h
    ↓
init.cpp
```

运行时调试：

```text
catkin build ros_debug_ws
    ↓
readelf 检查 Debug info
    ↓
ros_ws extend ros_debug_ws/devel
    ↓
重新 build ros1_hello
    ↓
ldd 检查 libroscpp.so 来源
    ↓
纯 GDB 验证断点
    ↓
VS Code F5/F11
```

这套顺序最大的价值是：

```text
每一步都有证据
```

而不是看到 F11 进不去时，连续修改十个 VS Code 配置项。

---

## 本章常见问题

### `catkin build roscpp` 提示没有 roscpp package

检查：

```bash
catkin list --unformatted | grep -x roscpp
```

如果没有输出，说明 `ros_comm` 没有正确出现在当前 source space。

### `std_msgs/String.h` 在 Container 存在，Host `test -f` 却失败

先：

```bash
ls -l ros_debug_ws/devel/include/std_msgs/String.h
```

如果它是一个指向 `/workspace/...` 的绝对符号链接，Host 无法解析 Container-only 路径。

本工程从一开始使用：

```bash
catkin config --merge-devel
```

避免这种 Host 可见性问题。

### F12 能进 `init.cpp`，F11 进不去

先检查：

```bash
ldd /workspace/ros_ws/devel/lib/ros1_hello/hello_node | grep roscpp
```

再：

```bash
readelf -S /workspace/ros_debug_ws/devel/lib/libroscpp.so \
    | grep -E 'debug_info|debug_line'
```

这两个证据比继续调 IntelliSense 更重要。

---

## 本章练习

1. 在 `ros::init()` 前设置断点，用 F11 进入 roscpp；
2. 在 `ros::NodeHandle nh;` 前后观察 call stack；
3. 用 `ldd` 分别观察 Debug overlay 前后的 `libroscpp.so` 来源；
4. 用 `readelf -S` 找 `.debug_info`；
5. 在 Dev Container 中用纯 GDB 重复同样的断点实验；
6. 对 `ros::Publisher::publish` 继续做 F12，画出“自己的代码 → roscpp”调用路径。

最后一章不是继续增加新功能，而是把整个工程的排查方法整理成一套固定流程。以后遇到红线、catkin 报错、F12/F5 失效时，先根据现象分类，再收集证据。

## 参考资料

- [ros_comm](https://github.com/ros/ros_comm)
- [roscpp_core](https://github.com/ros/roscpp_core)
- [rosconsole](https://github.com/ros/rosconsole)
- [std_msgs](https://github.com/ros/std_msgs)
- [ros_comm_msgs](https://github.com/ros/ros_comm_msgs)
- [GDB documentation](https://sourceware.org/gdb/documentation/)
