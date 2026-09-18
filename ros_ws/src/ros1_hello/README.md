# ros1_hello

一个最小但完整的 ROS1 C++ 学习 package。它保留标准 Topic、自定义 `.msg`，并新增一个用于工程启动章节的
READY 依赖示例。

```text
ros1_hello/
├── msg/
│   └── HelloStatus.msg
├── src/
│   ├── hello_node.cpp
│   ├── hello_listener.cpp
│   ├── custom_msg_publisher.cpp
│   ├── custom_msg_subscriber.cpp
│   ├── ready_server.cpp
│   └── ready_client.cpp
├── launch/
│   ├── hello.launch
│   └── custom_msg.launch
├── package.xml
└── CMakeLists.txt
```

原有 `/chatter` 和 `/hello_status` 教学链路保持不变。新增的 `ready_server` / `ready_client` 不替代这些示例，
只用于说明“roslaunch 负责启动进程，Node 自己负责业务 READY 依赖”。

## 构建

Container 中：

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
catkin build ros1_hello ros1_bringup
source /workspace/ros_ws/devel/setup.bash
```

构建后可以确认自定义消息与新增目标：

```bash
rosmsg show ros1_hello/HelloStatus
find /workspace/ros_ws/devel -path '*/ros1_hello/HelloStatus.h' -print
ls -l /workspace/ros_ws/devel/lib/ros1_hello/ready_server
ls -l /workspace/ros_ws/devel/lib/ros1_hello/ready_client
```

## 标准消息 `/chatter`

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

Terminal D：

```bash
rostopic type /chatter
rosmsg show std_msgs/String
rostopic echo /chatter
```

也可以一次启动：

```bash
roslaunch ros1_hello hello.launch
```

把发布频率改为 5 Hz：

```bash
roslaunch ros1_hello hello.launch publish_rate:=5.0
```

## 自定义消息 `/hello_status`

`msg/HelloStatus.msg`：

```text
int32 sequence
string text
```

在两个终端分别运行。Terminal A：

```bash
rosrun ros1_hello custom_msg_publisher
```

Terminal B：

```bash
rosrun ros1_hello custom_msg_subscriber
```

观察接口和 Topic：

```bash
rosmsg show ros1_hello/HelloStatus
rostopic type /hello_status
rostopic echo /hello_status
```

也可以一次启动：

```bash
roslaunch ros1_hello custom_msg.launch
```

## READY 依赖实验

完整工程入口位于另一个 package：`ros1_bringup`。

```bash
roslaunch ros1_bringup system.launch
```

这个 launch 会同时启动原有 `/chatter` 示例以及：

```text
ready_client
    -> waitForService("/demo_driver/ready")

ready_server
    -> 模拟驱动初始化
    -> advertiseService("/demo_driver/ready")
```

`ready_client` 只有确认依赖 READY 后，才开始订阅 `/chatter`。因此这个实验不使用固定 `sleep` 去猜另一个 Node
什么时候初始化完成，也不把 launch XML 的书写顺序当成业务依赖契约。

完整说明见：

- `docs/02_创建catkin工作空间_Package与第一个Node.md`
- `docs/03_让Node通信_Topic_Parameter与roslaunch.md`
- `docs/ROS教程11.5：roscore源码阅读——从启动脚本到Master注册表与控制面.md`
