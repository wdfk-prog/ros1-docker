# ros1_hello

一个最小但完整的 ROS1 C++ Topic 学习 package，同时保留标准消息与自定义 `.msg` 两条独立实验链。

```text
ros1_hello/
├── msg/
│   └── HelloStatus.msg
├── src/
│   ├── hello_node.cpp
│   ├── hello_listener.cpp
│   ├── custom_msg_publisher.cpp
│   └── custom_msg_subscriber.cpp
├── launch/
│   ├── hello.launch
│   └── custom_msg.launch
├── package.xml
└── CMakeLists.txt
```

原有示例保持不变：`hello_node` 使用 `std_msgs/String` 发布 `/chatter`，`hello_listener` 订阅 `/chatter`。

新增示例通过 `msg/HelloStatus.msg` 生成 `ros1_hello/HelloStatus` 类型，并在 `/hello_status` 上发布/订阅。这样可以直接对比“使用标准消息”和“自己定义 `.msg`”，同时不会影响后续章节基于 `/chatter` 的源码阅读。

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
catkin build ros1_hello
source /workspace/ros_ws/devel/setup.bash
```

构建后可以确认自定义消息与生成头文件：

```bash
rosmsg show ros1_hello/HelloStatus
find /workspace/ros_ws/devel -path '*/ros1_hello/HelloStatus.h' -print
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

也可以：

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

理解 `roslaunch` 后，也可以一次启动两个自定义消息 Node：

```bash
roslaunch ros1_hello custom_msg.launch
```

完整讲解见仓库 `docs/02_创建catkin工作空间_Package与第一个Node.md` 和 `docs/03_让Node通信_Topic_Parameter与roslaunch.md`。
