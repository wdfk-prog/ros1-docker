# ros1_hello

一个最小但完整的 ROS1 C++ Publisher/Subscriber 学习 package。

```text
ros1_hello/
├── package.xml
├── CMakeLists.txt
├── src/
│   ├── hello_node.cpp
│   └── hello_listener.cpp
└── launch/
    └── hello.launch
```

`hello_node` 发布 `/chatter`，`hello_listener` 订阅 `/chatter`。`hello.launch` 可以一次启动两个 Node，并通过 private parameter 设置发布频率。

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
```

## 手工运行

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
rostopic echo /chatter
```

## 使用 roslaunch

```bash
roslaunch ros1_hello hello.launch
```

把发布频率改为 5 Hz：

```bash
roslaunch ros1_hello hello.launch publish_rate:=5.0
```

完整讲解见仓库 `docs/02_创建catkin工作空间_Package与第一个Node.md` 和 `docs/03_让Node通信_Topic_Parameter与roslaunch.md`。
