# ros1_comm_lab

阶段 09 的最小实验 package，用于验证 ROS1 Noetic roscpp 的 CallbackQueue、Spinner 与 callback 并发行为。

## 构建

```bash
cd /workspace/ros_ws
catkin build ros1_comm_lab
source devel/setup.bash
```

## Spinner 阻塞实验

```bash
roslaunch ros1_comm_lab spinner_lab.launch spinner_mode:=spin
```

分别在两个终端持续发送：

```bash
rostopic pub -r 0.2 /comm_lab/slow std_msgs/UInt32 "data: 1"
```

```bash
rostopic pub -r 5 /comm_lab/fast std_msgs/UInt32 "data: 1"
```

切换到两个 worker：

```bash
roslaunch ros1_comm_lab spinner_lab.launch \
    spinner_mode:=multi spinner_threads:=2
```

验证同一个 subscription 是否允许并发：

```bash
roslaunch ros1_comm_lab spinner_lab.launch \
    spinner_mode:=multi \
    spinner_threads:=4 \
    allow_concurrent_callbacks:=true
```

## 自定义 CallbackQueue 实验

```bash
roslaunch ros1_comm_lab custom_queue_lab.launch
```

分别在两个终端执行：

```bash
rostopic pub -r 0.2 /comm_lab/driver std_msgs/UInt32 "data: 1"
```

```bash
rostopic pub -r 5 /comm_lab/fast std_msgs/UInt32 "data: 1"
```

`/comm_lab/driver` 由自定义 queue + 独立 `AsyncSpinner` worker 处理，`/comm_lab/fast` 继续由 global callback queue + `ros::spin()` 处理。
