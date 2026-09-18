# ros1_comm_lab

`ros1_comm_lab` 承载阶段 09～11 的 ROS1 roscpp 通信机制实验。

## 构建

```bash
cd /workspace/ros_ws
catkin build ros1_comm_lab
source devel/setup.bash
```

## 阶段 09：Spinner 与 CallbackQueue

```bash
roslaunch ros1_comm_lab spinner_lab.launch spinner_mode:=spin
```

分别在两个终端持续发送：

```bash
rostopic pub -r 0.2 /comm_lab/slow std_msgs/UInt32 "data: 1"
rostopic pub -r 5 /comm_lab/fast std_msgs/UInt32 "data: 1"
```

切换多线程 Spinner：

```bash
roslaunch ros1_comm_lab spinner_lab.launch \
    spinner_mode:=multi spinner_threads:=2
```

自定义 CallbackQueue：

```bash
roslaunch ros1_comm_lab custom_queue_lab.launch
```

## 阶段 10：Unit Test 与 rostest

运行本 package 的 gtest 与 rostest：

```bash
cd /workspace/ros_ws
catkin run_tests ros1_comm_lab
catkin_test_results
```

`catkin_add_gtest()` 注册的纯逻辑单测和 `add_rostest_gtest()` 注册的 Node 集成测试都会进入这套 test target；测试结果 XML 可在 `ros_ws/build/ros1_comm_lab/test_results/` 一带继续查看。

手工启动测试 Node：

```bash
roslaunch ros1_comm_lab testable_node.launch
```

发送合法输入：

```bash
rostopic pub -1 /comm_lab/test_input std_msgs/UInt32 "data: 7"
rostopic echo -n 1 /comm_lab/test_output
```

默认参数下输出应为：

```text
data: 17
```

发送越界输入：

```bash
rostopic pub -1 /comm_lab/test_input std_msgs/UInt32 "data: 101"
```

Node 会记录 warning，并且不会发布新的 `/comm_lab/test_output`。
