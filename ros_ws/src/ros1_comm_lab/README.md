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

## 阶段 11：Service 与 Action

阶段 11 新增自定义接口：

```text
srv/TransformValue.srv
  Request:  uint32 input
  Response: bool success, uint32 output, string message

action/Count.action
  Goal:     uint32 target
  Result:   bool completed, uint32 final_count, string message
  Feedback: uint32 current_count, float32 progress
```

构建后启动 Service Server 与 Action Server：

```bash
cd /workspace/ros_ws
catkin build ros1_comm_lab
source devel/setup.bash
roslaunch ros1_comm_lab service_action_lab.launch
```

Service 正常输入：

```bash
rosservice call /comm_lab/transform_value "input: 7"
rosrun ros1_comm_lab transform_service_client 7
```

默认参数下应得到 `success=true`、`output=17`。输入超过 `max_input` 时，Service RPC 仍正常返回，业务结果通过 `success=false` 表示：

```bash
rosservice call /comm_lab/transform_value "input: 101"
```

Action 正常完成：

```bash
rosrun ros1_comm_lab count_action_client 5
```

客户端会打印 Feedback，最终应进入 `SUCCEEDED`，并得到 `completed=1`、`final_count=5`。

Action 取消：

```bash
rosrun ros1_comm_lab count_action_client 100 0.5
```

第二个参数表示发送 Goal 后等待多少秒执行 `cancelGoal()`；已经进入 ACTIVE 的 Goal 正常取消后应进入 `PREEMPTED`。

观察 Action 展开的五类 Topic：

```bash
rostopic list | grep '^/comm_lab/count'
```

运行阶段 10、11 的全部 package 测试：

```bash
cd /workspace/ros_ws
catkin run_tests ros1_comm_lab
catkin_test_results
```

阶段 11 的 `service_action.test` 同时验证 Service 正常/业务拒绝以及 Action 完成/取消路径。
