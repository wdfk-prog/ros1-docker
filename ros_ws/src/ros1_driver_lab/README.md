# ros1_driver_lab

第 12 章教学 package。重点不是设备通信，而是两件事：

1. 已经取得的底盘数据应该如何映射到 ROS1 标准消息；
2. `/cmd_vel` 中的 `linear.x` / `angular.z` 如何通过差速底盘运动学转换为左右轮目标角速度。

本 package **不实现 TCP / CAN / Serial 协议**。`chassis_driver_node` 内部的 `DemoChassisDevice` 只是一个进程内教学数据源，用来让 Topic 可以真实运行和观察。

构建：

```bash
cd /workspace/ros_ws
catkin build ros1_driver_lab
source devel/setup.bash
```

运行：

```bash
roslaunch ros1_driver_lab chassis_lab.launch
```

另开终端发送速度命令：

```bash
source /workspace/ros_ws/devel/setup.bash
rostopic pub -r 5 /cmd_vel geometry_msgs/Twist \
  "linear: {x: 0.3, y: 0.0, z: 0.0}
angular: {x: 0.0, y: 0.0, z: 0.4}"
```

观察：

```bash
rostopic echo /joint_states
rostopic echo /imu/data_raw
rostopic echo /odom
rostopic echo /diagnostics
```

完整解释见：

```text
docs/12_ROS消息与驱动数据契约_差速底盘Driver.md
```
