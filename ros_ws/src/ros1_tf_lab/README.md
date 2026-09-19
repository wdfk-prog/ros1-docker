# ros1_tf_lab

第 13 章 TF / tf2 教学 package，复用 `ros1_driver_lab` 的 `/odom` 输出建立最小移动机器人 TF tree：

```text
map
└── odom
    └── base_link
        ├── laser_link
        └── imu_link
```

组件：

- `odom_tf_broadcaster`：订阅 `/odom`，按消息中的 `header.frame_id`、`child_frame_id`、pose 和 timestamp 发布动态 TF。
- `tf_query_node`：使用 `tf2_ros::Buffer` + `TransformListener` 查询 `map <- laser_link`，可切换 latest / now 并人为加入时间偏移。
- `tf_lab.launch`：启动第 12 章 Driver、静态 TF、动态 TF broadcaster 和查询节点。

构建：

```bash
cd /workspace/ros_ws
catkin build ros1_driver_lab ros1_tf_lab
source devel/setup.bash
```

运行：

```bash
roslaunch ros1_tf_lab tf_lab.launch
```

制造 future extrapolation：

```bash
roslaunch ros1_tf_lab tf_lab.launch query_mode:=now query_offset_sec:=0.5
```

完整解释见 `docs/13_TF_tf2与移动机器人坐标系.md`。
