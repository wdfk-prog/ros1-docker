# ros1_description_lab

第 14 章 URDF / Xacro / `robot_state_publisher` / `joint_states` 教学 package。

本 package 保留两份机器人模型：

- `urdf/agv.urdf`：与默认 Xacro 展开结构等价的纯 URDF 学习基线，适合直接学习 `link` / `joint` / `origin` / `axis`。
- `urdf/agv.urdf.xacro`：工程化 Xacro 源文件，通过 property、macro、include 和参数生成等价 URDF。

运行时链路：

```text
agv.urdf.xacro
    -> xacro processor
    -> URDF XML
    -> /robot_description
    -> robot_state_publisher

/joint_states
    -> robot_state_publisher
    -> /tf + /tf_static
```

TF owner：

- `ros1_driver_lab`：发布 `/joint_states` 与 `/odom`。
- `ros1_tf_lab/odom_tf_broadcaster`：发布动态 `odom -> base_link`。
- `robot_state_publisher`：读取 `/robot_description` 与 `/joint_states`，发布机器人自身 link tree。
- `tf2_ros/static_transform_publisher`：本章暂时提供恒等 `map -> odom`。

构建：

```bash
cd /workspace/ros_ws
catkin build ros1_driver_lab ros1_tf_lab ros1_description_lab
source devel/setup.bash
```

先单独展开 Xacro：

```bash
rosrun xacro xacro \
  $(rospack find ros1_description_lab)/urdf/agv.urdf.xacro \
  > /tmp/agv.generated.urdf
```

运行默认模型：

```bash
roslaunch ros1_description_lab description_lab.launch
```

关闭 camera 分支，观察 Xacro 参数如何改变最终 URDF：

```bash
roslaunch ros1_description_lab description_lab.launch use_camera:=false
```

观察固定传感器关系：

```bash
rosrun tf2_ros tf2_echo base_link laser_link
```

观察轮子动态关系：

```bash
rosrun tf2_ros tf2_echo base_link left_wheel_link
```

完整解释见 `docs/14_URDF_robot_state_publisher与joint_states.md`。
