# ros1_localization_lab

第 15 章 `robot_localization` 实验 package。它复用前面已经建立的 Driver、URDF/Xacro 与 TF 契约，并提供 EKF/UKF、`differential`、`relative` 的工程对照。

默认链路：

```text
ros1_driver_lab
├── /odom/raw      -> vx
└── /imu/data_raw  -> angular_velocity.z
          |
          v
robot_localization
├── /odometry/filtered
└── odom -> base_link
          |
          v
robot_state_publisher
└── base_link -> wheel/sensor frames
```

## EKF / UKF

默认 EKF：

```bash
roslaunch ros1_localization_lab localization_lab.launch
```

使用同一套 wheel + IMU 输入切换 UKF：

```bash
roslaunch ros1_localization_lab localization_lab.launch \
  filter_node:=ukf_localization_node \
  config_file:=$(rospack find ros1_localization_lab)/config/ukf_wheel_imu.yaml
```

只使用 wheel odometry 的对照配置：

```bash
roslaunch ros1_localization_lab localization_lab.launch \
  config_file:=$(rospack find ros1_localization_lab)/config/ekf_wheel_only.yaml
```

观察输出：

```bash
rostopic echo -n 1 /odom/raw
rostopic echo -n 1 /imu/data_raw
rostopic echo -n 1 /odometry/filtered
rosrun tf tf_echo odom base_link
```

## differential / relative

这两个实验只使用 `/demo_pose`，避免 wheel/IMU 输入干扰。

relative：

```bash
roslaunch ros1_localization_lab pose_mode_lab.launch \
  config_file:=$(rospack find ros1_localization_lab)/config/ekf_pose_relative.yaml
```

differential：

```bash
roslaunch ros1_localization_lab pose_mode_lab.launch \
  config_file:=$(rospack find ros1_localization_lab)/config/ekf_pose_differential.yaml
```

另开终端发布三帧固定 pose 序列：

```bash
rosrun ros1_localization_lab demo_pose_sequence.py
```

然后观察：

```bash
rostopic echo /odometry/filtered
```

本教学 Driver 的 IMU 数据由左右轮速度在进程内推导，和 wheel odometry 并不是独立物理传感器。因此 wheel + IMU 部分用于学习接口、状态维度、covariance、时间与 TF ownership，不用于证明真实传感器融合精度提升。
