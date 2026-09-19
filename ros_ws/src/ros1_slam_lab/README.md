# ros1_slam_lab

第 16 章配套实验。它不模拟真实雷达硬件协议，而是在一个确定的二维线段环境中根据 `/cmd_vel` 生成 `sensor_msgs/LaserScan`，用于观察：

```text
/scan + odom + TF -> slam_gmapping -> /map + map -> odom
/map + /scan + odom + TF -> AMCL -> map -> odom
```

两个主入口：

```bash
roslaunch ros1_slam_lab slam_mapping.launch
roslaunch ros1_slam_lab amcl_localization.launch
```

默认 `truth_linear_scale=1.03`，故意让激光对应的“真实运动”比里程计快 3%，便于观察全局定位通过 `map -> odom` 吸收累计误差。把它改为 `1.0` 可得到几乎无尺度漂移的对照组。
