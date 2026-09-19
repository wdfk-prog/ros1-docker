# ros1_navigation_lab

第 17 章 Navigation / `move_base` 教学 package。它不新增传感器 Driver，而是复用第 12～16 章已经建立的底盘、URDF、EKF、激光雷达模型和 AMCL，把数据接入 ROS1 Navigation Stack。

运行链：

```text
/map + map -> odom + odom -> base_link + /scan
                    |
                    v
                 move_base
       global/local costmap
          Navfn + DWA
                    |
                    v
                 /cmd_vel
                    |
                    v
              ros1_driver_lab
```

启动：

```bash
roslaunch ros1_navigation_lab navigation.launch
```

示例目标可以在 RViz 使用 **2D Nav Goal** 指定，也可以发布：

```bash
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: -2.2, z: 0.0}, orientation: {z: -0.7071068, w: 0.7071068}}}"
```

`navigation_world.yaml` 比静态 `lab_map` 多一段临时障碍。它不会出现在 `/map`，只会通过 `/scan` 被 `ObstacleLayer` 标记，用来观察静态地图与实时障碍物如何共同影响导航。
