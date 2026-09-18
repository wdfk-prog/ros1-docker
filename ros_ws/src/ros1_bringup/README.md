# ros1_bringup

`ros1_bringup` 是顶层启动 package。它不实现业务算法，只负责把多个 package 的 launch 文件和关键 Node 组织成一个系统入口。

当前入口：

```bash
roslaunch ros1_bringup system.launch
```

`system.launch` 会启动：

- `ros1_hello/hello.launch` 中的 `hello_node` 与 `hello_listener`；
- `ready_client`；
- `ready_server`。

`ready_client` 被故意写在 `ready_server` 前面。`ready_server` 模拟驱动初始化完成后才发布 `/demo_driver/ready` Service，
`ready_client` 使用 `ros::service::waitForService()` 等待 READY 条件成立，再进入 `/chatter` 订阅业务路径。
两个 READY Node 都标记为 `required="true"`，用于演示关键进程退出时关闭整套 roslaunch；这与业务 READY 条件是两个层次。

因此这个示例验证的是：**launch 负责启动进程，Node 自己负责业务就绪依赖。**

完整解释见 `docs/ROS教程11.5：roscore源码阅读——从启动脚本到Master注册表与控制面.md`。
