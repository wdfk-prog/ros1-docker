#!/usr/bin/env bash
# 统一的 ROS1 环境入口。
# source 顺序很重要：先系统 Noetic underlay，再源码调试 workspace，最后业务 workspace。
# 后 source 的 overlay 可以覆盖前面环境中同名 package，符合 catkin workspace 的常见叠加方式。

# 系统安装的 ROS1 Noetic 基础环境，提供 roscore/roscpp/标准消息等。
source /opt/ros/noetic/setup.bash

# 可选的 ROS 源码调试 workspace：只有构建过并生成 setup.bash 时才叠加。
if [ -f /workspace/ros_debug_ws/devel/setup.bash ]; then
    source /workspace/ros_debug_ws/devel/setup.bash
fi

# 业务学习 workspace 放在最后，让自己编译的 package 优先进入 ROS_PACKAGE_PATH/库搜索路径。
if [ -f /workspace/ros_ws/devel/setup.bash ]; then
    source /workspace/ros_ws/devel/setup.bash
fi
