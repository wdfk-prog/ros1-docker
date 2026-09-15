#!/usr/bin/env bash
# 统一的 ROS1 环境入口：先 Noetic，再 Debug overlay，最后业务 workspace。

source /opt/ros/noetic/setup.bash

if [ -f /workspace/ros_debug_ws/devel/setup.bash ]; then
    source /workspace/ros_debug_ws/devel/setup.bash
fi

if [ -f /workspace/ros_ws/devel/setup.bash ]; then
    source /workspace/ros_ws/devel/setup.bash
fi
