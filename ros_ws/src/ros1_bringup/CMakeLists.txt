cmake_minimum_required(VERSION 3.0.2)
project(ros1_bringup)

# bringup package 本身没有 C++ 源码，只需要 catkin 提供 package/安装能力。
find_package(catkin REQUIRED)

catkin_package()

# 把 launch/ 安装到 package 的 share 目录，安装空间中仍可 roslaunch ros1_bringup ...。
install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
