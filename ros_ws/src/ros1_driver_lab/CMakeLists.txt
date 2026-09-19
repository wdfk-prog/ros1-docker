cmake_minimum_required(VERSION 3.0.2)
project(ros1_driver_lab)

# 本章只使用 ROS1 标准消息，不定义自定义 .msg。
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
    diagnostic_msgs
    geometry_msgs
    nav_msgs
    roscpp
    sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# 第 12 章唯一的教学 Node：
# 1. 假设底层设备数据已经获得；
# 2. 映射成 JointState / Imu / Odometry / DiagnosticArray；
# 3. 把 /cmd_vel 转换为差速底盘左右轮目标角速度。
add_executable(chassis_driver_node
  src/chassis_driver_node.cpp
)

target_link_libraries(chassis_driver_node
  ${catkin_LIBRARIES}
)
