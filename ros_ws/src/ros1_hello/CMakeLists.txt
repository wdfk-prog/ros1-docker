cmake_minimum_required(VERSION 3.0.2)
project(ros1_hello)

# 找到 catkin，以及当前 package 使用的 ROS1 组件。
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

# 当前示例没有向其它 package 导出自定义 library/header，保持最小配置即可。
catkin_package()

# 使用 catkin 依赖导出的 include path。
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# src/hello_node.cpp -> hello_node executable。
add_executable(hello_node
  src/hello_node.cpp
)

target_link_libraries(hello_node
  ${catkin_LIBRARIES}
)

# src/hello_listener.cpp -> hello_listener executable。
add_executable(hello_listener
  src/hello_listener.cpp
)

target_link_libraries(hello_listener
  ${catkin_LIBRARIES}
)
