cmake_minimum_required(VERSION 3.0.2)
project(ros1_hello)

# 找到 catkin，以及当前 package 使用的 ROS1 组件。
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  std_srvs
)

# 声明本 package 自定义 Topic 消息。
add_message_files(
  FILES
  HelloStatus.msg
)

# 根据 .msg 生成 C++/Python 等语言绑定。
generate_messages()

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp std_msgs std_srvs
)

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

# 自定义 .msg 示例：单独使用新 Topic，避免改变原 /chatter 教学链路。
add_executable(custom_msg_publisher
  src/custom_msg_publisher.cpp
)

add_dependencies(custom_msg_publisher
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(custom_msg_publisher
  ${catkin_LIBRARIES}
)

add_executable(custom_msg_subscriber
  src/custom_msg_subscriber.cpp
)

add_dependencies(custom_msg_subscriber
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(custom_msg_subscriber
  ${catkin_LIBRARIES}
)

# 工程启动依赖示例：ready_server 模拟驱动初始化完成后提供 READY Service，
# ready_client 等待 READY 后才进入业务 Topic 路径。
add_executable(ready_server
  src/ready_server.cpp
)

target_link_libraries(ready_server
  ${catkin_LIBRARIES}
)

add_executable(ready_client
  src/ready_client.cpp
)

target_link_libraries(ready_client
  ${catkin_LIBRARIES}
)
