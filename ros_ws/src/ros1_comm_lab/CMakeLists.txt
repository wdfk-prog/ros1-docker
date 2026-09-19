cmake_minimum_required(VERSION 3.0.2)
project(ros1_comm_lab)

# 本 package 的示例代码使用 C++14（例如 std::make_unique 等写法）。
add_compile_options(-std=c++14)

# 告诉 CMake/catkin：当前 package 在“配置和编译阶段”需要哪些 ROS 组件。
# - roscpp/std_msgs：普通 ROS1 C++ Topic/Service 基础；
# - actionlib/actionlib_msgs：Action client/server 与标准 Action 状态消息；
# - message_generation：根据本 package 的 .srv/.action 生成 C++ 消息代码。
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  roscpp
  std_msgs
)

# 注册自定义 Service 定义；这里只登记输入文件，真正的生成代码会在后续构建 target 时产生。
add_service_files(
  FILES
    TransformValue.srv
)

# 注册自定义 Action 定义。一个 .action 会进一步生成 Goal/Result/Feedback 等多种消息类型。
add_action_files(
  FILES
    Count.action
)

# 配置 .srv/.action 的代码生成规则；真正的 C++/Python 等生成文件会在后续构建 target 时产生。
# DEPENDENCIES 列出这些接口定义中会引用到的其它消息 package。
generate_messages(
  DEPENDENCIES
    actionlib_msgs
    std_msgs
)

# 把本 package 对其它 package 的“导出依赖”告诉 catkin。
# message_runtime 是运行已生成消息代码时需要的依赖，与构建期 message_generation 分工不同。
catkin_package(
  CATKIN_DEPENDS
    actionlib
    actionlib_msgs
    message_runtime
    roscpp
    std_msgs
)

# include 目录放本 package 自己的公共头文件；${catkin_INCLUDE_DIRS} 来自上面的 find_package(catkin ...)。
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# 纯业务逻辑做成 library，既可以被 Node 链接，也可以直接被 gtest 单元测试链接。
add_library(ros1_comm_lab_core
  src/message_processor.cpp
)

# --------------------------- Callback / Spinner 实验 ---------------------------
# 每个 add_executable() 都会生成一个可以被 rosrun/roslaunch 启动的 Node 可执行文件。
add_executable(spinner_lab
  src/spinner_lab.cpp
)

target_link_libraries(spinner_lab
  ${catkin_LIBRARIES}
)

add_executable(custom_queue_lab
  src/custom_queue_lab.cpp
)

target_link_libraries(custom_queue_lab
  ${catkin_LIBRARIES}
)

# ------------------------------- 可测试 Node ---------------------------------
add_executable(testable_node
  src/testable_node.cpp
)

target_link_libraries(testable_node
  ros1_comm_lab_core
  ${catkin_LIBRARIES}
)

# ------------------------------ Service 实验 ---------------------------------
add_executable(transform_service_server
  src/transform_service_server.cpp
)

# 这个 target 会 include 由 TransformValue.srv 生成的头文件，所以必须先等待消息生成 target 完成。
add_dependencies(transform_service_server
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(transform_service_server
  ros1_comm_lab_core
  ${catkin_LIBRARIES}
)

add_executable(transform_service_client
  src/transform_service_client.cpp
)
add_dependencies(transform_service_client
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(transform_service_client
  ${catkin_LIBRARIES}
)

# ------------------------------- Action 实验 ---------------------------------
add_executable(count_action_server
  src/count_action_server.cpp
)
add_dependencies(count_action_server
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(count_action_server
  ${catkin_LIBRARIES}
)

add_executable(count_action_client
  src/count_action_client.cpp
)
add_dependencies(count_action_client
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(count_action_client
  ${catkin_LIBRARIES}
)

# CATKIN_ENABLE_TESTING 在 catkin 的测试模式下为真；普通生产构建不会无条件创建测试 target。
if(CATKIN_ENABLE_TESTING)
  # rostest 负责“先启动 ROS Node，再运行 gtest”的集成测试场景。
  find_package(rostest REQUIRED)

  # 纯 C++ 单测：不启动 ROS Master/Node，只验证 MessageProcessor 业务逻辑。
  catkin_add_gtest(test_message_processor
    test/test_message_processor.cpp
  )
  if(TARGET test_message_processor)
    target_link_libraries(test_message_processor
      ros1_comm_lab_core
    )
  endif()

  # rostest + gtest：testable_node.test 负责启动被测 Node，C++ 文件负责断言 Topic 行为。
  add_rostest_gtest(test_testable_node
    test/testable_node.test
    test/test_testable_node.cpp
  )
  if(TARGET test_testable_node)
    target_link_libraries(test_testable_node
      ${catkin_LIBRARIES}
    )
  endif()

  # Service/Action 集成测试同样先由 .test launch 启动 server，再运行 C++ gtest client。
  add_rostest_gtest(test_service_action
    test/service_action.test
    test/test_service_action.cpp
  )
  if(TARGET test_service_action)
    # 测试代码也会 include 自定义 Service/Action 生成头文件，因此需要消息生成依赖。
    add_dependencies(test_service_action
      ${${PROJECT_NAME}_EXPORTED_TARGETS}
      ${catkin_EXPORTED_TARGETS}
    )
    target_link_libraries(test_service_action
      ${catkin_LIBRARIES}
    )
  endif()
endif()
