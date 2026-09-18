cmake_minimum_required(VERSION 3.0.2)
project(ros1_comm_lab)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  roscpp
  std_msgs
)

add_service_files(
  FILES
    TransformValue.srv
)

add_action_files(
  FILES
    Count.action
)

generate_messages(
  DEPENDENCIES
    actionlib_msgs
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    actionlib
    actionlib_msgs
    message_runtime
    roscpp
    std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(ros1_comm_lab_core
  src/message_processor.cpp
)

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

add_executable(testable_node
  src/testable_node.cpp
)

target_link_libraries(testable_node
  ros1_comm_lab_core
  ${catkin_LIBRARIES}
)

add_executable(transform_service_server
  src/transform_service_server.cpp
)
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

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  catkin_add_gtest(test_message_processor
    test/test_message_processor.cpp
  )
  if(TARGET test_message_processor)
    target_link_libraries(test_message_processor
      ros1_comm_lab_core
    )
  endif()

  add_rostest_gtest(test_testable_node
    test/testable_node.test
    test/test_testable_node.cpp
  )
  if(TARGET test_testable_node)
    target_link_libraries(test_testable_node
      ${catkin_LIBRARIES}
    )
  endif()

  add_rostest_gtest(test_service_action
    test/service_action.test
    test/test_service_action.cpp
  )
  if(TARGET test_service_action)
    add_dependencies(test_service_action
      ${${PROJECT_NAME}_EXPORTED_TARGETS}
      ${catkin_EXPORTED_TARGETS}
    )
    target_link_libraries(test_service_action
      ${catkin_LIBRARIES}
    )
  endif()
endif()
