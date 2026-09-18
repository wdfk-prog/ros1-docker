cmake_minimum_required(VERSION 3.0.2)
project(ros1_comm_lab)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package()

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
endif()
