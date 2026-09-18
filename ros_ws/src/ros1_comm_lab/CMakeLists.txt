cmake_minimum_required(VERSION 3.0.2)
project(ros1_comm_lab)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
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
