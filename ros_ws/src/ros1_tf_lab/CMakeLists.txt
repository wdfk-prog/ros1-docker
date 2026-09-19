cmake_minimum_required(VERSION 3.0.2)
project(ros1_tf_lab)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  tf2
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
    roscpp
    tf2
    tf2_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(odom_tf_broadcaster
  src/odom_tf_broadcaster.cpp
)

target_link_libraries(odom_tf_broadcaster
  ${catkin_LIBRARIES}
)

add_executable(tf_query_node
  src/tf_query_node.cpp
)

target_link_libraries(tf_query_node
  ${catkin_LIBRARIES}
)
