cmake_minimum_required(VERSION 3.0.2)
project(ros1_navigation_lab)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY
  config
  launch
  rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
