cmake_minimum_required(VERSION 3.0.2)
project(ros1_slam_lab)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/simple_laser_world.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  config
  launch
  maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
