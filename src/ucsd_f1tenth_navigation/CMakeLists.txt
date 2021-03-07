cmake_minimum_required(VERSION 3.0.2)
project(ucsd_f1tenth_navigation_v2)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS
  scripts/follow_waypoints.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
