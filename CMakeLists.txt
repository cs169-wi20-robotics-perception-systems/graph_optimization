cmake_minimum_required(VERSION 2.8.3)
project(graph_optimization)

find_package(catkin REQUIRED
  g2o
  rospy
  rospkg
  message_filters
  numpy
)

catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
   scripts/simple_graph_optimization.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
