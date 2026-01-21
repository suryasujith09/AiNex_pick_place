cmake_minimum_required(VERSION 3.0.2)
project(pick_place)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/interactive_game.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
