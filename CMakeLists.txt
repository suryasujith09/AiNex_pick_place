cmake_minimum_required(VERSION 3.0.2)
project(pick_place)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
)

catkin_package()

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/interactive_game_final.py
  scripts/interactive_game.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)
