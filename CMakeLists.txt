cmake_minimum_required(VERSION 2.8.3)
project(my_package)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  cv_bridge
  geometry_msgs
  apriltag_ros
)

catkin_package()

catkin_install_python(PROGRAMS
  src/white_line_follower_node.py
  src/apriltag_detector_node.py
  src/tag_action_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
