cmake_minimum_required(VERSION 3.0.2)
project(arm_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  controller_manager
  joint_state_controller
  effort_controllers
  trajectory_msgs
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/joint_position_commander.py
  scripts/joint_position_commander_uam_v5.py
  scripts/arm_zero_hold_logger_uam_v5.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
