cmake_minimum_required(VERSION 3.0.2)
project(uav_arm_top)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  sensor_msgs
  std_msgs
  mavros_msgs
  uav_control
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy geometry_msgs sensor_msgs std_msgs mavros_msgs uav_control
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(eso_circle_offboard_node
  src/eso_circle_offboard_node.cpp
)
target_link_libraries(eso_circle_offboard_node
  ${catkin_LIBRARIES}
)

add_executable(eso_hover_disturbance_offboard_node
  src/eso_hover_disturbance_offboard_node.cpp
)
target_link_libraries(eso_hover_disturbance_offboard_node
  ${catkin_LIBRARIES}
)

add_executable(eso_square_arm_experiment_node
  src/eso_square_arm_experiment_node.cpp
)
target_link_libraries(eso_square_arm_experiment_node
  ${catkin_LIBRARIES}
)

install(TARGETS eso_circle_offboard_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS eso_hover_disturbance_offboard_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS eso_square_arm_experiment_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/uam_v5_arm_joint_state_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
