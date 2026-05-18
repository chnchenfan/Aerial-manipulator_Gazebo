cmake_minimum_required(VERSION 3.0.2)
project(uav_control)

## 1. 编译选项：使用 C++14 标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## 2. 查找依赖库
## 查找 ROS 核心组件
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rosbag
  std_msgs
  geometry_msgs
  sensor_msgs
  mavros_msgs
  tf2_ros
  tf
)

## 查找 Eigen3 (矩阵运算库)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy rosbag std_msgs geometry_msgs sensor_msgs mavros_msgs tf2_ros
)

## 4. 指定头文件路径
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}  # <--- 必须包含 Eigen 的头文件路径
)
