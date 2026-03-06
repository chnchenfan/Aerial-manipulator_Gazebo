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
  std_msgs
  geometry_msgs
  mavros_msgs
  tf2_ros
  tf
)

## 查找 Eigen3 (矩阵运算库)
find_package(Eigen3 REQUIRED)

## 3. 导出当前包的依赖关系
## 这里的 LIBRARIES 导出了我们将要生成的 uav_core_lib
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES uav_core_lib
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs mavros_msgs tf2_ros
  # 注意：为了避免之前的 CMake Warning，通常不需要在这里写 DEPENDS Eigen3
  # 只要在下面的 include_directories 里包含了 EIGEN3_INCLUDE_DIRS 即可
)

## 4. 指定头文件路径
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}  # <--- 必须包含 Eigen 的头文件路径
)

## ==========================================================
## 5.【核心库】编译算法实现 (Library)
## 把 ESO 和 控制器的数学逻辑编译成一个库，方便复用
## ==========================================================
add_library(uav_core_lib
  src/ControlUtils.cpp
  # src/PositionESO.cpp
  # src/PositionController.cpp
  src/TrajectoryGenerator.cpp
  # src/AttitudeESO.cpp
)
## 链接库需要的依赖
target_link_libraries(uav_core_lib
  ${catkin_LIBRARIES}
)


## ==========================================================
## 6.【测试节点】编译 ESO 测试/采集程序 (Test Executable)
## 这是放在 test 文件夹里的实验代码
## ==========================================================
# add_executable(eso_data_collector test/eso_data_collector.cpp)
# target_link_libraries(eso_data_collector
#   uav_core_lib       # 复用核心库
#   ${catkin_LIBRARIES}
# )

# ## 闭环悬停/控制测试节点
# add_executable(position_control_test test/position_control_test.cpp)
# target_link_libraries(position_control_test
#   uav_core_lib
#   ${catkin_LIBRARIES}
# )


## 安装 Python 脚本，便于 rosrun 调用
catkin_install_python(PROGRAMS
  scripts/plot_eso_results.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
