cmake_minimum_required(VERSION 3.0.2)
project(vr_pose_visualization)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
)

## 查找nlohmann/json（需先安装）
find_package(nlohmann_json REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vr_pose_visualization
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs tf2 tf2_ros
)

## 包含目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${nlohmann_json_INCLUDE_DIRS}
)

## 编译可执行文件
add_executable(vr_pose_server src/vr_pose_server.cpp)
target_link_libraries(vr_pose_server
  ${catkin_LIBRARIES}
  ${nlohmann_json_LIBRARIES}
  pthread  # 链接线程库
)

