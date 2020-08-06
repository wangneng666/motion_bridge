cmake_minimum_required(VERSION 3.0.2)
project(motion_bridge)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosmsg
  std_msgs
  moveit_core
  moveit_visual_tools
  moveit_ros_planning
  moveit_ros_planning_interface
  hirop_msgs
    tf
#  pluginlib
#  geometric_shapes
#  pcl_ros
#  pcl_conversions
#  rosbag
#  tf2_ros
#  tf2_eigen
#  tf2_geometry_msgs
)
catkin_package(
CATKIN_DEPENDS message_runtime
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
  ${catkin_INCLUDE_DIRS}
)


add_executable(motion_bridge_exe src/main.cpp src/motion.cpp)

target_link_libraries(motion_bridge_exe
  ${catkin_LIBRARIES}
)

#add_executable(motion_bridge_exe_test src/motion_planning_api_tutorial_trajectory_demo.cpp)
#target_link_libraries(motion_bridge_exe_test
#  ${catkin_LIBRARIES}
#)

