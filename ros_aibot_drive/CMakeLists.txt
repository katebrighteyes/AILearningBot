cmake_minimum_required(VERSION 3.0.2)
project(ros_aibot_drive)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  nav_msgs 
  sensor_msgs
  message_filters 
  tf 
  dynamic_reconfigure  
)

add_message_files(
  FILES 
)

generate_messages(
  DEPENDENCIES 
  std_msgs
  nav_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES can_ctl_ros
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
  CATKIN_DEPENDS 
  message_runtime 
  message_filters
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}/src

)






