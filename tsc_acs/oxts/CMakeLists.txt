cmake_minimum_required(VERSION 2.8.3)
project(oxts)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  udp
  message_generation
)
add_message_files(
  FILES
  GnssInfo.msg
  KalmanInnovation.msg
  ReceiverInfo.msg
  AntennaPosition.msg
  BatchB.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  udp
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(xnav550 src/xNav550.cpp src/xNav550Node.cpp)
target_link_libraries(xnav550 ${catkin_LIBRARIES})
add_dependencies(xnav550 udp_generate_messages_cpp)
add_dependencies(xnav550 oxts_generate_messages_cpp)
