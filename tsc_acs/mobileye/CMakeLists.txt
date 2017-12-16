cmake_minimum_required(VERSION 2.8.3)
project(mobileye)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)
add_message_files(
  FILES
  ObstacleStatus.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(mobileye
	src/Mobileye.cpp
	src/MobileyeNode.cpp
	src/MobileyeNode.ObstacleStatus.cpp)
target_link_libraries(mobileye ${catkin_LIBRARIES})
add_dependencies(mobileye mobileye_generate_messages_cpp socketcan_bridge)
