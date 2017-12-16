cmake_minimum_required(VERSION 2.8.3)
project(demand_limiter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  dead_mans_handle
  pod
)
#add_message_files(
#  FILES
#
#)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS pod message_runtime std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(demandLimiter
	src/demandLimiter.cpp
	src/demandLimiterNode.cpp
	)
target_link_libraries(demandLimiter ${catkin_LIBRARIES})
add_dependencies(demandLimiter pod_generate_messages_cpp dead_mans_handle_generate_messages_cpp std_msgs_cpp)

