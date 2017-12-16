cmake_minimum_required(VERSION 2.8.3)
project(dead_mans_handle)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)
add_message_files(
  FILES
  SpeedLimit.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(deadMansHandle
	src/deadMansHandle.cpp
	src/deadMansHandleNode.cpp
	)
target_link_libraries(deadMansHandle ${catkin_LIBRARIES})
add_dependencies(deadMansHandle dead_mans_handle_generate_messages_cpp)
add_dependencies(deadMansHandle arduino_generate_messages_cpp)

add_executable(speedLimitOnAxleStands
	src/SpeedLimitOnAxleStands.cpp
	src/SpeedLimitOnAxleStandsNode.cpp
	)
target_link_libraries(speedLimitOnAxleStands ${catkin_LIBRARIES})
add_dependencies(speedLimitOnAxleStands dead_mans_handle_generate_messages_cpp)
add_dependencies(speedLimitOnAxleStands arduino_generate_messages_cpp)
