cmake_minimum_required(VERSION 2.8.3)
project(age_checker)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pod
)
#add_message_files(
#  FILES
#
#)

#generate_messages(
#  DEPENDENCIES
#  std_msgs
#)

catkin_package(
  CATKIN_DEPENDS pod message_runtime std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(ageChecker
	src/ageChecker.cpp
	src/ageCheckerNode.cpp
	)
target_link_libraries(ageChecker ${catkin_LIBRARIES})
add_dependencies(ageChecker pod_generate_messages_cpp std_msgs_cpp)

