cmake_minimum_required(VERSION 2.8.3)
project(sim_pod)

find_package(
  catkin
  REQUIRED COMPONENTS
  roscpp
  std_msgs
  pod
  acs
  oxts
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(sim_ple
	src/PodSim.cpp
	src/PodSimNode.cpp
	../acs/src/GpsPath.cpp)
target_link_libraries(sim_ple ${catkin_LIBRARIES})
add_dependencies(sim_ple pod_generate_messages_cpp)

add_executable(o_demand
	src/CircleDemand.cpp)
target_link_libraries(o_demand ${catkin_LIBRARIES})
add_dependencies(o_demand
	pod_generate_messages_cpp
	oxts_generate_messages_cpp )
