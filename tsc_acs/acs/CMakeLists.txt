cmake_minimum_required(VERSION 2.8.3)
project(acs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
  pod
  oxts
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/AutoDemand.cfg
  cfg/PathReplay.cfg
)

add_message_files(
  FILES
  AutoDemandCfg.msg
  PathReplayCfg.msg
  Pose2DStamped.msg
  Goal.msg
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
  geometry_msgs
  nav_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(path_replay
	src/GpsPathReplay.cpp
	src/GpsPathReplayNode.cpp
	src/GpsPath.cpp)
target_link_libraries(path_replay ${catkin_LIBRARIES})
add_dependencies(path_replay acs_generate_messages_cpp)

add_executable(auto_demand
	src/AutoDemand.cpp
	src/AutoDemandNode.cpp)
target_link_libraries(auto_demand ${catkin_LIBRARIES})
add_dependencies(auto_demand lutz_generate_messages_cpp)
add_dependencies(auto_demand ${PROJECT_NAME}_gencfg)

add_executable(steer_smoother
	src/SteerSmoother.cpp
	src/SteerSmootherNode.cpp)
target_link_libraries(steer_smoother ${catkin_LIBRARIES})
add_dependencies(steer_smoother pod_generate_messages_cpp)

add_executable(gps_to_local
	src/GpsToLocal.cpp
	src/GpsToLocalNode.cpp
	src/GpsPath.cpp)
target_link_libraries(gps_to_local ${catkin_LIBRARIES})
add_dependencies(gps_to_local
	acs_generate_messages_cpp
	oxts_generate_messages_cpp)
	
add_executable(record_route
	src/RecordRoute.cpp
	src/RecordRouteNode.cpp
	src/GpsPath.cpp)
target_link_libraries(record_route ${catkin_LIBRARIES})
add_dependencies(record_route
	oxts_generate_messages_cpp)