cmake_minimum_required(VERSION 2.8.3)
project(ibeo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
)
add_message_files(
  FILES
  FusionObject.msg
  FusionObjectList.msg
  FusionScan.msg
  IbeoHeader.msg
  Message.msg
  Point2D32.msg
  Resolution.msg
  ScannerInfo.msg
  ScanPoint.msg
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
  geometry_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(tcp
	src/Tcp.cpp
	src/TcpNode.cpp)
target_link_libraries(tcp ${catkin_LIBRARIES})
add_dependencies(tcp ibeo_generate_messages_cpp)

add_executable(fusion
	src/Fusion.cpp
	src/FusionNode.cpp
	src/FusionNode.Objects.cpp
	src/FusionNode.Scan.cpp)
target_link_libraries(fusion ${catkin_LIBRARIES})
add_dependencies(fusion ibeo_generate_messages_cpp)

add_executable(object_marker
	src/ObjectMarker.cpp
	src/ObjectMarkerNode.cpp)
target_link_libraries(object_marker ${catkin_LIBRARIES})
add_dependencies(object_marker ibeo_generate_messages_cpp)

add_executable(point_cloud_file
	src/PointCloudFile.cpp)
target_link_libraries(point_cloud_file ${catkin_LIBRARIES})

add_executable(point_cloud_to_float
	src/PointCloudToFloat.cpp)
target_link_libraries(point_cloud_to_float ${catkin_LIBRARIES})

