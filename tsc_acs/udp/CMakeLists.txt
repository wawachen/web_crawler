cmake_minimum_required(VERSION 2.8.3)
project(udp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)
add_message_files(
  FILES
  Packet.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(client src/Client.cpp src/ClientNode.cpp)
target_link_libraries(client ${catkin_LIBRARIES})
add_dependencies(client udp_generate_messages_cpp)
