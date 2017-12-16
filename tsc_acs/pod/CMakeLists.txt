cmake_minimum_required(VERSION 2.8.3)
project(pod)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  lutz
  message_generation
)
add_message_files(
  FILES
#  Demand.msg
  PodDemand.msg
  PodDemandSource.msg
#  ControlDemand.msg
  AuxiliaryDemand.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(xbox_joy src/XboxJoy.cpp src/XboxJoyNode.cpp)
target_link_libraries(xbox_joy ${catkin_LIBRARIES})
add_dependencies(xbox_joy lutz_generate_messages_cpp pod_generate_messages_cpp)

add_executable(turnigy_joy src/TurnigyJoy.cpp src/TurnigyJoyNode.cpp)
target_link_libraries(turnigy_joy ${catkin_LIBRARIES})
add_dependencies(turnigy_joy lutz_generate_messages_cpp pod_generate_messages_cpp arduino_generate_messages_cpp)

add_executable(control_switch src/ControlSwitch.cpp src/ControlSwitchNode.cpp)
target_link_libraries(control_switch ${catkin_LIBRARIES})
add_dependencies(control_switch lutz_generate_messages_cpp pod_generate_messages_cpp)

add_executable(auxiliary_switch src/AuxiliarySwitch.cpp src/AuxiliarySwitchNode.cpp)
target_link_libraries(auxiliary_switch ${catkin_LIBRARIES})
add_dependencies(auxiliary_switch lutz_generate_messages_cpp pod_generate_messages_cpp)

add_executable(key_entry
	src/KeyEntry.cpp
	src/KeyEntryNode.cpp
	)
target_link_libraries(key_entry ${catkin_LIBRARIES})
add_dependencies(key_entry std_msgs_cpp)

add_executable(dummy_acs
	src/DummyACSNode.cpp
	)
target_link_libraries(dummy_acs ${catkin_LIBRARIES} ncurses)
add_dependencies(dummy_acs lutz_generate_messages_cpp)


