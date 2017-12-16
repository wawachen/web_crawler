cmake_minimum_required(VERSION 2.8.3)
project(lutz)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  sound_play
)
add_message_files(
  FILES
  PodHeader.msg
  
  AuxPower.msg
  Battery.msg
  Debug.msg
  EPS.msg
  FaultCodes.msg
  Handshake.msg
  Imu1.msg
  Imu2.msg
  Indicator.msg
  ParkBrake.msg
  Powertrain.msg
  Status.msg
  Steer.msg
  Ultrasonics.msg
  Version.msg
  WheelCount.msg
  
  AuxPowerRequest.msg
  ControlCommand1.msg
  ControlCommand2.msg
  
  SessionControl.msg
  AutonomousRequest.msg
  UICode.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(pod_rx
	src/PodRx.cpp
	src/PodRxNode.cpp
	src/PodRxNode.AuxPower.cpp
	src/PodRxNode.Battery.cpp
	src/PodRxNode.Debug.cpp
	src/PodRxNode.EPS.cpp
	src/PodRxNode.FaultCodes.cpp
	src/PodRxNode.Handshake.cpp
	src/PodRxNode.Imu.cpp
	src/PodRxNode.Indicator.cpp
	src/PodRxNode.ParkBrake.cpp
	src/PodRxNode.Powertrain.cpp
	src/PodRxNode.Status.cpp
	src/PodRxNode.Steer.cpp
	src/PodRxNode.Ultrasonics.cpp
	src/PodRxNode.Version.cpp
	src/PodRxNode.WheelCount.cpp)
target_link_libraries(pod_rx ${catkin_LIBRARIES})
add_dependencies(pod_rx lutz_generate_messages_cpp socketcan_bridge)

add_executable(pod_tx
	src/PodTx.cpp
	src/PodTxNode.cpp
	src/PodTxNode.AuxPowerRequest.cpp
	src/PodTxNode.ControlCommand1.cpp
	src/PodTxNode.ControlCommand2.cpp
	src/PodTxNode.Handshake.cpp)
target_link_libraries(pod_tx ${catkin_LIBRARIES})
add_dependencies(pod_tx lutz_generate_messages_cpp socketcan_bridge)

add_executable(lutz_version
	src/Version.cpp)
target_link_libraries(lutz_version ${catkin_LIBRARIES})
add_dependencies(lutz_version lutz_generate_messages_cpp socketcan_bridge)

add_executable(handshake
	src/Handshake.cpp
	src/HandshakeNode.cpp
	)
target_link_libraries(handshake ${catkin_LIBRARIES})
add_dependencies(handshake lutz_generate_messages_cpp socketcan_bridge)

add_executable(message_builder
	src/MessageBuilder.cpp
	src/MessageBuilderNode.cpp
	)
target_link_libraries(message_builder ${catkin_LIBRARIES})
add_dependencies(message_builder lutz_generate_messages_cpp)
add_dependencies(message_builder pod_generate_messages_cpp)

add_executable(session_control
	src/SessionControl.cpp
	src/SessionControlNode.cpp
	src/SessionControlNode.EnterAutonomous.cpp
	src/SessionControlNode.StateMachine.cpp
#	src/SessionControlNode.Handshake.cpp
	)
target_link_libraries(session_control ${catkin_LIBRARIES})
add_dependencies(session_control lutz_generate_messages_cpp)

add_executable(console_ui
	src/ConsoleUI.cpp
	src/ConsoleUINode.cpp
	)
target_link_libraries(console_ui ${catkin_LIBRARIES})
add_dependencies(console_ui lutz_generate_messages_cpp)

add_executable(text2speech
	src/Text2Speech.cpp
	src/Text2SpeechNode.cpp
	)
target_link_libraries(text2speech ${catkin_LIBRARIES})
add_dependencies(text2speech sound_play lutz_generate_messages_cpp)

add_executable(dummy_autonomous_request
	src/DummyAutoRequestNode.cpp
	)
target_link_libraries(dummy_autonomous_request ${catkin_LIBRARIES})
add_dependencies(dummy_autonomous_request lutz_generate_messages_cpp)

add_executable(on_axle_stands
	src/OnAxleStands.cpp
	src/OnAxleStandsNode.cpp
	)
target_link_libraries(on_axle_stands ${catkin_LIBRARIES})
add_dependencies(on_axle_stands pod_generate_messages_cpp std_msgs_cpp)
