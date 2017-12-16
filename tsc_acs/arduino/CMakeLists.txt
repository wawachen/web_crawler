cmake_minimum_required(VERSION 2.8.3)
project(arduino)

find_package(catkin REQUIRED COMPONENTS
  rosserial_arduino 
  rosserial_client
  std_msgs
  message_generation
)

add_message_files(
  FILES
  rcReader.msg
  arduinoDiagnostics.msg
  deadMansHandleReading.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs rosserial_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/Arduino)
  file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/Arduino)
  file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/Arduino/libraries) 
endif()


file(REMOVE_RECURSE ${CMAKE_CURRENT_SOURCE_DIR}/Arduino/libraries/ros_lib)

execute_process(
    COMMAND rosrun rosserial_arduino make_libraries.py ${CMAKE_CURRENT_SOURCE_DIR}/Arduino/libraries/
)

rosserial_generate_ros_lib(
  PACKAGE rosserial_arduino
  SCRIPT make_libraries.py
)

rosserial_configure_client(
  DIRECTORY src
  TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
)

rosserial_add_client_target(src arduino ALL)
rosserial_add_client_target(src arduino-upload)

