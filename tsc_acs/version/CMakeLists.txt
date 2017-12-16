cmake_minimum_required(VERSION 2.8.3)
project(bacs_version)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

execute_process(
    COMMAND python ${CMAKE_CURRENT_SOURCE_DIR}/get_git_info.py
)

add_message_files(
  FILES
  Version.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(version src/version.cpp)
target_link_libraries(version ${catkin_LIBRARIES})
add_dependencies(version bacs_version_generate_messages_cpp)

