cmake_minimum_required(VERSION 3.8)
project(boldbot_gazebo_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ignition-gazebo6 REQUIRED)

set(plugin_name BoldbotGazeboPlugin)
add_library(${plugin_name} SHARED
  src/boldbot_gazebo_plugin.cpp)

target_link_libraries(${plugin_name}
  ignition-gazebo6::core)

install(
  TARGETS ${plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_package()
