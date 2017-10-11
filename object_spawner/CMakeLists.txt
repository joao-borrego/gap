# 2.8.8 required to use PROTOBUF_IMPORT_DIRS
cmake_minimum_required(VERSION 2.8.8 FATAL_ERROR)
FIND_PACKAGE( Boost 1.40 COMPONENTS system REQUIRED )
set (CMAKE_CXX_FLAGS "-g -Wall -std=c++11")

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
  pkg_check_modules(SDF sdformat)
endif()
include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${SDF_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/msgs
  )
link_directories(
    ${GAZEBO_LIBRARY_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}/msgs)
add_subdirectory(msgs)

# Spawner client
add_executable (
    spawner_client
    spawner_client.cc)
target_link_libraries(
    spawner_client
    object_spawner_msgs
    ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${SDF_LIBRARIES})
add_dependencies(
    spawner_client
    object_spawner_msgs)

# Spawner gazebo server plugin
add_library(object_spawner SHARED object_spawner.cc )
target_link_libraries(
    object_spawner
    object_spawner_msgs
    ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${SDF_LIBRARIES})
add_dependencies(
    object_spawner
    object_spawner_msgs)