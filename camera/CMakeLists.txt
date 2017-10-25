# 2.8.8 required to use PROTOBUF_IMPORT_DIRS
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

# Include and compile custom messages
include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${SDF_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/msgs
  )
link_directories(
    ${GAZEBO_LIBRARY_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}/msgs)
add_subdirectory(msgs)

# Gazebo generic camera utils plugin
add_library(camera_utils SHARED camera_utils.cc )
target_link_libraries(
    camera_utils
    camera_utils_msgs
    ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${SDF_LIBRARIES}
    CameraPlugin)
add_dependencies(
    camera_utils
    camera_utils_msgs)

# Camera utils plugin client
add_executable (
    camera_client
    camera_client.cc)
target_link_libraries(
    camera_client
    camera_utils_msgs
    ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${SDF_LIBRARIES})
add_dependencies(
    camera_client
    camera_utils_msgs)