# 2.8.8 required to use PROTOBUF_IMPORT_DIRS
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
find_package(OpenCV REQUIRED)
find_package (Eigen3 REQUIRED NO_MODULE)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${SDF_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}/../world_utils/msgs
  ${CMAKE_CURRENT_BINARY_DIR}/../camera_utils/msgs
  )



link_directories(
    ${GAZEBO_LIBRARY_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}../world_utils/msgs
    ${CMAKE_CURRENT_BINARY_DIR}../camera_utils/msgs
  )

# Camera utils plugin client
add_executable (
    capture_example
    capture_example.cc
    utils.cc)
target_link_libraries(
    capture_example
    camera_utils_msgs
    world_utils_msgs
    ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${SDF_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(
    capture_example
    camera_utils_msgs
    world_utils_msgs)
