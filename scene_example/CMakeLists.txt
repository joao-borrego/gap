# 2.8.8 required to use PROTOBUF_IMPORT_DIRS
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
#find_package(PCL REQUIRED)

#include_directories(include ${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})


include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${SDF_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PROJECT_BINARY_DIR}/msgs/CameraUtils
  ${PROJECT_BINARY_DIR}/msgs/VisualUtils
  ${PROJECT_BINARY_DIR}/msgs/WorldUtils
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${PROJECT_BINARY_DIR}/msgs/CameraUtils
  ${PROJECT_BINARY_DIR}/msgs/VisualUtils
  ${PROJECT_BINARY_DIR}/msgs/WorldUtils
)

# Camera utils plugin client
add_executable (
  scene_example
  scene_example.cc
  utils.cc
  object.cc)
target_link_libraries(
  scene_example
  camera_utils_msgs
  visual_utils_msgs
  world_utils_msgs
  ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${SDF_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(
  scene_example
  camera_utils_msgs
  visual_utils_msgs
  world_utils_msgs)
