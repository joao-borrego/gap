# 2.8.8 required to use PROTOBUF_IMPORT_DIRS
cmake_minimum_required(VERSION 2.8.8 FATAL_ERROR)

# Find Protobuf package
find_package(Protobuf REQUIRED)

# Obtain Protobuf import directories
set(PROTOBUF_IMPORT_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()

# Obtain set of proto messages definitions
set (msgs
  # CameraUtils messages
  camera_utils_request.proto
  camera_utils_response.proto
  point_projection.proto
  # VisualUtils messages
  visual_utils_request.proto
  visual_utils_response.proto
  # WorldUtils Messages
  world_utils_request.proto
  world_utils_response.proto
  object.proto
  # Message dependencies
  ${PROTOBUF_IMPORT_DIRS}/header.proto
  ${PROTOBUF_IMPORT_DIRS}/time.proto
  ${PROTOBUF_IMPORT_DIRS}/quaternion.proto
  ${PROTOBUF_IMPORT_DIRS}/pose.proto
  ${PROTOBUF_IMPORT_DIRS}/vector2d.proto
  ${PROTOBUF_IMPORT_DIRS}/vector3d.proto
)

# Generate Protobuf sources
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})
# Compile shared library
add_library(gap_msgs SHARED ${PROTO_SRCS})
# Link Protobuf library
target_link_libraries(gap_msgs ${PROTOBUF_LIBRARY})

# Install library
install(TARGETS gap_msgs
  DESTINATION "${msgs_lib_dest}")
install(FILES
  ${PROTO_HDRS} DESTINATION "${msgs_include_dest}")
