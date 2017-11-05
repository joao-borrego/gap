find_package(Protobuf REQUIRED)

set(PROTOBUF_IMPORT_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()

set (msgs
  # Custom messages
  world_utils_request.proto
  world_utils_response.proto
  # Message dependencies
  ${PROTOBUF_IMPORT_DIRS}/pose.proto
  ${PROTOBUF_IMPORT_DIRS}/vector3d.proto
  ${PROTOBUF_IMPORT_DIRS}/quaternion.proto
  ${PROTOBUF_IMPORT_DIRS}/header.proto
  ${PROTOBUF_IMPORT_DIRS}/time.proto
)
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})
add_library(world_utils_msgs SHARED ${PROTO_SRCS})
target_link_libraries(world_utils_msgs ${PROTOBUF_LIBRARY})
