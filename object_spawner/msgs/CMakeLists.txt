find_package(Protobuf REQUIRED)

set(PROTOBUF_IMPORT_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()

set (msgs
  # Custom messages
  object_spawner_request.proto
  # Pose messages
  ${PROTOBUF_IMPORT_DIRS}/pose.proto
  ${PROTOBUF_IMPORT_DIRS}/vector3d.proto
  ${PROTOBUF_IMPORT_DIRS}/quaternion.proto
  
  # DEBUG, remove eventually
  # Might be useful to check the generated libraries?
  #${PROTOBUF_IMPORT_DIRS}/model.proto
  #${PROTOBUF_IMPORT_DIRS}/joint.proto
  #${PROTOBUF_IMPORT_DIRS}/axis.proto
  #${PROTOBUF_IMPORT_DIRS}/sensor.proto
  #${PROTOBUF_IMPORT_DIRS}/camerasensor.proto
  #${PROTOBUF_IMPORT_DIRS}/vector2d.proto
  #${PROTOBUF_IMPORT_DIRS}/distortion.proto
  #${PROTOBUF_IMPORT_DIRS}/raysensor.proto
  #${PROTOBUF_IMPORT_DIRS}/contactsensor.proto
  #${PROTOBUF_IMPORT_DIRS}/logical_camera_sensor.proto
  #${PROTOBUF_IMPORT_DIRS}/link.proto
  #${PROTOBUF_IMPORT_DIRS}/inertial.proto
  #${PROTOBUF_IMPORT_DIRS}/collision.proto
  #${PROTOBUF_IMPORT_DIRS}/geometry.proto
  #${PROTOBUF_IMPORT_DIRS}/boxgeom.proto
  #${PROTOBUF_IMPORT_DIRS}/cylindergeom.proto
  #${PROTOBUF_IMPORT_DIRS}/spheregeom.proto
  #${PROTOBUF_IMPORT_DIRS}/planegeom.proto
  #${PROTOBUF_IMPORT_DIRS}/imagegeom.proto
  #${PROTOBUF_IMPORT_DIRS}/heightmapgeom.proto
  #${PROTOBUF_IMPORT_DIRS}/image.proto
  #${PROTOBUF_IMPORT_DIRS}/meshgeom.proto
  #${PROTOBUF_IMPORT_DIRS}/polylinegeom.proto
  #${PROTOBUF_IMPORT_DIRS}/surface.proto
  #${PROTOBUF_IMPORT_DIRS}/friction.proto
  #${PROTOBUF_IMPORT_DIRS}/visual.proto
  #${PROTOBUF_IMPORT_DIRS}/plugin.proto
  #${PROTOBUF_IMPORT_DIRS}/projector.proto
  #${PROTOBUF_IMPORT_DIRS}/battery.proto

  ${PROTOBUF_IMPORT_DIRS}/header.proto
  ${PROTOBUF_IMPORT_DIRS}/time.proto
)
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})
add_library(object_spawner_msgs SHARED ${PROTO_SRCS})
target_link_libraries(object_spawner_msgs ${PROTOBUF_LIBRARY})
