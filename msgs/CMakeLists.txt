# 2.8.8 required to use PROTOBUF_IMPORT_DIRS
cmake_minimum_required(VERSION 2.8.8 FATAL_ERROR)

# Messages

# Messages for WorldUtils plugin
add_subdirectory(WorldUtils)
# Messages for CameraUtils plugin
add_subdirectory(CameraUtils)