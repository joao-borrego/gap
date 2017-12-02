# 2.8.8 required to use PROTOBUF_IMPORT_DIRS
cmake_minimum_required(VERSION 2.8.8 FATAL_ERROR)

# Project name
project(GazeboUtils)

# Project version
set (GAZEBO_UTILS_MAJOR_VERSION 0)
set (GAZEBO_UTILS_MINOR_VERSION 1)
set (GAZEBO_UTILS_PATCH_VERSION 0)
set (GAZEBO_UTILS_VERSION
	${GAZEBO_UTILS_MAJOR_VERSION}.${GAZEBO_UTILS_MINOR_VERSION})
set (GAZEBO_UTILS_FULL_VERSION
	${GAZEBO_UTILS_MAJOR_VERSION}.${GAZEBO_UTILS_MINOR_VERSION}.${GAZEBO_UTILS_PATCH_VERSION})

# Additional cmake modules
set(CMAKE_MODULE_PATH
	${CMAKE_MODULE_PATH}
	${CMAKE_CURRENT_SOURCE_DIR}/cmake
	)

# Cmake safeguards e.g. prevent in-source builds
include(safeguards)

# Check dependencies
find_package(gazebo REQUIRED)
find_package(Boost 1.40 COMPONENTS system REQUIRED)

# Build project

# Messages

add_subdirectory(msgs)

# Plugins

# World plugin
add_subdirectory(world_utils)
# Camera plugin
add_subdirectory(camera_utils)
# Pattern generation tool
add_subdirectory(pattern_generation)

# Examples

# Generate objects and capture picture
add_subdirectory(capture_example)