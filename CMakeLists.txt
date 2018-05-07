# 2.8.8 required to use PROTOBUF_IMPORT_DIRS
cmake_minimum_required(VERSION 2.8.8 FATAL_ERROR)

# Project name
project(GAP)
string (TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)
string (TOUPPER ${PROJECT_NAME} PROJECT_NAME_UPPER)

# Project version
set (GAP_MAJOR_VERSION 1)
set (GAP_MINOR_VERSION 0)
set (GAP_PATCH_VERSION 0)
set (GAP_VERSION
	${GAP_MAJOR_VERSION}.${GAP_MINOR_VERSION})
set (GAP_FULL_VERSION
	${GAP_MAJOR_VERSION}.${GAP_MINOR_VERSION}.${GAP_PATCH_VERSION})

# Additional cmake modules
set(CMAKE_MODULE_PATH
	${CMAKE_MODULE_PATH}
	${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# CMake safeguards e.g. prevent in-source builds
include(safeguards)

# Check dependencies
find_package(gazebo REQUIRED)
find_package(Boost COMPONENTS system REQUIRED)

# Setup directories for library installation

set (msgs_include_dest "include/gap-${GAP_VERSION}")
set (msgs_lib_dest "lib/gap-${GAP_VERSION}")

# Build project

# Messages

add_subdirectory(msgs)

# Plugins

# Camera plugin
add_subdirectory(camera_utils)
# Visual plugin
add_subdirectory(visual_utils)
# World plugin
add_subdirectory(world_utils)

# Examples

# Camera tools example client
add_subdirectory(examples/camera_example)
# Visual tools example client
add_subdirectory(examples/visual_example)
# World tools example client
add_subdirectory(examples/world_example)
# Random scene generator
add_subdirectory(examples/scene_example)

