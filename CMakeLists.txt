cmake_minimum_required(VERSION 3.0)

# Project name
project(gap VERSION 1.0.0)
message (STATUS "${PROJECT_NAME} version: ${PROJECT_VERSION}")

# Additional cmake modules
set(CMAKE_MODULE_PATH
    ${CMAKE_MODULE_PATH}
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# CMake safeguards e.g. prevent in-source builds
include(safeguards)

# Check dependencies
find_package(Boost COMPONENTS system REQUIRED)
find_package(gazebo REQUIRED)
find_package(Protobuf REQUIRED)

# Setup directories

# Setup directories for library installation

set (project_major "${PROJECT_NAME}-${PROJECT_VERSION_MAJOR}")
set (msgs_include_dest "include/${project_major}")
set (msgs_lib_dest "lib/${project_major}")

# Add uninstall command
add_custom_target(uninstall
    "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/uninstall.cmake"
)

# Build project

# Messages
add_subdirectory(msgs)
# Plugins
add_subdirectory(plugins)
# Examples
add_subdirectory(examples)
