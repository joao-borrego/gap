# Temporary fix to include libraries from external project
# until a proper package is created

# TODO - Create package
# Manually set path to GAP include directories e.g.
# set (GAP_INCLUDE_DIRS ~/workspace/gap/build)
set (GAP_INCLUDE_DIRS path/to/gap/binaries)

# 2.8.8 required to use PROTOBUF_IMPORT_DIRS
cmake_minimum_required(VERSION 2.8.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

# Find packages
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
  pkg_check_modules(SDF sdformat)
endif()

# Include and link Gazebo and SDF libraries
include_directories(${GAZEBO_INCLUDE_DIRS} ${SDF_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

# Include and link GAP libraries
include_directories(${GAP_INCLUDE_DIRS}/msgs)
link_directories(${GAP_INCLUDE_DIRS}/msgs)

# External example client
add_executable (external_example external_example.cc)
target_link_libraries(external_example
    gap_msgs
    ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${SDF_LIBRARIES})
