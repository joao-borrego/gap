# 2.8.8 required to use PROTOBUF_IMPORT_DIRS
cmake_minimum_required(VERSION 2.8.8 FATAL_ERROR)
FIND_PACKAGE( Boost COMPONENTS system REQUIRED )
FIND_PACKAGE( OpenCV REQUIRED )
set (CMAKE_CXX_FLAGS "-g -Wall -std=c++11")

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
  pkg_check_modules(SDF sdformat)
endif()
include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${SDF_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/msgs
  )


include_directories(include)
# Spawner gazebo server plugin
add_library(pattern_generation SHARED src/PatternGeneration.cpp src/PerlinNoise.cpp)
target_link_libraries(
    pattern_generation
    ${Boost_LIBRARIES} ${OpenCV_LIBS})


# Spawner client
add_executable (
    pattern_generation_test
    src/tests/pattern_generation_test.cpp)
target_link_libraries(
    pattern_generation_test
    pattern_generation
    ${Boost_LIBRARIES} ${OpenCV_LIBS})
