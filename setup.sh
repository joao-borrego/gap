#!/bin/bash

# Source gazebo environment (and surpress not found error)
source /usr/share/gazebo-7/setup.sh &>/dev/null
source /usr/share/gazebo-8/setup.sh &>/dev/null
# Export location of object spawner gazebo plugin
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:`pwd`/object_spawner/build
# Export location of custom resources
export GAZEBO_RESOURCE_PATH=${GAZEBO_RESOURCE_PATH}:`pwd`/media
