#!/bin/bash

# Source gazebo environment (and surpress not found error)
source /usr/share/gazebo-7/setup.sh &>/dev/null
source /usr/share/gazebo-8/setup.sh &>/dev/null
# Export location of object spawner gazebo plugin
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:`pwd`/build/object_spawner
# Export location of camera tools gazebo plugin
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:`pwd`/build/camera_utils
# Export location of custom resources
export GAZEBO_RESOURCE_PATH=${GAZEBO_RESOURCE_PATH}:`pwd`/media
# Export location of custom models
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:`pwd`/models

echo "gazebo-utils was successfully initialised."
