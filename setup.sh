#!/bin/bash

# Source gazebo environment
# Source installation
source /usr/local/share/gazebo-9/setup.sh 2> /dev/null
# deb installation
source /usr/share/gazebo-9/setup.sh 2> /dev/null

# Libraries

# Export location of gazebo plugins
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:`pwd`/build/lib

# Resources

# Export location of custom resources
export GAZEBO_RESOURCE_PATH=${GAZEBO_RESOURCE_PATH}:/DATA/Gazebo
# Export location of custom models
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:`pwd`/models

echo "GAP was successfully initialised."
