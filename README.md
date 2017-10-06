# gazebo-utils
Set of tools to interact with gazebo simulator

### Object spawner

#### Compilation

Clone the repository to your workspace directory.

```
cd ~/workspace/gazebo-utils/object-spawner/build &&
cmake ../ &&
make
```

#### Executing

```
cd ~/workspace/gazebo-utils/object-spawner/ &&
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:`pwd`/build &&
gzserver spawner.world --verbose
```
