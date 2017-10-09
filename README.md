# gazebo-utils
Set of tools to interact with gazebo simulator

### Object spawner

This utility allows models to be added and removed programatically during simulation execution.

#### Dependencies

Of course, you must have Gazebo installed.

Protobuf (Google's Protocol Buffers) compiler is needed to compile custom messages.

```
sudo apt install protobuf-compiler
```

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
