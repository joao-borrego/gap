# gazebo-utils
Set of tools to interact with Gazebo simulator

This project comprises a set of tools for Gazebo which provide an interface to interact
programatically with the simulator.
These tools communicate directly with Gazebo server, not depending on any ROS modules.
It includes:

- [Object spawner plugin], that allows you to spawn boxes, spheres and cylinders, as well as custom models either by
a uri reference or directly with an sdf string. Furthermore, it allows a model to be rendered with custom textures.

- [Camera plugin], which creates an interface to capture frames at specific instants.

- [Pattern generation tool], which can randomly generate a high number of different types of textures,
that are ready to be rendered in Gazebo.

### Dependencies

The code has been tested in Gazebo 7.8.1 running on Ubuntu 16.04.

Gazebo internal message passing relies on Protobuf, which is why the compiler needs to be installed in order
to generate the tools' custom messages.

```
sudo apt install protobuf-compiler
```

The pattern generation tool depends on OpenCV.

### Compilation

Clone the repository to your workspace directory and build from source.

```
cd ~/workspace/gazebo-utils/ &&
mkdir build &&
cd build &&
cmake ../ &&
make
```

Alternatively you can build each plugin/tool individually in a similar fashion.

### Initialization

Make sure you properly initialise the required environment variables.
We provide a simple script for this:

```
cd ~/workspace/gazebo-utils &&
source setup.sh
```


[Object spawner plugin]: object_spawner
[Camera plugin]: camera
[Pattern generation tool]: pattern_generation