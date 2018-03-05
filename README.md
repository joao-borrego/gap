# gazebo-utils
Set of tools to interact with Gazebo simulator

This project comprises a set of tools for Gazebo which provide an interface to interact
programatically with the simulator.
These tools communicate directly with Gazebo server, not depending on any ROS modules.
It includes:

- [Camera Utils plugin], to control camera objects, namely moving the camera and saving rendered frames at specific instants.

- [Visual Utils plugin], to control the visual appearance of an object during simulation, including changing the Visual object's pose, material and scale.

- [World Utils plugin], that allows you to spawn boxes, spheres and cylinders, as well as custom models either by
a uri reference or directly with an sdf string. Furthermore, it allows a model to be rendered with custom textures.

- [Pattern generation tool], which can randomly generate a high number of different types of textures,
that are ready to be rendered in Gazebo.

### Examples

Check out the [examples] to check out what you can achieve with these plugins.

### Dependencies

The code has been tested in Gazebo 9.0.0 built from source and running on Ubuntu 16.04.

Gazebo internal message passing relies on Protobuf, which is why the compiler needs to be installed in order
to generate the tools' custom messages.

```
sudo apt install protobuf-compiler
```

OpenCV 2 is required for the pattern generation tool and a few debug options.

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

[examples]: examples
[Camera Utils plugin]: camera_utils
[Visual Utils plugin]: visual_utils
[World Utils plugin]: world_utils
[Pattern generation tool]: pattern_generation
