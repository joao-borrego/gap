<p align="center"> 
    <img src=.image/logo.png>
</p>

--------------------

This project comprises a set of tools for [Gazebo] which provide an interface to interact programatically with the simulator.
These tools communicate directly with Gazebo server, not depending on any ROS modules.
It includes:

- [Camera Utils plugin], to control camera objects, namely moving the camera and saving rendered frames at specific instants.

- [Visual Utils plugin], to control the visual appearance of an object during simulation, including changing the Visual object's pose, material and scale.

- [World Utils plugin], that allows you to spawn models either by a uri reference or directly with an sdf string.
Furthermore, it allows a model to be rendered with custom textures.

This project was originally conceived so we could develop a [scene generator] in Gazebo, employing domain randomisation in an attempt to bridge the **reality gap** between real life images and synthetically generated frames.

### Examples

Check out the [examples] and see what you can achieve with these plugins.

### Documentation

Take a look at the automatic [documentation] for file and class description.


### Dependencies

The code has been tested in Gazebo 9.0.0 from the official stable repository as well as built from source and running on Ubuntu 16.04 and 18.04.

Gazebo internal message passing relies on Protobuf, which is why the compiler needs to be installed in order
to generate the tools' custom messages.
Eigen 3 is required for [scene_example].

``` bash
sudo apt install protobuf-compiler
sudo apt install libeigen3-dev
```
For custom texture generation, we have developed a [pattern generation tool], which can randomly generate a high number of 4 different types of textures which produces materials in a format Gazebo can recognise.

### Compilation

Clone the repository to your workspace directory and build from source.

``` bash
cd ~/workspace/gap/ &&
mkdir build &&
cd build &&
cmake ../ &&
make
```

Alternatively you can build each plugin/tool individually in a similar fashion.

### Initialization

Make sure you properly initialise the required environment variables.
We provide a simple script for this:

``` bash
cd ~/workspace/gap &&
source setup.sh
```

### Reference

A scientific [paper] will soon be published and specified in this section for citation purposes.

### Disclaimer

This project is not directly affiliated with Gazebo.

[Gazebo]: http://gazebosim.org/
[Camera Utils plugin]: camera_utils
[Visual Utils plugin]: visual_utils
[World Utils plugin]: world_utils
[examples]: examples
[scene generator]: examples/scene_example
[scene_example]: examples/scene_example
[documentation]: http://web.tecnico.ulisboa.pt/joao.borrego/gap/
[pattern generation tool]: https://github.com/ruipimentelfigueiredo/pattern-generation-lib
[paper]: http://vislab.isr.ist.utl.pt/wp-content/uploads/2018/04/jborrego-icarsc2018.pdf
