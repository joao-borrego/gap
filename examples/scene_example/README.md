### Random scene generation

This example intends to create a large dataset of randomized scenes of up to 10 objects, namely boxes, spheres and cylinders, collecting both fully simulated Full-HD images and each object's classification and bounding box.

### Prerequisites

#### Custom materials

Generate custom textures and have Gazebo load them at launch.
We used four different texture types, each with a given name pattern: Plugin/flat_XXX, Plugin/perlin_XXX, and so on, where the XXX represent the index of a particular texture, ranging from 1 to the number of textures of a given type.

This step requires you to:
1. Add a file similar to [plugin.material] to `gazebo-9/media/materials/scripts/`, which is located in `/usr/local/share/` for a default installation from source.
2. Copy the texture images to `gazebo-9/media/materials/scripts/textures/plugin`, according to the previously mentioned naming pattern.


### Running

Open up two terminals in the root directory of the repository.
On terminal 1 run gazebo server:
```
cd ~/workspace/gazebo-utils/ &&
source setup.sh &&
gzserver worlds/spawner.world
```

On terminal 2 run the example client:
```
cd ~/workspace/gazebo-utils/ &&
./build/examples/scene_example/scene_example -s 200 -d ./train/SHAPES2018/dataset/ -i ./train/SHAPES2018/images/ 
```

This should generate a dataset with 200 images, spread across two subdirectories 000 and 100.
This is due to performance concerns.
You can use `scene_example --help` to obtain an explanation of each command-line argument.

### Debugging dataset output

We provide a [debugging tool] written in Python 3, which relies on Tkinter to create the GUI, and shows the resulting dataset, one image at a time. 

[plugin.material]: plugin.material
[debugging tool]: https://github.com/jsbruglie/gazebo-utils/blob/dev/scripts/scene_checker.py
