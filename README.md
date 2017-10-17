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
# Server and plugin
gazebo spawner.world
# Client
./build/spawner_client
```

#### Custom textures

In order to use custom textures, the recommended procedure is as follows:

Create a folder media in `~/workspace/gazebo-utils/`.
The textures have to respect the following file structure:

``` ├
media/
   └╴model_name/
      └╴materials/
         └╴scripts/
            └╴model_name.material
         └╴textures/
            └╴img.png
```

The sdf model itself is not needed, but the contents of `model_name.material` should resemble:

```
material Model/Texture
{
  technique
  {
    pass
    {
      texture_unit
      {
        texture img.png
        filtering anistropic
        max_anisotropy 16
      }
    }
  }
}

```

Finally, but **VERY IMPORTANTLY** export the media directory so gazebo can find it:
```
cd ~/workspace/gazebo-utils &&
version=7 &&
source /usr/share/gazebo-$version/setup.sh &&
export GAZEBO_RESOURCE_PATH=${GAZEBO_RESOURCE_PATH}:`pwd`/media
```

### Texture generator

TODO

#### Debug

Make sure a message is shown when gazebo is loaded indicating the plugin was initialized.
It might be the case that the environment variables were not correctly exported.