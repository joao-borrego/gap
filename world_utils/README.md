### World Utils

This utility allows models to be added and removed programatically during simulation execution.

#### Message types

##### Request

Work In Progress

<table>
  <tr>
    <th>Type</th>
    <th>Parameter</th>
    <th>Description</th>
    <th>Values</th>
  </tr>
  <tr>
    <td rowspan="10">Spawn</td>
    <td>ModelType<br></td>
    <td>Type of model<br></td>
    <td>Sphere, Cylinder, Box, Custom, Custom Light, Model <br></td>
  </tr>
  <tr>
    <td>Name<br><br></td>
    <td>Name of entity to spawn<br></td>
    <td>string</td>
  </tr>
  <tr>
    <td>Pose<br></td>
    <td>Pose of entity<br></td>
    <td>gazebo.msgs.Pose</td>
  </tr>
  <tr>
    <td>Mass</td>
    <td>Entity mass<br></td>
    <td>double</td>
  </tr>
  <tr>
    <td>Texture URI<br></td>
    <td>Custom material URI<br></td>
    <td>string (No need to enclose in &lt;uri&gt; tags )<br></td>
  </tr>
  <tr>
    <td>Texture Name<br></td>
    <td>Custom material name<br></td>
    <td>string</td>
  </tr>
  <tr>
    <td>Radius</td>
    <td>Radius of cylinder or sphere<br></td>
    <td>double</td>
  </tr>
  <tr>
    <td>Length</td>
    <td>Length of cylinder<br></td>
    <td>double</td>
  </tr>
  <tr>
    <td>Box Size<br></td>
    <td>Dimensions of box object<br></td>
    <td>gazebo.msgs.Vector3d</td>
  </tr>
  <tr>
    <td>sdf</td>
    <td>SDF string for custom object<br></td>
    <td>string<br></td>
  </tr>
  <tr>
    <td>Remove</td>
    <td>Name</td>
    <td>Name of entity to remove<br></td>
    <td>string (If none is provided, the world is cleared)<br></td>
  </tr>
  <tr>
    <td rowspan="2">Move</td>
    <td>Name<br></td>
    <td>Name of entity to move<br></td>
    <td>string</td>
  </tr>
  <tr>
    <td>Pose</td>
    <td>Desired pose for entity<br></td>
    <td>gazebo.msgs.Pose</td>
  </tr>
  <tr>
    <td>Physics</td>
    <td>State</td>
    <td>Desired state for physics engine<br></td>
    <td>bool (false: off, true: on)<br></td>
  </tr>
  <tr>
    <td>Pause</td>
    <td>State</td>
    <td>Start or stop  simulation<br></td>
    <td>bool (false: unpause, true: pause)<br></td>
  </tr>
  <tr>
    <td>Status</td>
    <td>Name</td>
    <td>Entity to provide details about<br></td>
    <td>string (If none provided, world is assumed)<br></td>
  </tr>
</table>

##### Response

#### Example client

TODO

#### Custom textures

In order to use custom textures, the recommended procedure is as follows:

Create a folder media in the root with:

```
cd ~/workspace/gazebo-utils &&
mkdir media
```

The textures have to respect the following file structure:

```
media/
  └╴materials/
    ├╴scripts/
    │ └╴scipt_name.material
    └╴textures/
      └╴img.png
```

The sdf model itself is not needed, but the contents of `script_name.material` should resemble (the filtering is optional):

```
material Material/Name
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

The pattern generation tool is a good example of how to generate materials that follow these guidelines.