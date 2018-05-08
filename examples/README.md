## Examples

This set of plugins was originally conceived so we could ultimately build the [scene generation] example.
It uses all three CameraUtils, VisualUtils and WorldUtils plugins.

In order to interact with our tools you need only write your own client applications that use our custom message definitions.
These are compiled into a shared library for your convenience, and can be easily linked to by installing our package.
See [external_example] for more details.

Furthermore, we provide simples example client applications to interact with each plugin.
- [camera_example], for acquiring frames using CameraUtils
- [visual_example], for changing object visuals using VisualUtils
- [world_example], for interacting with the world using WorldUtils 

[external_example]: external_example
[scene generation]: scene_example
[camera_example]: camera_example 
[visual_example]: visual_example
[world_example]: world_example
