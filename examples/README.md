## Examples

This set of plugins was originally conceived so we could ultimately build the [scene generation] example.
It uses all three CameraUtils, VisualUtils and WorldUtils plugins.

If you want to use our tools as a library you can do so with a simple hack until the project is organised as a CMake package.
This is planned for a future release.
For now, check [external_example] and its [CMakeLists.txt]

Furthermore, we provide simples example client applications to interact with each plugin.
- [camera_example], for acquiring frames using CameraUtils
- [visual_example], for changing object visuals using VisualUtils
- [world_example], for interacting with the world using WorldUtils 

[external_example]: external_example
[CMakeLists.txt]: external_example/CMakeLists.txt

[scene generation]: scene_example
[camera_example]: camera_example 
[visual_example]: visual_example
[world_example]: world_example
