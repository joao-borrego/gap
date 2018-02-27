/// \file scene_example/scene_example.cc
///
/// \brief TODO
///
/// TODO
///

#include "scene_example.hh"

// Global variables

// 5 x 5 object Grid
ObjectGrid g_grid(5, 5, 5, 5);
// Object count
int g_object_count{0};
std::mutex g_object_count_mutex;
// Camera ready
bool g_camera_ready{false};
std::mutex g_camera_ready_mutex;

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
    // Command-line arguments
    unsigned int scenes {0};
    unsigned int start {0};
    std::string media_dir;
    std::string imgs_dir;
    std::string dataset_dir;
    bool debug {false};

    // Parse command-line arguments
    parseArgs(argc, argv, scenes, start, media_dir, imgs_dir, dataset_dir, debug);

    // Setup filesystem

    // Create output directories
    createDirectory(dataset_dir);
    // Open texture folder and obtain file list
    std::string textures_dir = media_dir + "/materials/scripts";
    std::vector<std::string> textures;
    getFilenamesInDir(textures_dir, textures);

    debugPrintTrace("Found " << textures.size() << " textures");

    // Setup communication

    // Setup Gazebo client
    gazebo::client::setup(argc, argv);
    // Optional verification for Google Protocol Buffers version
    GOOGLE_PROTOBUF_VERIFY_VERSION;

    // Create the communication node
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();
    // Publish to the object spawner request topic
    gazebo::transport::PublisherPtr pub_world =
        node->Advertise<world_utils::msgs::WorldUtilsRequest>(WORLD_UTILS_TOPIC);
    // Subscribe to the object spawner reply topic and link callback function
    gazebo::transport::SubscriberPtr sub_world =
        node->Subscribe(WORLD_UTILS_RESPONSE_TOPIC, onWorldUtilsResponse);
    // Publish to the camera topic
    gazebo::transport::PublisherPtr pub_camera =
        node->Advertise<camera_utils::msgs::CameraUtilsRequest>(CAMERA_UTILS_TOPIC);
    // Subscribe to the camera utils reply topic and link callback function
    gazebo::transport::SubscriberPtr sub_camera =
        node->Subscribe(CAMERA_UTILS_RESPONSE_TOPIC, onCameraUtilsResponse);

    // Wait for WorldUtils plugin to launch
    pub_world->WaitForConnection();

    debugPrintTrace("Connected to Gazebo Server");

    // Setup scene generation

    // Spawn light source and camera
    world_utils::msgs::WorldUtilsRequest msg;
    msg.set_type(SPAWN);
    addModelFromFile(msg, "models/custom_sun.sdf", false, textures);
    addModelFromFile(msg, "models/custom_camera.sdf", false, textures);
    pub_world->Publish(msg);

    debugPrintTrace("Spawning light and camera");

    // Wait for light source and camera to spawn
    while (waitForObjectCount(2)){
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        queryObjectCount(pub_world);
    }

    debugPrintTrace("Done waiting for spawn");

    // Disable physics engine
    setPhysics(pub_world, false);
    debugPrintTrace("Disable physics engine");

    // Main loop

    for (int iter; iter < scenes; iter++){

        // Populate grid with random objects
        int num_objects = (getRandomInt(5, 10));
        g_grid.populate(num_objects);

        debugPrintTrace("Scene (" << iter + 1 << "/"
        	<< scenes << "): " << num_objects << " objects");

        // Obtain 3D surface points

        // Request move camera
        // Request move light

        // Wait empty world
        // Request spawn: ground and objects

        // Wait for camera and light to move
        // Request point projection

        // Wait for objects to spawn
        // Request image
        // Wait for image

        // If debug, view image

        // Wait for projections
        // Request clear world
        // Save annotations to file
    }

    // Clean up

    sub_world.reset();
    sub_camera.reset();
    node->Fini();
    gazebo::client::shutdown();

    debugPrintTrace("All scenes generated sucessfully! Exiting...");
}

//////////////////////////////////////////////////
void addModelFromFile(
    world_utils::msgs::WorldUtilsRequest & msg,
    const std::string & file,
    const bool custom_texture,
    std::vector<std::string> & textures)
{
    // Read file to SDF string
    std::ifstream infile {file};
    std::string model_sdf {
        std::istreambuf_iterator<char>(infile), std::istreambuf_iterator<char>()
    };

    // Add object to request message
    world_utils::msgs::Object *object = msg.add_object();
    object->set_model_type(CUSTOM);
    object->set_sdf(model_sdf);

    // Apply custom texture
    if (custom_texture) {
        addCustomTexture(object, textures);
    }
}

//////////////////////////////////////////////////
void addCustomTexture(
    world_utils::msgs::Object *object,
     std::vector<std::string> & textures)
{
    int texture_idx  = getRandomInt(0, textures.size() - 1);
    std::string texture = textures.at(texture_idx);
    std::stringstream texture_uri;
    std::stringstream texture_name;

    // According to documented structure for textures
    texture_uri << "file://materials/scripts/" << texture << ".material"
        << "</uri><uri>file://materials/textures/";
    texture_name << "Plugin/" << texture;

    object->set_texture_uri(texture_uri.str());
    object->set_texture_name(texture_name.str());
}

//////////////////////////////////////////////////
bool waitForObjectCount(int num_objects){
    
    std::lock_guard<std::mutex> lock(g_object_count_mutex);
    return (num_objects != g_object_count);
}

//////////////////////////////////////////////////
void queryObjectCount(gazebo::transport::PublisherPtr pub){
    
    world_utils::msgs::WorldUtilsRequest msg;
    msg.set_type(STATUS);
    pub->Publish(msg, false);
}

//////////////////////////////////////////////////
void onWorldUtilsResponse(WorldUtilsResponsePtr &_msg)
{
    if (_msg->type() == INFO){
        if (_msg->has_object_count()){
            std::lock_guard<std::mutex> lock(g_object_count_mutex);
            g_object_count = _msg->object_count();
        }
    }
}

//////////////////////////////////////////////////
void onCameraUtilsResponse(CameraUtilsResponsePtr &_msg)
{
    // TODO
}

//////////////////////////////////////////////////
void setPhysics(gazebo::transport::PublisherPtr pub, bool enable)
{
    world_utils::msgs::WorldUtilsRequest msg;
    msg.set_type(PHYSICS);
    msg.set_state(enable);
    pub->Publish(msg);
}
