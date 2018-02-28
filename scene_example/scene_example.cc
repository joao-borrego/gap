/// \file scene_example/scene_example.cc
///
/// \brief TODO
///
/// TODO
///

#include "scene_example.hh"

// Global variables

// 5 x 5 object Grid
ObjectGrid g_grid(4, 4, 4, 4, 1);
// Camera ready
bool g_camera_ready{false};
std::mutex g_camera_ready_mutex;
// Regex objects
std::regex g_regex_uid(REGEX_XML_UID);
std::regex g_regex_model(REGEX_XML_MODEL);

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
    // Publish to the visual plugin topic
    gazebo::transport::PublisherPtr pub_visual =
        node->Advertise<visual_utils::msgs::VisualUtilsRequest>(VISUAL_UTILS_TOPIC);

    // Wait for WorldUtils plugin to launch
    pub_world->WaitForConnection();

    debugPrintTrace("Connected to World plugin");

    // Setup scene generation

    // Disable physics engine
    setPhysics(pub_world, false);
    debugPrintTrace("Disable physics engine");

    // Spawn required objects
    world_utils::msgs::WorldUtilsRequest msg_w;
    msg_w.set_type(SPAWN);
    addModelFromFile(msg_w, "models/custom_sun.sdf");
    addModelFromFile(msg_w, "models/custom_ground.sdf");
    addModelFromFile(msg_w, "models/custom_camera.sdf");
    addDynamicModels(msg_w);
    pub_world->Publish(msg_w);
    debugPrintTrace("Spawning objects");

    // Wait for a subscriber to connect to this publisher
    pub_visual->WaitForConnection();
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    debugPrintTrace("Done waiting for spawn");

    // Main loop
    for (int iter = 0; iter < scenes; iter++) {

        // Populate grid with random objects
        int num_objects = (getRandomInt(5, 10));
        g_grid.populate(num_objects);

        debugPrintTrace("Scene (" << iter + 1 << "/"
        	<< scenes << "): " << num_objects << " objects");

        // Request move camera
        // Request move light

        // Update scene
        visual_utils::msgs::VisualUtilsRequest msg_v;
        msg_v.set_type(UPDATE);
        updateObjects(msg_v);
        pub_visual->Publish(msg_v);

        // TEST
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

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
    const std::string & file)
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
}

//////////////////////////////////////////////////
void addDynamicModels(world_utils::msgs::WorldUtilsRequest & msg)
{
    const std::vector<std::string> types = {"sphere", "cylinder","box"};

    for (int i = 0; i < types.size(); i++)
    {
        for (int j = 1; j <= 10; j++)
        {
            // Read file to SDF string
            std::string file_name = "models/custom_" + types[i] + ".sdf";
            std::ifstream infile {file_name};
            std::string sdf {
                std::istreambuf_iterator<char>(infile), std::istreambuf_iterator<char>()
            };

            std::string name = types[i] + "_" + std::to_string(j);
            // Replace content inside <uid> tags in model SDF (VisualPlugin parameter)
            std::string uid = "<uid>" + name + "</uid>";
            sdf = std::regex_replace(sdf, g_regex_uid, uid);

            // Replace model name inside <model name=""> tag
            std::string model = "<model name=\"" + name + "\">";
            sdf = std::regex_replace(sdf, g_regex_model, model);

            // Add object to request message
            world_utils::msgs::Object *object = msg.add_object();
            object->set_model_type(CUSTOM);
            object->set_sdf(sdf);
        }
    }
}

//////////////////////////////////////////////////
void updateObjects(visual_utils::msgs::VisualUtilsRequest & msg)
{
    int total = g_grid.objects.size();
    const std::vector<std::string> types = {"sphere", "cylinder","box"};

    // Object parameters
    std::string name;
    ignition::math::Pose3d pose;
    ignition::math::Vector3d scale;

    for  (int i = 0; i < total; i++) {
        name = g_grid.objects.at(i).name;
        pose = g_grid.objects.at(i).pose;
        scale = g_grid.objects.at(i).scale;

        gazebo::msgs::Pose *msg_pose = msg.add_poses();
        gazebo::msgs::Vector3d *msg_scale = msg.add_scale();

        msg.add_targets(name);
        gazebo::msgs::Set(msg_pose, pose);
        gazebo::msgs::Set(msg_scale, scale);
    }
}

//////////////////////////////////////////////////
void onWorldUtilsResponse(WorldUtilsResponsePtr &_msg)
{
    // TODO
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
