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
    world_utils::msgs::WorldUtilsRequest msg_spawn;
    msg_spawn.set_type(SPAWN);
    addModelFromFile(msg_spawn, "models/custom_sun.sdf");
    addModelFromFile(msg_spawn, "models/custom_ground.sdf");
    addModelFromFile(msg_spawn, "models/custom_camera.sdf");
    addDynamicModels(msg_spawn);
    pub_world->Publish(msg_spawn);
    debugPrintTrace("Spawning objects");

    // Wait for a subscriber to connect to this publisher
    pub_visual->WaitForConnection();
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    debugPrintTrace("Done waiting for spawn");

    // Camera and light poses
    ignition::math::Pose3d camera_pose, light_pose;
    
    // Main loop
    for (int iter = 0; iter < scenes; iter++) {

        // Populate grid with random objects
        int num_objects = (getRandomInt(5, 10));
        g_grid.populate(num_objects);

        debugPrintTrace("Scene (" << iter + 1 << "/"
            << scenes << "): " << num_objects << " objects");

        // Create message with desired 3D points to project in camera plane
        camera_utils::msgs::CameraUtilsRequest msg_points;
        msg_points.set_type(PROJECTION_REQUEST);
        addProjections(msg_points);

        // Calculate new camera and light poses
        camera_pose = getRandomCameraPose();
        light_pose = getRandomLightPose();

        // Request move camera
        world_utils::msgs::WorldUtilsRequest msg_move;
        msg_move.set_type(MOVE);
        addMoveObject(msg_move, "custom_camera", false, camera_pose);
        addMoveObject(msg_move, "custom_sun", true, light_pose);
        pub_world->Publish(msg_move);

        // Update scene
        visual_utils::msgs::VisualUtilsRequest msg_visual;
        msg_visual.set_type(UPDATE);
        updateObjects(msg_visual);
        pub_visual->Publish(msg_visual);

        // TODO
        // Wait for camera to move to new position
        // Request point projection
        pub_camera->Publish(msg_points);

        // Wait for objects to spawn
        // Capture the scene and save it to a file
        /*
        captureScene(pub_camera, iter);
        while (waitForCamera()){
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        */
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // TODO
        // If debug, view image
        // Wait for projections
        // Save annotations to file
    }

    // Force save camera raw data buffer

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
    msg.add_targets("ground");
}

//////////////////////////////////////////////////
void addMoveObject(
    world_utils::msgs::WorldUtilsRequest & msg,
    const std::string & name,
    const bool is_light,
    const ignition::math::Pose3d & pose){

    world_utils::msgs::Object *object = msg.add_object();
    object->set_name(name);
    if (is_light) {
        object->set_model_type(CUSTOM_LIGHT);
    }

    gazebo::msgs::Pose *pose_msg = new gazebo::msgs::Pose();
    gazebo::msgs::Set(pose_msg, pose);
    object->set_allocated_pose(pose_msg);
}

//////////////////////////////////////////////////
ignition::math::Pose3d getRandomCameraPose()
{
    static const ignition::math::Quaternion<double> correct_orientation(
        ignition::math::Vector3d(0,1,0), - M_PI / 2.0);
 
    ignition::math::Quaternion<double> original_orientation(
        getRandomDouble(0, M_PI / 5.0),
        getRandomDouble(0, M_PI / 5.0),
        getRandomDouble(0, M_PI / 5.0));

    ignition::math::Pose3d new_pose;
    ignition::math::Vector3d position(2, 2, 5);

    new_pose.Set(position,
        (correct_orientation * original_orientation).Inverse());
    new_pose = new_pose.RotatePositionAboutOrigin(original_orientation);

    return new_pose;
}

//////////////////////////////////////////////////
ignition::math::Pose3d getRandomLightPose()
{
    ignition::math::Quaternion<double> light_orientation(
        getRandomDouble(-M_PI / 3.0,M_PI / 3.0),
        getRandomDouble(-M_PI / 3.0,M_PI / 3.0),
        getRandomDouble(-M_PI / 3.0,M_PI / 3.0)); 
    
    ignition::math::Pose3d new_pose;
    ignition::math::Vector3d position(2, 2, 5);

    new_pose.Set(position, (light_orientation).Inverse());
    new_pose = new_pose.RotatePositionAboutOrigin(light_orientation);

    return new_pose;
}

//////////////////////////////////////////////////
void addProjections(camera_utils::msgs::CameraUtilsRequest & msg)
{
    int num_obj = g_grid.objects.size();
    for (int i = 0; i < num_obj; i++)
    {
        camera_utils::msgs::PointProjection *proj = msg.add_projections();
        
        int num_points = g_grid.objects[i].points.size();
        for (int j = 0; j < num_points; j++)
        {
            gazebo::msgs::Vector3d *points_msg = proj->add_point3();
            points_msg->set_x(g_grid.objects[i].points[j](0));
            points_msg->set_y(g_grid.objects[i].points[j](1));
            points_msg->set_z(g_grid.objects[i].points[j](2));
        }
        proj->set_name(g_grid.objects[i].name);
    }
}

//////////////////////////////////////////////////
void captureScene(gazebo::transport::PublisherPtr pub, int iteration)
{
    camera_utils::msgs::CameraUtilsRequest msg;
    msg.set_type(CAPTURE_REQUEST);
    msg.set_file_name(std::to_string(iteration));
    pub->Publish(msg, false);
}

//////////////////////////////////////////////////
bool waitForCamera()
{
    std::lock_guard<std::mutex> lock(g_camera_ready_mutex);
    if (g_camera_ready) {
        g_camera_ready = false;
        return false;
    }
    return true;
}

//////////////////////////////////////////////////
void onWorldUtilsResponse(WorldUtilsResponsePtr &_msg)
{
    // TODO
}

//////////////////////////////////////////////////
void onCameraUtilsResponse(CameraUtilsResponsePtr &_msg)
{
    if (_msg->type() == CAPTURE_RESPONSE) {

        if (_msg->success()){
            std::lock_guard<std::mutex> lock(g_camera_ready_mutex);
            g_camera_ready = true;
        }
    }
}

//////////////////////////////////////////////////
void setPhysics(gazebo::transport::PublisherPtr pub, bool enable)
{
    world_utils::msgs::WorldUtilsRequest msg;
    msg.set_type(PHYSICS);
    msg.set_state(enable);
    pub->Publish(msg);
}
