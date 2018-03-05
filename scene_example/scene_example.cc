/// \file scene_example/scene_example.cc
///
/// \brief TODO
///
/// TODO
///

#include "scene_example.hh"

// Global variables

// 4 x 4 object Grid
ObjectGrid g_grid(4, 4, 4, 4, 1);

// Variables that lock progress for synchronous scene generation
bool g_moved {false};
std::mutex g_moved_mutex;
bool g_camera_ready {false};
std::mutex g_camera_ready_mutex;
bool g_points_ready {false};
std::mutex g_points_ready_mutex;

// Global camera pose
ignition::math::Pose3d g_camera_pose;

// Global current iteration
int g_iteration;

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

    // Light poses
    ignition::math::Pose3d light_pose;
    
    // Main loop
    for (g_iteration = 0; g_iteration < scenes; g_iteration++) {

        // Populate grid with random objects
        int num_objects = (getRandomInt(5, 10));
        g_grid.populate(num_objects);

        debugPrintTrace("Scene (" << g_iteration << "/"
            << scenes - 1 << "): " << num_objects << " objects");

        // Create message with desired 3D points to project in camera plane
        camera_utils::msgs::CameraUtilsRequest msg_points;
        msg_points.set_type(PROJECTION_REQUEST);
        addProjections(msg_points);

        // Calculate new camera and light poses
        g_camera_pose = getRandomCameraPose();
        light_pose = getRandomLightPose();

        // Request move camera
        world_utils::msgs::WorldUtilsRequest msg_move;
        msg_move.set_type(WORLD_MOVE);
        //addMoveObject(msg_move, "custom_camera", false, g_camera_pose);
        addMoveObject(msg_move, "custom_sun", true, light_pose);
        pub_world->Publish(msg_move);

        // Update scene
        visual_utils::msgs::VisualUtilsRequest msg_visual;
        msg_visual.set_type(UPDATE);
        updateObjects(msg_visual);
        pub_visual->Publish(msg_visual);

        moveCamera(pub_camera);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Wait for camera to move to new position
        while (waitForMove()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        //debugPrintTrace("[1/3] Done waiting for camera and light to move.");

        // Capture the scene and save it to a file
        captureScene(pub_camera, g_iteration);
        while (waitForCamera()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        // Request point projection
        pub_camera->Publish(msg_points);

        //debugPrintTrace("[2/3] Done waiting for frame to be acquired.");

        // Wait for projections
        while (waitForProjections()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10)); 
        }
        
        //debugPrintTrace("[3/3] Done waiting for annotations.");

        // Save annotations to file
        storeAnnotations(dataset_dir, g_iteration);

        // If debug, view image
        if (debug) {
            visualizeData(imgs_dir, g_iteration);
        }
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
        getRandomDouble(-M_PI / 5.0, M_PI / 5.0),
        getRandomDouble(-M_PI / 5.0, M_PI / 5.0),
        getRandomDouble(-M_PI / 5.0, M_PI / 5.0)); 
    
    ignition::math::Pose3d new_pose;
    ignition::math::Vector3d position(2, 2, 6);

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
void moveCamera(gazebo::transport::PublisherPtr pub)
{
    camera_utils::msgs::CameraUtilsRequest msg;
    msg.set_type(MOVE_REQUEST);

    gazebo::msgs::Pose *pose_msg = new gazebo::msgs::Pose();
    gazebo::msgs::Set(pose_msg, g_camera_pose);
    msg.set_allocated_pose(pose_msg);
    
    pub->Publish(msg, false);
}

//////////////////////////////////////////////////
void captureScene(gazebo::transport::PublisherPtr pub, int iteration)
{
    camera_utils::msgs::CameraUtilsRequest msg;
    msg.set_type(CAPTURE_REQUEST);
    msg.set_file_name(std::to_string(iteration / 100) + "00/" + std::to_string(iteration));
    pub->Publish(msg, false);
}

//////////////////////////////////////////////////
bool waitForMove()
{
    std::lock_guard<std::mutex> lock(g_moved_mutex);
    if (g_moved) {
        g_moved = false;
        return false;
    }
    return true;
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
bool waitForProjections()
{
    std::lock_guard<std::mutex> lock(g_points_ready_mutex);
    if (g_points_ready) {
        g_points_ready = false;
        return false;
    }
    return true;
}

//////////////////////////////////////////////////
void onWorldUtilsResponse(WorldUtilsResponsePtr &_msg)
{
    /*
    if (_msg->type() == SUCCESS) {
        std::lock_guard<std::mutex> lock(g_moved_mutex);
        g_moved = true;
    }
    */
}

//////////////////////////////////////////////////
void onCameraUtilsResponse(CameraUtilsResponsePtr &_msg)
{
    if (_msg->type() == MOVE_RESPONSE)
    {
        std::lock_guard<std::mutex> lock(g_moved_mutex);
        g_moved = true;
    }
    else if (_msg->type() == CAPTURE_RESPONSE)
    {
        if (_msg->success()) {
            std::lock_guard<std::mutex> lock(g_camera_ready_mutex);
            g_camera_ready = true;
        }
    }
    else if (_msg->type() == PROJECTION_RESPONSE)
    {
        int objects = _msg->projections_size();
        int x_min, x_max, y_min, y_max;
        int x_tmp, y_tmp;

        // Ensure projections correspond to desired camera pose
        ignition::math::Pose3d camera_pose(gazebo::msgs::ConvertIgn(_msg->pose()));
        if (camera_pose != g_camera_pose) return;

        for (int i = 0; i < objects; i++)
        {
            int points = _msg->projections(i).point2_size();
            x_min = y_min = INT_MAX;
            x_max = y_max = INT_MIN; 
            for (int j = 0; j < points; j++)
            {
                // Obtain 2D bounding box
                x_tmp = _msg->projections(i).point2(j).x();
                y_tmp = _msg->projections(i).point2(j).y();
                if (x_min > x_tmp) x_min = x_tmp;
                if (x_max < x_tmp) x_max = x_tmp;
                if (y_min > y_tmp) y_min = y_tmp;
                if (y_max < y_tmp) y_max = y_tmp; 
            }
            // Store bounding box
            g_grid.objects[i].bounding_box.push_back(x_min);
            g_grid.objects[i].bounding_box.push_back(y_min);
            g_grid.objects[i].bounding_box.push_back(x_max);
            g_grid.objects[i].bounding_box.push_back(y_max);
        }

        std::lock_guard<std::mutex> lock(g_points_ready_mutex);
        g_points_ready = true;
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

//////////////////////////////////////////////////
void visualizeData(const std::string & image_dir, int iteration)
{
    std::string image_ext = ".png";
    std::string image_name = std::to_string(iteration / 100) + "00/" + std::to_string(iteration);
    std::string filename = image_dir + image_name + image_ext;
    
    // Read image from file
    cv::Mat image = cv::imread(filename, CV_LOAD_IMAGE_COLOR);
    // Check if image is valid
    if (!image.data)
    {
       std::cerr <<  "Could not open '" << filename << "'" << std::endl;
    }
    else
    {
        cv::Scalar color;
        cv::Scalar blue(255,0,0), red(0,255,0), green(0,0,255);
        for (int i = 0; i < g_grid.objects.size(); i++)
        {
            Object obj = g_grid.objects[i];
            if      (obj.type == SPHERE)    color = blue;
            else if (obj.type == CYLINDER)  color = red;
            else                            color = green;

            cv::rectangle(image,
                cv::Point(obj.bounding_box[0], obj.bounding_box[1]),
                cv::Point(obj.bounding_box[2], obj.bounding_box[3]),
                color,2,8,0);
        }

        // Create a window for display
        cv::namedWindow("Display", cv::WINDOW_KEEPRATIO);
        // Show image and wait for keystroke
        cv::imshow("Display", image);
        cv::waitKey(0);
    }
}

//////////////////////////////////////////////////
void storeAnnotations(
    const std::string & path,
    const int iteration)
{
    std::string ext_img = ".png";
    std::string ext_data = ".xml";
    std::string image_name = std::to_string(iteration / 100) + "00/" +
        std::to_string(iteration) + ext_img;
    std::string data_name = std::to_string(iteration) + ext_data;

    std::ofstream out(path+"/"+data_name);

    // TODO
    int camera_width = 1920;
    int camera_height = 1080;
    int camera_depth = 3;

    out << "<annotation>\n"
        << "  <folder>images</folder>\n"
        << "  <filename>"+image_name+"</filename>\n"
        << "  <source>\n"
        << "    <database>The SHAPE2018 Database</database>\n"
        << "    <annotation>SHAPE SHAPE2018</annotation>\n"
        << "    <image>" << image_name <<"</image>\n"
        << "    <pose>" << g_camera_pose <<"</pose>\n"
        << "  </source>\n"
        << "  <size>\n"
        << "    <width>"  << camera_width  << "</width>\n"
        << "    <height>" << camera_height << "</height>\n"
        << "    <depth>"  << camera_depth  << "</depth>\n"
        << "  </size>\n"
        << "  <segmented>1</segmented>\n";

    for (int i = 0; i < g_grid.objects.size(); i++)
    {
        Object object = g_grid.objects[i];
        out << "  <object>\n"
            << "    <name>" << g_grid.TYPES[object.type] << "</name>\n"
            << "    <pose>" << object.pose << "</pose>\n"
            << "    <truncated>0</truncated>\n"
            << "    <difficult>1</difficult>\n"
            << "    <bndbox>\n"
            << "      <xmin>"<< object.bounding_box[0] <<"</xmin>\n"
            << "      <ymin>"<< object.bounding_box[1] <<"</ymin>\n"
            << "      <xmax>"<< object.bounding_box[2] <<"</xmax>\n"
            << "      <ymax>"<< object.bounding_box[3] <<"</ymax>\n"
            << "    </bndbox>\n"
            << "  </object>\n";
    }

    out << "</annotation>";
    // TODO - Keep file open
    out.close();
}
