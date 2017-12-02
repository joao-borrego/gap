/**
 * @file capture_example.hh
 *
 * @brief Example application using object spawner and camera plugins
 *
 * This example spawns randomly generated objects and captures images from
 * a camera in a given position.
 *
 * It requires that
 * - gazebo-utils/media folder is adequatly populated with different materials
 * - gazebo-utils/model folder has the custom_camera.sdf, custom_sun.sdf and custom_ground.sdf models
 * - output_dir exists
 *
 * @param[in]  _argc  The number of command-line arguments
 * @param      _argv  The argv The value of the command-line arguments
 *
 * @return     0
 */

#include "capture_example.hh"

// Global variables

// Objects
std::vector<Object> objects;
// Object count on server
int object_count{0};
std::mutex object_count_mutex;
// Camera ready
bool camera_ready{false};
std::mutex camera_ready_mutex;
// 3D bounding boxes
BoundingBox3d bbs_3d;
std::mutex bounding_box_3d_mutex;
// 2D projections of 3D points
BoundingBox2d points_2d;
std::mutex points_2d_mutex;

// Grid
std::vector<int> cells_array;
const double x_size = 5.0;
const double y_size = 5.0;
const unsigned int x_cells = 5;
const unsigned int y_cells = 5;
double cell_size_x = x_size / x_cells;
double cell_size_y = y_size / y_cells;

// Camera properties
std::shared_ptr<CameraInfo> camera_info;

int main(int argc, char **argv)
{
    // Command-line arguments
    unsigned int scenes{0};
    unsigned int start{0};
    std::string media_dir;
    std::string dataset_dir;

    // Aux variables

    // Vector of texture filenames
    std::string materials_dir;
    std::string scripts_dir;
    std::vector<std::string> textures;
    // Camera pose
    ignition::math::Pose3d camera_pose;
    ignition::math::Vector3d camera_position(0, 0, 5.0);
    // Msg for initial objects
    world_utils::msgs::WorldUtilsRequest msg_basic_objects;
    // Random objects
    int num_objects{0};
    int min_objects{5}, max_objects{10};

    // Parse command-line args
    parseArgs(argc, argv, scenes, start, media_dir, dataset_dir);

    // Create folder for storing output folder
    createDirectory(dataset_dir);

    // Create a vector with the name of every texture in the textures dir
    materials_dir = media_dir + "/materials";
    scripts_dir = materials_dir + "/scripts";
    getFilenamesInDir(scripts_dir, textures);

    // Load gazebo as a client
    #if GAZEBO_MAJOR_VERSION < 6
    gazebo::setupClient(argc, argv);
    #else
    gazebo::client::setup(argc, argv);
    #endif

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

    // Wait for the WorldUtils plugin to launch */
    pub_world->WaitForConnection();

    // Generate a random camera pose
    camera_pose = getRandomCameraPose(camera_position);

    // Spawn light source and camera
    msg_basic_objects.set_type(SPAWN);
    addModelToMsg(msg_basic_objects, objects, "models/custom_sun.sdf",
        true, false, false, 0, 0, textures);
    addModelToMsg(msg_basic_objects, objects, "models/custom_camera.sdf",
        false, false, false, 0, 0, textures);
    pub_world->Publish(msg_basic_objects);

    // Wait for CameraUtils plugin to launch and models to spawn
    pub_camera->WaitForConnection();
    while (waitForSpawner(1)){
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        queryModelCount(pub_world);
    }

    // Create cell grid
    for (int i = 0;  i < x_cells * y_cells; i++){
        cells_array.push_back(i);
    }

    // Disable world physics
    changePhysics(pub_world, false);

    // Query camera parameters
    debugPrintTrace("Query camera parameters");
    queryCameraParameters(pub_camera);
    while (waitForCamera()){
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    debugPrintTrace("Initialisation complete");

    // Main program loop
    for (int i = start; i < scenes; i++){


        // Move camera
        camera_pose = getRandomCameraPose(camera_position);
        moveObject(pub_world, "custom_camera", camera_pose);

        // TODO - Bug first iteration gets garbage. Rethink operation order
        if (i == start) sleep(1);

        debugPrintTrace("Done moving camera to random position");


        // Number of objects to spawn
        num_objects = (getRandomInt(min_objects, max_objects));

        debugPrintTrace("Scene (" << i + 1 << "/" << scenes << "): " << num_objects << " objects");

        // Spawn ground and random objects

        world_utils::msgs::WorldUtilsRequest msg_random_objects;
        msg_random_objects.set_type(SPAWN);
        addModelToMsg(msg_random_objects, objects, "models/custom_ground.sdf",
            false, false, true, 0, 0, textures);
        shuffleIntVector(cells_array);
        for (int j = 0; j < num_objects; j++){
            int cell_num = cells_array[j];
            int cell_x = floor(cell_num / x_cells);
            int cell_y = floor(cell_num - cell_x * x_cells);
            addModelToMsg(msg_random_objects, objects, "",
                false, true, true, cell_x, cell_y, textures);
        }
        pub_world->Publish(msg_random_objects);

        // Wait for object count to match object number + camera + ground
        while (waitForSpawner(num_objects + 2)){
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            queryModelCount(pub_world);
        }

        /*
        // Wait for all messages to be sent
        while (pub_world->GetOutgoingCount() > 0){
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        */

        // Get 3D bounding boxes from WorldUtils 
        bbs_3d.clear();
        queryModelBoundingBox(pub_world, objects);
        while (waitForBoundingBox(num_objects)){
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        debugPrintTrace("Done waiting for 3D bounding boxes");


        // Get 2D projection of the defining 8 points of the 3D bounding box
        points_2d.clear();
        query2DcameraPoint(pub_camera, objects);
        
        while(waitFor2DPoints(8 * num_objects)){
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        debugPrintTrace("Done waiting for 2D point projections");

        // Obtain 2D bounding boxes
        std::vector<cv::Rect> bound_rect(num_objects);
        obtain2DBoundingBoxes(objects, bound_rect);

        debugPrintTrace("Done calculating 2D bounding boxes");
        
        // Save annotations to a file
        storeAnnotations(objects, camera_pose, dataset_dir,
            std::to_string(i)+".xml", std::to_string(i) + ".jpg");

        debugPrintTrace("Done saving annotations");

        // Capture the scene and save it to a file
        captureScene(pub_camera, i);
        while (waitForCamera()){
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        
        debugPrintTrace("Done waiting for camera to capture scene");

        /*
        // Visualize data
        visualizeData("/tmp/camera_utils_output/", std::to_string(i),
            num_objects, points_2d, bound_rect);
        */

        // Clear world - removes objects matching "plugin" prefix
        std::vector<std::string> object_names;
        object_names.push_back("plugin");
        clearWorld(pub_world, object_names);

        while (waitForSpawner(1)){
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            queryModelCount(pub_world);
        }
        objects.clear();

        debugPrintTrace("Done clearing random objects");
    }

    // Clean up
    std::vector<std::string> object_names;
    // Every spawned object will match "_"
    object_names.push_back("_");
    clearWorld(pub_world, object_names);

    sub_world.reset();
    sub_camera.reset();
    node->Fini();

    /* Shut down */
    #if GAZEBO_MAJOR_VERSION < 6
    gazebo::shutdown();
    #else
    gazebo::client::shutdown();
    #endif

    debugPrintTrace("All scenes generated sucessfully! Exiting...");

    return 0;
}


ignition::math::Pose3d getRandomCameraPose(const ignition::math::Vector3d & camera_position) {

    static const ignition::math::Quaternion<double> correct_orientation(
        ignition::math::Vector3d(0,1,0), - M_PI / 2.0);
    /*
    ignition::math::Quaternion<double> camera_orientation(
        getRandomDouble(0, M_PI / 2.0),
        getRandomDouble(0, M_PI / 2.0),
        getRandomDouble(0, M_PI / 2.0));
    */
    ignition::math::Quaternion<double> camera_orientation(0,0,0);

    ignition::math::Pose3d camera_pose;
    camera_pose.Set(
        camera_position,
        (correct_orientation*camera_orientation).Inverse());
    camera_pose=camera_pose.RotatePositionAboutOrigin(camera_orientation);

    return camera_pose;
}

void addModelToMsg(
    world_utils::msgs::WorldUtilsRequest & msg,
    std::vector<Object> & objects,
    const std::string & model_path,
    const bool is_light,
    const bool is_random,
    const bool use_custom_textures,
    const int cell_x,
    const int cell_y,
    std::vector<std::string> & textures){
    
    world_utils::msgs::Object *object = msg.add_object();

    // Handle model type
    if (! is_random){
        
        // Load sdf model from file
        std::ifstream infile {model_path};
        std::string model_sdf { 
            std::istreambuf_iterator<char>(infile), std::istreambuf_iterator<char>() };
        
        if (is_light){
            object->set_model_type(CUSTOM_LIGHT);
        } else {
            object->set_model_type(CUSTOM);
        }
        object->set_sdf(model_sdf);
    
    } else {

        // Generate random object from possible types
        genRandomObjectInGrid(objects, object, cell_x, cell_y);
    }

    // Apply custom texture pattern
    if (use_custom_textures){

        int texture_idx  = getRandomInt(0, textures.size() - 1);
        std::string texture = textures.at(texture_idx);
        std::stringstream texture_uri;
        std::stringstream texture_name;

        texture_uri << "file://materials/scripts/" << texture << ".material"
            << "</uri><uri>file://materials/textures/";
        texture_name << "Plugin/" << texture;

        object->set_texture_uri(texture_uri.str());
        object->set_texture_name(texture_name.str());
    }
}

void genRandomObjectInGrid(
    std::vector<Object> & objects,
    world_utils::msgs::Object *object,
    const int cell_x,
    const int cell_y){

    // Variables
    gazebo::msgs::Vector3d *pos = new gazebo::msgs::Vector3d();
    gazebo::msgs::Quaternion *ori = new gazebo::msgs::Quaternion();
    gazebo::msgs::Pose *pose = new gazebo::msgs::Pose();
    ignition::math::Quaternion<double> object_orientation;
    double radius{0.0}, length{0.0};
    double size_x{0.0}, size_y{0.0}, size_z{0.0};

    int object_type = getRandomInt(0, 2);
    object->set_model_type(classes_id[object_type]);
    std::string object_name = class_instance_names[object_type] + 
        std::to_string((class_instance_counters[object_type])++);
    object->set_name(object_name);

    // Mass
    object->set_mass(1);

    // Orientation
    bool horizontal = (getRandomDouble(0.0, 1.0) < 0.5);
    if (horizontal && object_type == cylinder){
        object_orientation = ignition::math::Quaternion<double> (
            0.0, M_PI * 0.5, getRandomDouble(0.0, M_PI));
    } else {
        object_orientation = ignition::math::Quaternion<double> (
            0.0, 0.0, getRandomDouble(0.0, M_PI));
    }
    gazebo::msgs::Set(ori, object_orientation);
    pose->set_allocated_orientation(ori);

    // Dimensions
    if (object_type == sphere || object_type == cylinder){
    
        // Radius
        radius = getRandomDouble(0.1,
            std::min(cell_size_x, cell_size_y) * 0.5);
        object->set_radius(radius);

        // Cylinder length
        if (object_type == cylinder){
            if (horizontal){
                length = getRandomDouble(0.1, std::min(cell_size_x, cell_size_y) * 0.5);
            } else {
                length = getRandomDouble(0.5, 1.0);    
            }
            object->set_length(length);
        }
    
    } else if (object_type == box){

        gazebo::msgs::Vector3d *size = new gazebo::msgs::Vector3d();

        // TODO fix collisions - Box size
        size_x = getRandomDouble(cell_size_x * 0.1, cell_size_x);
        size_y = getRandomDouble(cell_size_y * 0.1, cell_size_y);
        size_z = getRandomDouble(0.5, 1.0);

        size->set_x(size_x);
        size->set_y(size_y);
        size->set_z(size_z);
        object->set_allocated_box_size(size);
    }

    // Position
    pos->set_x(cell_x * cell_size_x + 0.5 * cell_size_x - x_cells * 0.5 * cell_size_x);
    pos->set_y(cell_y * cell_size_y + 0.5 * cell_size_y - y_cells * 0.5 * cell_size_y);
    
    if (object_type == sphere || object_type == cylinder){

        pos->set_z(radius);

    } else if (object_type == box){
        
        pos->set_z(size_z * 0.5);
    }
    pose->set_allocated_position(pos);

    // Pose
    object->set_allocated_pose(pose);

    objects.push_back( Object(object_name, object_type, gazebo::msgs::ConvertIgn(*pose)));
}

void moveObject(
    gazebo::transport::PublisherPtr pub,
    const std::string &name,
    const ignition::math::Pose3d &pose){

    world_utils::msgs::WorldUtilsRequest msg;
    msg.set_type(MOVE);
    world_utils::msgs::Object *object = msg.add_object();
    object->set_name(name);

    gazebo::msgs::Pose *pose_msg = new gazebo::msgs::Pose();
    gazebo::msgs::Set(pose_msg, pose);
    object->set_allocated_pose(pose_msg);
    pub->Publish(msg);
}

void clearWorld(
    gazebo::transport::PublisherPtr pub,
    std::vector<std::string> &object_names){

    world_utils::msgs::WorldUtilsRequest msg;
    msg.set_type(REMOVE);
    for (int i = 0; i < object_names.size();++i){
        world_utils::msgs::Object* object = msg.add_object();
        object->set_name(object_names[i]);
    }
    pub->Publish(msg);
}

void changePhysics(gazebo::transport::PublisherPtr pub, bool enable){
    world_utils::msgs::WorldUtilsRequest msg;
    msg.set_type(PHYSICS);
    msg.set_state(enable);
    pub->Publish(msg);
}

void pauseWorld(gazebo::transport::PublisherPtr pub, bool enable){
    world_utils::msgs::WorldUtilsRequest msg;
    msg.set_type(PAUSE);
    msg.set_state(enable);
    pub->Publish(msg);
}

void captureScene(gazebo::transport::PublisherPtr pub, int j){
    camera_utils::msgs::CameraUtilsRequest msg;
    msg.set_type(CAPTURE_REQUEST);
    msg.set_file_name(std::to_string(j));
    pub->Publish(msg,false);
}

void queryCameraParameters(gazebo::transport::PublisherPtr pub){
    camera_utils::msgs::CameraUtilsRequest msg;
    msg.set_type(CAMERA_INFO_REQUEST);
    pub->Publish(msg,false);
}

// Handle model count

bool waitForSpawner(int desired_objects){
    
    std::lock_guard<std::mutex> lock(object_count_mutex);
    return (desired_objects != object_count);
}

void queryModelCount(gazebo::transport::PublisherPtr pub){
    
    world_utils::msgs::WorldUtilsRequest msg;
    msg.set_type(STATUS);
    pub->Publish(msg,false);
}

// Handle 3d bounding boxes

void queryModelBoundingBox(
    gazebo::transport::PublisherPtr pub,
    const std::vector<Object> & objects){
    
    world_utils::msgs::WorldUtilsRequest msg;
    msg.set_type(STATUS);
    for (int i = 0; i < objects.size(); i++){
        world_utils::msgs::BoundingBox* bounding_box = msg.add_bounding_box();
        bounding_box->set_name(objects[i].name);
    }

    pub->Publish(msg,false);
}

bool waitForBoundingBox(int desired_objects){

    std::lock_guard<std::mutex> lock(bounding_box_3d_mutex);
    return (bbs_3d.size() != desired_objects);
}

// Handle 2d point projections

void query2DcameraPoint(
    gazebo::transport::PublisherPtr pub,
    const std::vector<Object> &objects){

    camera_utils::msgs::CameraUtilsRequest msg;
    msg.set_type(CAMERA_POINT_REQUEST);

    for (int i = 0; i < objects.size(); i++){

        std::pair <BoundingBox3d::iterator, BoundingBox3d::iterator> ret;
        ret = bbs_3d.equal_range(objects[i].name);
    
        for (BoundingBox3d::iterator it=ret.first; it!=ret.second; ++it){

            // Generate all possible combinations for center +- coordinate
            for (int x = 1; x >= -1; x -= 2){
                for (int y = 1; y >= -1; y -= 2){
                    for (int z = 1; z >= -1; z -= 2){

                        ignition::math::Vector3d point = it->second.center;
                        point.X() += x * it->second.size.X() / 2.0;
                        point.Y() += y * it->second.size.Y() / 2.0;
                        point.Z() += z * it->second.size.Z() / 2.0;

                        gazebo::msgs::Vector3d *point_msg = new gazebo::msgs::Vector3d();
                        point_msg->set_x(point.X());
                        point_msg->set_y(point.Y());
                        point_msg->set_z(point.Z());

                        camera_utils::msgs::BoundingBoxCamera *bounding_box = 
                            msg.add_bounding_box();
                        bounding_box->set_name(objects[i].name);
                        bounding_box->set_allocated_point3d(point_msg);
                    }
                }
            }
        }
    }
    pub->Publish(msg,false);
}

bool waitFor2DPoints(int desired_points){

    std::lock_guard<std::mutex> lock(points_2d_mutex);
    return (points_2d.size() != desired_points);
}

void obtain2DBoundingBoxes(
    std::vector<Object> &objects,
    std::vector<cv::Rect> &bounding_rectangles){

    for (int i = 0; i < objects.size(); i++){

        std::pair <BoundingBox2d::iterator, BoundingBox2d::iterator> ret;
        ret = points_2d.equal_range(objects[i].name);
        std::vector<cv::Point> contours_poly(8);
        int p = 0;

        for (std::multimap<std::string,ignition::math::Vector2d>::iterator
            it = ret.first; it != ret.second; it++){
                contours_poly[p++]=cv::Point(it->second.X(),it->second.Y());
        }
        bounding_rectangles[i] = cv::boundingRect(cv::Mat(contours_poly));
        objects[i].bounding_box = bounding_rectangles[i];
    }
}

void onWorldUtilsResponse(WorldUtilsResponsePtr &_msg){
    if (_msg->type() == INFO){
        if (_msg->has_object_count()){
            std::lock_guard<std::mutex> lock(object_count_mutex);
            object_count = _msg->object_count();
        }
    } else if (_msg->type() == PROPERTIES){

        for (int i = 0; i < _msg->bounding_box_size(); i++){
            ignition::math::Vector3d bb_center =
                gazebo::msgs::ConvertIgn(_msg->bounding_box(i).bb_center());
            ignition::math::Vector3d bb_size =
                gazebo::msgs::ConvertIgn(_msg->bounding_box(i).bb_size());

            BoundingBox3dClass bb(bb_center, bb_size);
            std::lock_guard<std::mutex> lock(bounding_box_3d_mutex);
            bbs_3d.insert( std::pair<std::string,BoundingBox3dClass>(
                _msg->bounding_box(i).name(), bb) );
        }
    }
}

bool waitForCamera(){

    std::lock_guard<std::mutex> lock(camera_ready_mutex);

    if (camera_ready){
        camera_ready = false;
        return false;
    }
    return true;
}

void onCameraUtilsResponse(CameraUtilsResponsePtr &_msg){

    if (_msg->type() == CAMERA_POINT_RESPONSE){

        for (int i = 0; i <_msg->bounding_box_size(); ++i){
            ignition::math::Vector2d point_2d =
                gazebo::msgs::ConvertIgn(_msg->bounding_box(i).point());
            points_2d.insert( std::pair<std::string,ignition::math::Vector2d>
                (_msg->bounding_box(i).name(), point_2d) );
        }

    } else if (_msg->type() == CAPTURE_RESPONSE){

        if (_msg->success()){
            std::lock_guard<std::mutex> lock(camera_ready_mutex);
            camera_ready = true;
        }

    } else if (_msg->type() == CAMERA_INFO_RESPONSE){

        if (_msg->success()){
            std::lock_guard<std::mutex> lock(camera_ready_mutex);
            camera_ready = true;
            camera_info = std::shared_ptr<CameraInfo>(new CameraInfo(
                _msg->camera_info().width(),
                _msg->camera_info().height(),
                _msg->camera_info().depth()) );
        }

    } else {

        std::cerr << "CameraUtils plugin error! Exiting..." << std::endl;
        exit(EXIT_FAILURE);
    }
}

void storeAnnotations(
    const std::vector<Object> &objects,
    const ignition::math::Pose3d & camera_pose,
    const std::string &path,
    const std::string &file_name,
    const std::string &image_name){

    std::ofstream out(path+"/"+file_name);

    out << "<annotation>\n"
        << "  <folder>images</folder>\n"
        << "  <filename>"+image_name+"</filename>\n"
        << "  <source>\n"
        << "    <database>The SHAPE2017 Database</database>\n"
        << "    <annotation>SHAPE SHAPE2017</annotation>\n"
        << "    <image>" << image_name <<"</image>\n"
        << "    <pose>" << camera_pose <<"</pose>\n"
        << "  </source>\n"
        << "  <size>\n"
        << "    <width>"  << camera_info->width  << "</width>\n"
        << "    <height>" << camera_info->height << "</height>\n"
        << "    <depth>"  << camera_info->depth  << "</depth>\n"
        << "  </size>\n"
        << "  <segmented>1</segmented>\n";

    for(unsigned int i=0; i<objects.size(); ++i){
        out << "  <object>\n"
            << "    <name>" << classes_name[objects[i].type] << "</name>\n"
            << "    <pose>" << objects[i].pose << "</pose>\n"
            << "    <truncated>0</truncated>\n"
            << "    <difficult>1</difficult>\n"
            << "    <bndbox>\n"
            << "      <xmin>"<< objects[i].bounding_box.x <<"</xmin>\n"
            << "      <ymin>"<< objects[i].bounding_box.y <<"</ymin>\n"
            << "      <xmax>"<< objects[i].bounding_box.x + objects[i].bounding_box.width <<"</xmax>\n"
            << "      <ymax>"<< objects[i].bounding_box.y + objects[i].bounding_box.height <<"</ymax>\n"
            << "    </bndbox>\n"
            << "  </object>\n";
    }

    out << "</annotation>";
    out.close();
}

void visualizeData(
    const std::string &image_dir,
    const std::string &image_name,
    int num_objects,
    BoundingBox2d &points_2d,
    std::vector<cv::Rect> &bound_rect){
    
    std::string filename = image_dir + image_name + ".png";
    // Read image from file
    cv::Mat image = cv::imread(filename, CV_LOAD_IMAGE_COLOR);
    // Check if image is valid
    if (!image.data){
       std::cerr <<  "Could not open '" << filename << "'" << std::endl;
    } else {

        for (int i = 0; i < num_objects; i++){

            std::pair <BoundingBox2d::iterator, BoundingBox2d::iterator> ret;
            ret = points_2d.equal_range(objects[i].name);

            for (std::multimap<std::string,ignition::math::Vector2d>::iterator
                it = ret.first; it!=ret.second; it++){

                cv::circle(image, cv::Point(it->second.X(),it->second.Y()),
                    5, cv::Scalar(255, 0, 1));
            }
            cv::rectangle(image, bound_rect[i].tl(), bound_rect[i].br(),
                cv::Scalar(255,0,1), 2, 8, 0 );
        }
        
        // Create a window for display
        cv::namedWindow("Display window", cv::WINDOW_KEEPRATIO);
        // Show image and wait for keystroke
        cv::imshow("Display window", image );
        cv::waitKey(0);
    }
}
