#include "capture_example.hh"

namespace fs = boost::filesystem;

/**
 * @brief      Example application using object spawner and camera plugins
 * 
 * This example spawns randomly generated objects and captures images from 
 * a camera in a given position.
 * 
 * It requires that
 * - gazebo-utils/media folder is adequatly populated with different materials
 * - gazebo-utils/model folder has the custom_camera.sdf and custom_ground.sdf models
 * - output_dir exists 
 *
 * @param[in]  _argc  The number of command-line arguments
 * @param      _argv  The argv The value of the command-line arguments
 *
 * @return     0
 */

double dRand(double fMin, double fMax)
{
    /* Initialize random device */
    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_int_distribution<int> dist;

    double f = (double)dist(mt) / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

/** Protect access to object_count */
std::mutex object_count_mutex;
int object_count{0};
/** Protect access to camera success */
std::mutex camera_success_mutex;
int camera_success{0};

int main(int argc, char **argv)
{

    /* TODO - Process options */
    if(argc<3)
    {
        std::cout << "invalid number of arguments"<< std::endl;
        exit(-1);
    }

    std::string media_dir = std::string(argv[1]);
    unsigned int scenes = atoi(argv[2]);

    std::string materials_dir = media_dir+"/materials";
    std::string scripts_dir = media_dir+"/materials/scripts";

    //std::cout << scripts_dir << std::endl;
    //std::cout << scenes << std::endl;

    /* Initialize random device */
    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_int_distribution<int> dist;


    /* Load gazebo as a client */
    #if GAZEBO_MAJOR_VERSION < 6
    gazebo::setupClient(argc, argv);
    #else
    gazebo::client::setup(argc, argv);
    #endif

    /* Create the communication node */
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    /* Publish to the object spawner request topic */
    gazebo::transport::PublisherPtr pub_spawner =
        node->Advertise<object_spawner_msgs::msgs::SpawnRequest>(OBJECT_SPAWNER_TOPIC);

    /* Subscribe to the object spawner reply topic and link callback function */
    gazebo::transport::SubscriberPtr sub_spawner = node->Subscribe(OBJECT_SPAWNER_REPLY_TOPIC, updateModelCount);

    /* Publish to the camera topic */
    gazebo::transport::PublisherPtr pub_camera =
        node->Advertise<camera_utils_msgs::msgs::CameraRequest>(CAMERA_UTILS_TOPIC);

     /* Subscribe to the camera utils reply topic and link callback function */
    gazebo::transport::SubscriberPtr sub_camera = node->Subscribe(CAMERA_UTILS_REPLY_TOPIC, updateCameraSuccess);

    /* Wait for a subscriber to connect */
    pub_spawner->WaitForConnection();

    /* Create a vector with the name of every texture in the textures dir */
    std::vector<std::string> textures;
    for (auto &p : fs::directory_iterator(scripts_dir)){
        std::string aux(fs::basename(p));
        textures.push_back(aux.c_str());
    }

    ignition::math::Quaternion<double> camera_orientation(0, M_PI/2.0, 0);
    int min_objects = 5;
    int max_objects = 10;
    
    spawnModelFromFile(
        pub_spawner, "models/custom_sun.sdf", true, false, false, textures);

    /* Main loop */
    for (int i = 0; i < scenes; i++){
    
        /* Random object number */
        int num_objects = (dist(mt) % max_objects) + min_objects;

        std::cout << "Number of objects:" << num_objects << std::endl;
        /* Spawn ground and camera */
        spawnModelFromFile(
            pub_spawner, "models/custom_ground.sdf", false, false, true, textures);


        spawnModelFromFile(
            pub_spawner, "models/custom_camera.sdf", false, true, false,
            textures, 2.5, 2.5, 3.5, camera_orientation);
        pub_camera->WaitForConnection();

        /* Spawn random objects */

        int x_cells=10;
        int y_cells=10;

        // Create 10 by 10 cell grid
        std::vector<int> cells_array;
        for (int i = 0;  i < x_cells * y_cells; ++i){
	       cells_array.push_back(i);
        }

        std::mt19937 g(rd());
        std::shuffle(cells_array.begin(), cells_array.end(), g);
 
        //std::copy(cells_array.begin(), cells_array.end(), std::ostream_iterator<int>(std::cout, " "));
        double grid_cell_size = 0.5;
        for (int j = 0; j < num_objects; ++j){
            unsigned int rand_cell_x = floor(cells_array[j]/x_cells);
            unsigned int rand_cell_y = floor(cells_array[j]-rand_cell_x*x_cells);
            spawnRandomObject(pub_spawner, textures, rand_cell_x, rand_cell_y, grid_cell_size);
        }

        while (waitForSpawner(num_objects + 2)){
            usleep(1000);
            queryModelCount(pub_spawner);
        }
        
        /* Still needed! */
        sleep(1);

        /* Disable physics */
        changePhysics(pub_spawner, false);

        /* Capture the scene and save it to a file */
        captureScene(pub_camera, i);
        
        while (waitForCamera()){
            usleep(1000);
        }

        /* Disable physics */
        changePhysics(pub_spawner, true);

        /* Clear the scene */
        clearWorld(pub_spawner);

        while (waitForSpawner(0)){
            usleep(1000);
            queryModelCount(pub_spawner);
        }
    }

    /* Shut down */
    #if GAZEBO_MAJOR_VERSION < 6
    gazebo::shutdown();
    #else
    gazebo::client::shutdown();
    #endif

    return 0;
}

/* Spawn objects */

void spawnModelFromFile(
    gazebo::transport::PublisherPtr pub,
    const std::string model_path,
    const bool is_light,
    const bool use_custom_pose,
    const bool use_custom_textures,
    std::vector<std::string> textures,
    const double & px, 
    const double & py,
    const double & pz,
    const ignition::math::Quaternion<double> & orientation){

    /* Read model sdf string from file */
    std::ifstream infile {model_path};
    std::string model_sdf { std::istreambuf_iterator<char>(infile), std::istreambuf_iterator<char>() };
    
    object_spawner_msgs::msgs::SpawnRequest msg;
    msg.set_type(SPAWN);
    if (is_light){
        msg.set_model_type(CUSTOM_LIGHT);
    } else {
        msg.set_model_type(CUSTOM);
    }
    msg.set_sdf(model_sdf);

    if (use_custom_pose){
        gazebo::msgs::Vector3d *pos = new gazebo::msgs::Vector3d();
        gazebo::msgs::Quaternion *ori = new gazebo::msgs::Quaternion(gazebo::msgs::Convert(orientation));
        gazebo::msgs::Pose *pose = new gazebo::msgs::Pose();
        pos->set_x(px);
        pos->set_y(py);
        pos->set_z(pz);
        pose->set_allocated_position(pos);
        pose->set_allocated_orientation(ori);
        msg.set_allocated_pose(pose);
    }

    if (use_custom_textures){
    /* Initialize random device */
    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_int_distribution<int> dist;
        int idx=dist(mt) % textures.size();

        std::string texture = textures.at(idx);
        std::stringstream texture_uri;
        std::stringstream texture_name;

        texture_uri << "file://materials/scripts/" << texture << ".material"
        << "</uri><uri>file://materials/textures/";
        texture_name << "Plugin/" << texture;

        msg.set_texture_uri(texture_uri.str());
        msg.set_texture_name(texture_name.str());
    }
    pub->Publish(msg);
}

void spawnRandomObject(
    gazebo::transport::PublisherPtr pub,
    std::vector<std::string> textures,
    unsigned int & x_cell,
    unsigned int & y_cell,
    double & grid_cell_size){

    /* Initialize random device */
    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_int_distribution<int> dist;

    object_spawner_msgs::msgs::SpawnRequest msg;
    
    msg.set_type(SPAWN);
    

    int model_aux=dist(mt) % 3;

    if (model_aux == 0){
        msg.set_model_type(CYLINDER);
    }
    else if(model_aux == 1){
        msg.set_model_type(BOX);
    }
    else if(model_aux == 2){
        msg.set_model_type(SPHERE);
    }

    /* External optional fields have to be allocated */
    gazebo::msgs::Vector3d *pos = new gazebo::msgs::Vector3d();
    gazebo::msgs::Quaternion *ori = new gazebo::msgs::Quaternion();
    gazebo::msgs::Pose *pose = new gazebo::msgs::Pose();
    gazebo::msgs::Vector3d *size = new gazebo::msgs::Vector3d();


    /*ori->set_x(0.0);
    ori->set_y(0.0);
    ori->set_z(0.0);
    ori->set_w(1.0);*/
    /* Mass */
    msg.set_mass(dist(mt) % 5 + 1.0);
    /* Sphere/cylinder radius */
    double radius=dRand(0.1,grid_cell_size*0.5);

    msg.set_radius(radius);

    /* Box size */ 
    double x_length=dRand(0.1,grid_cell_size);
    double y_length=dRand(0.1,grid_cell_size);    
    double z_length=dRand(0.1,grid_cell_size);

    size->set_x(x_length);
    size->set_y(y_length);
    size->set_z(z_length);
    
    /* Cylinder length */
    msg.set_length(z_length);

    /* Pose */
    ignition::math::Quaternion<double> object_orientation;

    if(dRand(0.5,1.0)<0.5)
    {
       // Horizontal
       double yaw=dRand(0.0,M_PI);


       object_orientation=ignition::math::Quaternion<double> (0.0, M_PI*0.5, yaw);
       pos->set_z(radius); //height is radius
    }
    else
    {

       double roll=dRand(0.0,M_PI);
       double pitch=dRand(0.0,M_PI);

       // Vertical
       object_orientation=ignition::math::Quaternion<double> (0.0, 0.0, 0.0);
       pos->set_z(z_length*0.5);
       if(model_aux == 2)
       {
           pos->set_z(radius);
       }
    }
	


    pos->set_x(x_cell*grid_cell_size+0.5*grid_cell_size);
    pos->set_y(y_cell*grid_cell_size+0.5*grid_cell_size);

    ori=new gazebo::msgs::Quaternion(gazebo::msgs::Convert(object_orientation));


    /* Material script */
    int idx = dist(mt) % textures.size();

    std::string texture = textures.at(idx);
    std::stringstream texture_uri;
    std::stringstream texture_name;

    texture_uri << "file://materials/scripts/" << texture << ".material"
    << "</uri><uri>file://materials/textures/";
    texture_name << "Plugin/" << texture;

    msg.set_texture_uri(texture_uri.str());
    msg.set_texture_name(texture_name.str());

    /* Associate dynamic fields */
    pose->set_allocated_position(pos);
    pose->set_allocated_orientation(ori);
    msg.set_allocated_pose(pose);
    msg.set_allocated_box_size(size);

    /* Send the message */
    pub->Publish(msg);
}

void clearWorld(gazebo::transport::PublisherPtr pub){

    object_spawner_msgs::msgs::SpawnRequest msg;
    msg.set_type(CLEAR);
    msg.set_name("plugin");
    pub->Publish(msg);
}

void changePhysics(gazebo::transport::PublisherPtr pub, bool enable){
    object_spawner_msgs::msgs::SpawnRequest msg;
    msg.set_type(TOGGLE);
    msg.set_state(enable);
    pub->Publish(msg);
}

void pauseWorld(gazebo::transport::PublisherPtr pub, bool enable){
    object_spawner_msgs::msgs::SpawnRequest msg;
    msg.set_type(PAUSE);
    msg.set_state(enable);
    pub->Publish(msg);
}


void captureScene(gazebo::transport::PublisherPtr pub, int idx){

    camera_utils_msgs::msgs::CameraRequest msg;
    msg.set_type(CAPTURE);
    msg.set_file_name(std::to_string(idx));
    pub->Publish(msg);
}

/* Handle object count */

bool waitForSpawner(int desired_objects){
    std::lock_guard<std::mutex> lock(object_count_mutex);
    if (desired_objects == object_count)
        return false;
    return true;
}

void queryModelCount(gazebo::transport::PublisherPtr pub){
    object_spawner_msgs::msgs::SpawnRequest msg;
    msg.set_type(STATUS);
    pub->Publish(msg);
}

void updateModelCount(SpawnerReplyPtr &_msg){
    if (_msg->type() == INFO){
        if (_msg->has_object_count()){
            std::lock_guard<std::mutex> lock(object_count_mutex);
            object_count = _msg->object_count();
        }
    }
}

/* Handle camera success */

bool waitForCamera(){
    std::lock_guard<std::mutex> lock(camera_success_mutex);
    if (camera_success){
        camera_success = false;
        return false;
    }
    return true;
}

void updateCameraSuccess(CameraReplyPtr &_msg){
    if (_msg->success()){
        std::lock_guard<std::mutex> lock(camera_success_mutex);
        camera_success = true;    
    } else {
        std::cout << "Camera could not save to file! Exiting..." << std::endl;
        exit(EXIT_FAILURE);
    }
}