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

    std::cout << scripts_dir << std::endl;
    std::cout << scenes << std::endl;

    /* Load gazebo as a client */
    #if GAZEBO_MAJOR_VERSION < 6
    gazebo::setupClient(argc, argv);
    #else
    gazebo::client::setup(argc, argv);
    #endif

    /* Create the communication node */
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    /* Publish to the object spawner topic */
    gazebo::transport::PublisherPtr pub_spawner =
        node->Advertise<object_spawner_msgs::msgs::SpawnRequest>(OBJECT_SPAWNER_TOPIC);

    /* Publish to the camera topic */
    gazebo::transport::PublisherPtr pub_camera =
        node->Advertise<camera_utils_msgs::msgs::CameraRequest>(CAMERA_UTILS_TOPIC);

    /* Wait for a subscriber to connect */
    pub_spawner->WaitForConnection();


    /* TODO - Replace rand by better function */
    srand(time(NULL));

    /* Create a vector with the name of every texture in the textures dir */
    std::vector<std::string> textures;
    for (auto &p : fs::directory_iterator(scripts_dir)){
        std::string aux(fs::basename(p));
        textures.push_back(aux.c_str());
    }

    /* Disable physics */
    //togglePhysics(pub_spawner);
    
    /* Main loop */

    for (int i = 0; i < scenes; i++){

        /* Spawn ground and camera */
        spawnModelFromFile(pub_spawner, "models/custom_ground.sdf");
        spawnModelFromFile(pub_spawner, "models/custom_camera.sdf");
        /* Wait for a subscriber to connect */
        pub_camera->WaitForConnection();

        /* Spawn random objects */
        int num_objects = rand() % 7 + 3;
        for (int j = 0; j < num_objects; j++){
            spawnRandomObject(pub_spawner, textures);
        }

        /* Make sure the server has updated */
        sleep(1);

        /* Capture the scene and save it to a file */
        captureScene(pub_camera);

        sleep(1);

        /* Clear the scene */
        clearWorld(pub_spawner);

        sleep(2);

    }

    /* Shut down */
    #if GAZEBO_MAJOR_VERSION < 6
    gazebo::shutdown();
    #else
    gazebo::client::shutdown();
    #endif

    return 0;
}

void spawnModelFromFile(
    gazebo::transport::PublisherPtr pub,
    const std::string model_path){

    // TODO - Retexture custom file
    object_spawner_msgs::msgs::SpawnRequest msg;
    msg.set_type(SPAWN);
    msg.set_model_type(CUSTOM);
    std::ifstream infile {model_path};
    std::string model_sdf { std::istreambuf_iterator<char>(infile), std::istreambuf_iterator<char>() };
    msg.set_sdf(model_sdf);
    pub->Publish(msg);

}

void spawnRandomObject(
    gazebo::transport::PublisherPtr pub,
    std::vector<std::string> textures){

    object_spawner_msgs::msgs::SpawnRequest msg;
    
    msg.set_type(SPAWN);
    
    int model_aux = rand() % 3;
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

    /* Pose */
    pos->set_x((rand() % 500 - 250) / 200.0);
    pos->set_y((rand() % 500 - 250) / 200.0);
    pos->set_z((rand() % 50) / 200.0);
    ori->set_x(0.0);
    ori->set_y(0.0);
    ori->set_z(0.0);
    ori->set_w(0.0);
    /* Mass */
    msg.set_mass(rand() % 5 + 1.0);
    /* Sphere/cylinder radius */
    msg.set_radius((rand() % 20 + 1.0) / 20.0);
    /* Cylinder length */
    msg.set_length((rand() % 20 + 1.0) / 20.0);
    /* Box size */ 
    size->set_x((rand() % 20 + 5.0) / 20.0);
    size->set_y((rand() % 20 + 5.0) / 20.0);
    size->set_z((rand() % 20 + 5.0) / 20.0);
    
    /* Material script */
    int idx = rand() % textures.size();

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
    pub->Publish(msg);
}

void togglePhysics(gazebo::transport::PublisherPtr pub){
    object_spawner_msgs::msgs::SpawnRequest msg;
    msg.set_type(TOGGLE);
    pub->Publish(msg);
}

void captureScene(gazebo::transport::PublisherPtr pub){

    camera_utils_msgs::msgs::CameraRequest msg;
    msg.set_type(CAPTURE);
    pub->Publish(msg);
}
