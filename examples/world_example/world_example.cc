/*!
    \file examples/world_example/world_example.cc
    \brief World tools example client implementation

    An example client to interact with WorldlUtils plugin

    \author João Borrego : jsbruglie
    \author Rui Figueiredo : ruipimentelfigueiredo
*/

#include "world_example.hh"

namespace fs = boost::filesystem;

// Regular expression for replacing texture img in file
const std::string g_material_reg_str = "(?:material )(.+)(?=\n)";
const std::string g_texture_reg_str = "(?:texture )(.+)(?=\n)";
std::regex g_material_reg(g_material_reg_str);
std::regex g_texture_reg(g_texture_reg_str);

void changeTexture(
    const std::string & material_script,
    int material_id,
    const std::string & texture_file)
{

    std::ifstream ifs(material_script);
    std::string file_content(
        (std::istreambuf_iterator<char>(ifs)),
        (std::istreambuf_iterator<char>()));

    std::stringstream new_material;
    new_material << "material Plugin/" << material_id;

    std::stringstream new_texture;
    new_texture << "texture " << texture_file.c_str();

    std::string aux = std::regex_replace(
        file_content.c_str(), g_material_reg, new_material.str());

    std::string new_file_content = std::regex_replace(
        aux, g_texture_reg, new_texture.str());

    std::ofstream ofs(material_script);
    ofs << new_file_content;
    ofs.close();
}

int main(int _argc, char **_argv)
{
    std::vector<std::string> textures;

    // Create a vector with the name of every texture in the textures dir
    for (auto &p : fs::directory_iterator(SCRIPTS_DIR)) {
        std::string aux(fs::basename(p));
        textures.push_back(aux.c_str());
    }

    // Load gazebo as a client
    gazebo::client::setup(_argc, _argv);

    // Create the communication node
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    // Publish to the object spawner topic
    gazebo::transport::PublisherPtr pub =
        node->Advertise<world_utils::msgs::WorldUtilsRequest>(WORLD_UTILS_TOPIC);

    // Wait for a subscriber to connect to this publisher
    pub->WaitForConnection();

    // Initialize random seed
    srand(time(NULL));

    // Main loop
    std::string line = "";

    while(std::cout << PROMPT){

        getline(std::cin, line);
        std::stringstream input_stream(line);

        // Create a custom message (placeholder message)
        world_utils::msgs::WorldUtilsRequest msg;

        // Fill the contents of the message
        msg.set_type(SPAWN);

        int model_aux = rand() % 3;
        world_utils::msgs::Object* object = msg.add_object();
        if(model_aux == 0){
            object->set_model_type(CYLINDER);
        }
        else if(model_aux == 1){
            object->set_model_type(BOX);
        }
        else if(model_aux == 2){
            object->set_model_type(SPHERE);
        }

        std::string command = input_stream.str();
        if (command == "remove"){
            msg.set_type(REMOVE);
            object->set_name("plugin");
        } else if (command == "remove_light"){
            msg.set_type(REMOVE);
            object->set_model_type(CUSTOM_LIGHT);
            object->set_name("plugin");

        } else if (command == "ground"){
            object->set_model_type(CUSTOM);
            std::ifstream infile { "models/custom_ground.sdf" };
            std::string model_sdf { std::istreambuf_iterator<char>(infile), std::istreambuf_iterator<char>() };
            object->set_sdf(model_sdf);
        } else if (command == "light"){
            object->set_model_type(CUSTOM_LIGHT);
            std::ifstream infile { "models/custom_sun.sdf" };
            std::string model_sdf { std::istreambuf_iterator<char>(infile), std::istreambuf_iterator<char>() };
            object->set_sdf(model_sdf);
        }

        // External optional fields have to be allocated
        gazebo::msgs::Vector3d *pos = new gazebo::msgs::Vector3d();
        gazebo::msgs::Quaternion *ori = new gazebo::msgs::Quaternion();
        gazebo::msgs::Pose *pose = new gazebo::msgs::Pose();
        gazebo::msgs::Vector3d *size = new gazebo::msgs::Vector3d();

        // Assign values to fields

        // Pose
        pos->set_x((rand() % 500 - 250) / 20.0);
        pos->set_y((rand() % 500 - 250) / 20.0);
        pos->set_z((rand() % 50) / 20.0);
        ori->set_x(0.0);
        ori->set_y(0.0);
        ori->set_z(0.0);
        ori->set_w(0.0);
        // Mass
        object->set_mass(rand() % 5 + 1.0);
        // Sphere/cylinder radius
        object->set_radius((rand() % 20 + 1.0) / 5.0);
        // Cylinder length
        object->set_length((rand() % 20 + 5.0) / 3.0);
        // Box size
        size->set_x((rand() % 20 + 5.0) / 3.0);
        size->set_y((rand() % 20 + 5.0) / 3.0);
        size->set_z((rand() % 20 + 5.0) / 3.0);

        // Material script
        int idx = rand() % textures.size();

        std::string texture = textures.at(idx);
        std::stringstream texture_uri;
        std::stringstream texture_name;

        texture_uri << "file://materials/scripts/" << texture << ".material"
        << "</uri><uri>file://materials/textures/";
        texture_name << "Plugin/" << texture;

        object->set_texture_uri(texture_uri.str());
        object->set_texture_name(texture_name.str());

        // Associate dynamic fields
        if (command != "ground"){
            pose->set_allocated_position(pos);
            pose->set_allocated_orientation(ori);
            object->set_allocated_pose(pose);
            object->set_allocated_box_size(size);
        }

        // Send the message
        pub->Publish(msg);
    }

    // Shut down
    gazebo::client::shutdown();
    return 0;
}