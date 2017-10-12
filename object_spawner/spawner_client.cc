#include "spawner_client.hh"

/**
 * @brief      Client app for object spawner interface
 *
 * @param[in]  _argc  The argc
 * @param      _argv  The argv
 *
 * @return     0
 */
int main(int _argc, char **_argv)
{
    /* Load gazebo as a client */
    #if GAZEBO_MAJOR_VERSION < 6
    gazebo::setupClient(_argc, _argv);
    #else
    gazebo::client::setup(_argc, _argv);
    #endif

    /* Create the communication node */
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    /* Publish to the object spawner topic */
    gazebo::transport::PublisherPtr pub =
        node->Advertise<object_spawner_msgs::msgs::SpawnRequest>
        ("~/gazebo-utils/object_spawner");

    /* Wait for a subscriber to connect to this publisher */
    pub->WaitForConnection();

    /* Initialize random seed */
    srand(time(NULL));

    /* Main loop */
    std::string line = "";

    while(std::cout << PROMPT){

        getline(std::cin, line);
        std::stringstream input_stream(line);

        /* Create a custom message (placeholder message) */
        object_spawner_msgs::msgs::SpawnRequest msg;

        /* Fill the contents of the message */
        msg.set_type(SPAWN);
        msg.set_model_type(BOX);
        /* External optional fields have to be allocated */
        gazebo::msgs::Vector3d *pos = new gazebo::msgs::Vector3d();
        gazebo::msgs::Quaternion *ori = new gazebo::msgs::Quaternion();
        gazebo::msgs::Pose *pose = new gazebo::msgs::Pose();
        gazebo::msgs::Vector3d *size = new gazebo::msgs::Vector3d();

        /* Assign values to fields */

        /* Pose */
        pos->set_x((rand()%500 - 250) / 20.0);
        pos->set_y((rand()%500 - 250) / 20.0);
        pos->set_z((rand()%50) / 20.0);
        ori->set_x(0.0);
        ori->set_y(0.0);
        ori->set_z(0.0);
        ori->set_w(0.0);
        /* Mass */
        msg.set_mass(rand()%5 + 1.0);
        /* Sphere/cylinder radius */
        msg.set_radius((rand()%20 + 1.0) / 5.0);
        /* Cylinder length */
        msg.set_length((rand()%20 + 5.0) / 3.0);
        /* Box size */
        size->set_x((rand()%20 + 5.0) / 3.0);
        size->set_y((rand()%20 + 5.0) / 3.0);
        size->set_z((rand()%20 + 5.0) / 3.0);
        /* Material script */
        msg.set_texture_uri("model://cylinder/materials/scripts</uri>\
            <uri>model://cylinder/materials/textures");
        msg.set_texture_name("Cylinder/Diffuse");

        /* Associate dynamic fields */
        pose->set_allocated_position(pos);
        pose->set_allocated_orientation(ori);
        msg.set_allocated_pose(pose);
        msg.set_allocated_box_size(size);

        /* Send the message */
        pub->Publish(msg);
    }

    /* Shut down */
    #if GAZEBO_MAJOR_VERSION < 6
    gazebo::shutdown();
    #else
    gazebo::client::shutdown();
    #endif

    return 0;
}