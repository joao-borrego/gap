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
        msg.set_type(SPAWN_SPHERE);
        /* External optional fields have to be allocated */
        gazebo::msgs::Vector3d *pos = new gazebo::msgs::Vector3d();
        gazebo::msgs::Quaternion *ori = new gazebo::msgs::Quaternion();
        gazebo::msgs::Pose *pose = new gazebo::msgs::Pose();
        /* Assign values to fields */
        pos->set_x((rand()%500 - 250) / 20.0);
        pos->set_y((rand()%500 - 250) / 20.0);
        pos->set_z((rand()%50) / 20.0);
        ori->set_x(0.0);
        ori->set_y(0.0);
        ori->set_z(0.0); 
        ori->set_w(0.0);
        msg.set_mass(rand()%5 + 1.0);
        msg.set_radius((rand()%20 + 1.0) / 5.0);
        /* Associate dynamic fields */
        pose->set_allocated_position(pos);
        pose->set_allocated_orientation(ori);
        msg.set_allocated_pose(pose);

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