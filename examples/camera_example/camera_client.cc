#include "camera_client.hh"

using namespace camera_utils::msgs;

/**
 * @brief      Client app for camera utils interface
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
        node->Advertise<CameraUtilsRequest>(CAMERA_UTILS_TOPIC);

    /* Wait for a subscriber to connect to this publisher */
    pub->WaitForConnection();

    /* Main loop */
    std::string line = "";

    while(std::cout << PROMPT){

        getline(std::cin, line);
        std::stringstream input_stream(line);

        /* Create a custom message (placeholder message) */
        CameraUtilsRequest msg;

        msg.set_type(CAPTURE);

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
