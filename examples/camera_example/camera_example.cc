/*!
    \file examples/camera_example/camera_example.hh
    \brief Camera tools example client implementation

    An example client to interact with CameraUtils plugin

    \author João Borrego : jsbruglie
    \author Rui Figueiredo : ruipimentelfigueiredo
*/

#include "camera_example.hh"

using namespace camera_utils::msgs;

int main(int _argc, char **_argv)
{

    // Load gazebo as a client
    gazebo::client::setup(_argc, _argv);

    // Create the communication node
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    // Publish to the object spawner topic
    gazebo::transport::PublisherPtr pub =
        node->Advertise<CameraUtilsRequest>(CAMERA_UTILS_TOPIC);

    // Wait for a subscriber to connect to this publisher
    pub->WaitForConnection();

    // Create a custom message to aqcuire a frame
    CameraUtilsRequest msg;
    msg.set_type(CAPTURE);
    // Send the message
    pub->Publish(msg);

    // Shut down
    gazebo::client::shutdown();
    return 0;
}
