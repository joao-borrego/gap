/// \file visual_utils/VisualUtilsExample.cc
/// \brief Visual Utils plugin example client implementation
/// \author João Borrego

#include "VisualUtilsExample.hh"

using namespace visual_utils::msgs;

/// \brief TODO
int main(int argc, char **argv)
{

    // Setup communication

    // Setup Gazebo client
    gazebo::client::setup(argc, argv);
    // Create the communication node
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();
    // Publish to the visual plugin topic
    gazebo::transport::PublisherPtr pub =
        node->Advertise<VisualUtilsRequest>(VISUAL_UTILS_TOPIC);
    // Wait for a subscriber to connect to this publisher
    pub->WaitForConnection();

    // Main loop
    std::string line = "";

    while(std::cout << PROMPT){

        getline(std::cin, line);
        std::stringstream input_stream(line);

        // Create and send a custom message
        VisualUtilsRequest msg;
        msg.set_type(UPDATE_REQUEST);
        pub->Publish(msg);
    }

    // Clean up
    gazebo::client::shutdown();
    return 0;
}
