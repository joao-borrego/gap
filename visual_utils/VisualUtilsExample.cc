/// \file visual_utils/VisualUtilsExample.cc
/// \brief Visual Utils plugin example client implementation
/// \author João Borrego

#include "VisualUtilsExample.hh"

using namespace visual_utils::msgs;

/// \brief Main function for Visual plugin example client
/// 
/// This simple demo changes the material of 4 objects.
/// To launch run:
/// \code
///	gazebo worlds/visual.world
/// ./build/visual_utils/visual_example
/// \endcode
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
    for (int i = 0; i < 10; i++){
        
        // Create and send a custom message
        VisualUtilsRequest msg;
        msg.set_type(UPDATE);
        // Define targets of request
        msg.add_targets(std::string("box_1"));
        msg.add_targets(std::string("sphere_1"));
        msg.add_targets(std::string("cylinder_1"));
        msg.add_targets(std::string("ground_1"));
        pub->Publish(msg);
    	// Wait 500 milliseconds
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // Clean up
    gazebo::client::shutdown();
    return 0;
}
