/*! 
    \file examples/visual_example/visual_example.hh
    \brief Visual tools example client

    An example client to interact with VisualUtils plugin
    
    \author João Borrego : jsbruglie
    \author Rui Figueiredo : ruipimentelfigueiredo
*/

// Gazebo
#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

// I/O streams
#include <iostream>
// For sleeps
#include <chrono>
#include <thread>

// Custom messages
#include "visual_utils_request.pb.h"

/// Topic monitored by the server for incoming commands
#define VISUAL_UTILS_TOPIC "~/gazebo-utils/visual_utils"

/// Request update
#define UPDATE  visual_utils::msgs::VisualUtilsRequest::UPDATE
