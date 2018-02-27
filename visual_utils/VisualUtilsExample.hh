/// \file visual_utils/VisualUtilsExample.hh
/// \brief Visual Utils plugin example client headers
/// \author João Borrego

// Gazebo
#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

// I/O streams
#include <iostream>

/* Custom messages */
#include "visual_utils_request.pb.h"

// Command prompt
#define PROMPT "> "

// Topic monitored by the server for incoming commands
#define VISUAL_UTILS_TOPIC "~/gazebo-utils/visual_utils"

/// Request update
#define UPDATE_REQUEST  visual_utils::msgs::VisualUtilsRequest::UPDATE
