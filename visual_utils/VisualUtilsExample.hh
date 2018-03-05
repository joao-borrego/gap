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

// For sleeps
#include <chrono>
#include <thread>

// Custom messages
#include "visual_utils_request.pb.h"

/// \brief Topic monitored by the server for incoming commands
#define VISUAL_UTILS_TOPIC "~/gazebo-utils/visual_utils"

/// \brief Request update
#define UPDATE  visual_utils::msgs::VisualUtilsRequest::UPDATE
