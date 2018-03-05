/*!
    \file examples/camera_example/camera_example.hh
    \brief Camera tools example client

    An example client to interact with CameraUtils plugin

    \author João Borrego : jsbruglie
    \author Rui Figueiredo : ruipimentelfigueiredo
*/

// Gazebo
#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

// I/O streams
#include <iostream>

// Custom messages
#include "camera_utils_request.pb.h"

/// Request to capture a frame and save it to disk
#define CAPTURE camera_utils::msgs::CameraUtilsRequest::CAPTURE

/// Topic monitored by the server for incoming commands
#define CAMERA_UTILS_TOPIC "~/gazebo-utils/camera_utils"
