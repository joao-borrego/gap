/* Gazebo */
#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

/* I/O streams */
#include <iostream>

/* Custom messages */
#include "camera_utils_request.pb.h"

/**
 * Gazebo's API has changed between major releases. These changes are
 * accounted for with #if..#endif blocks in this file.
 */
#if GAZEBO_MAJOR_VERSION < 6
#include <gazebo/gazebo.hh>
#else
#include <gazebo/gazebo_client.hh>
#endif

/** Request to capture a frame and save it to disk */
#define CAPTURE camera_utils::msgs::CameraUtilsRequest::CAPTURE

/** Command prompt */
#define PROMPT "> "
/** Topic monitored by the server for incoming commands */
#define CAMERA_UTILS_TOPIC "~/gazebo-utils/camera_utils"