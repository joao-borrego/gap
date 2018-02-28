/// \file scene_example/scene_example.hh
///
/// \brief TODO
///
/// TODO
///

// Includes

// Gazebo
#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

// Custom messages 
#include "camera_utils_request.pb.h"
#include "camera_utils_response.pb.h"
#include "visual_utils_request.pb.h"
#include "world_utils_request.pb.h"
#include "world_utils_response.pb.h"

// Utilities
#include "utils.hh"
// Object class
#include "object.hh"

// I/O streams 
#include <iostream>
// File streams 
#include <fstream>
// Iterating over the contents of a dir 
#include <boost/filesystem.hpp>
// Protecting variables 
#include <mutex>
// Sleep 
#include <chrono>
#include <thread>
// Regular expressions
#include <regex>
// TODO 
#include <Eigen/Dense>

/// OpenCV 2 
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc.hpp"

//////////////////////////////////////////////////

// Macros

/// Matches name field in <model name=""> XML tag
#define REGEX_XML_MODEL "<model name=(\"([^\"]|\"\")*\")>"
/// Matches string enclosed in <uid> XML tags
#define REGEX_XML_UID   "<uid>[\\s\\S]*?<\\/uid>"

//////////////////////////////////////////////////

// Macros for custom messages

// Visual utils

/// Request update
#define UPDATE  visual_utils::msgs::VisualUtilsRequest::UPDATE

// World utils 

/// Spawn entity 
#define SPAWN           world_utils::msgs::WorldUtilsRequest::SPAWN
/// Move entity 
#define MOVE            world_utils::msgs::WorldUtilsRequest::MOVE
/// Remove entity from the world 
#define REMOVE          world_utils::msgs::WorldUtilsRequest::REMOVE
/// Start or stop physcis simulation 
#define PHYSICS         world_utils::msgs::WorldUtilsRequest::PHYSICS

/// Spawn custom object 
#define CUSTOM          world_utils::msgs::Object::CUSTOM
/// Spawn custom light object 
#define CUSTOM_LIGHT    world_utils::msgs::Object::CUSTOM_LIGHT

//////////////////////////////////////////////////

// API Topics

/// Topic monitored by the server for incoming camera requests 
#define CAMERA_UTILS_TOPIC          "~/gazebo-utils/camera_utils"
/// Topic for receiving replies from the camera plugin server  
#define CAMERA_UTILS_RESPONSE_TOPIC "~/gazebo-utils/camera_utils/response"
/// Topic monitored by the server for incoming object spawn requests 
#define WORLD_UTILS_TOPIC           "~/gazebo-utils/world_utils"
/// Topic for receiving replies from the object spawner server 
#define WORLD_UTILS_RESPONSE_TOPIC  "~/gazebo-utils/world_utils/response"
// Topic monitored by the server for incoming commands
#define VISUAL_UTILS_TOPIC          "~/gazebo-utils/visual_utils"

// Message pointer typedefs

typedef const boost::shared_ptr<const world_utils::msgs::WorldUtilsResponse>
    WorldUtilsResponsePtr;
typedef const boost::shared_ptr<const camera_utils::msgs::CameraUtilsResponse>
    CameraUtilsResponsePtr;

//////////////////////////////////////////////////

/// Function prototypes

/// \brief Adds an SDF model to a WorldUtils request
/// \param msg
/// \param file
void addModelFromFile(
    world_utils::msgs::WorldUtilsRequest & msg,
    const std::string & file);

/// \brief TODO
void addDynamicModels(world_utils::msgs::WorldUtilsRequest & msg);

/// \brief TODO
void updateObjects(visual_utils::msgs::VisualUtilsRequest & msg);

/// \brief Returns true if number of objects have not spawned yet
/// \param num_objects Desired number of objects
/// \return False if desired number of objects have spawned
bool waitForObjectCount(int num_objects);

/// \brief Updates World object count
/// \param pub WorldUtils publisher ptr
void queryObjectCount(gazebo::transport::PublisherPtr pub);

/// \brief Callback function for WordUtils response
/// \param _msg Incoming message
void onWorldUtilsResponse(WorldUtilsResponsePtr & _msg);

/// \brief Callback function for CameraUtils response
/// \param _msg Incoming message
void onCameraUtilsResponse(CameraUtilsResponsePtr & _msg);

/// \brief Enables/disables physics engine
/// \param pub WorldUtils publisher ptr
/// \param enable Desired physics engine status
void setPhysics(gazebo::transport::PublisherPtr pub, bool enable);