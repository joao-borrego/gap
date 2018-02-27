/// \file scene_example/scene_example.hh
///
/// \brief TODO
///
/// TODO
///

/// Includes

/// Gazebo
#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

/// Custom messages 

/// Camera utils request 
#include "camera_utils_request.pb.h"
/// Camera utils reply 
#include "camera_utils_response.pb.h"
/// World utils request 
#include "world_utils_request.pb.h"
/// World utils response 
#include "world_utils_response.pb.h"

/// Utilities
#include "utils.hh"
/// Object class
#include "object.hh"

/// I/O streams 
#include <iostream>
/// File streams 
#include <fstream>
/// For iterating over the contents of a dir 
#include <boost/filesystem.hpp>
/// For protecting variables 
#include <mutex>
/// For sleeps 
#include <chrono>
#include <thread>
/// TODO 
#include <Eigen/Dense>

/// OpenCV 2 
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc.hpp"

//////////////////////////////////////////////////

/// Macros for custom messages
 
/// Camera utils 

/// Request for camera object info 
#define CAMERA_INFO_REQUEST     camera_utils::msgs::CameraUtilsRequest::CAMERA_INFO
/// Request to capture a frame and save it to disk 
#define CAPTURE_REQUEST         camera_utils::msgs::CameraUtilsRequest::CAPTURE
/// Request for projection of 3D point in world to 2D camera reference plane 
#define CAMERA_POINT_REQUEST    camera_utils::msgs::CameraUtilsRequest::CAMERA_POINT

/// TODO
#define CAMERA_INFO_RESPONSE    camera_utils::msgs::CameraUtilsResponse::CAMERA_INFO
/// TODO
#define CAPTURE_RESPONSE        camera_utils::msgs::CameraUtilsResponse::CAPTURE
/// TODO
#define CAMERA_POINT_RESPONSE   camera_utils::msgs::CameraUtilsResponse::CAMERA_POINT

/// World utils 

/// Spawn entity 
#define SPAWN           world_utils::msgs::WorldUtilsRequest::SPAWN
/// Move entity 
#define MOVE            world_utils::msgs::WorldUtilsRequest::MOVE
/// Remove entity from the world 
#define REMOVE          world_utils::msgs::WorldUtilsRequest::REMOVE
/// Start or stop physcis simulation 
#define PHYSICS         world_utils::msgs::WorldUtilsRequest::PHYSICS
/// Pause or resume simulation 
#define PAUSE           world_utils::msgs::WorldUtilsRequest::PAUSE
/// Get entity or world information 
#define STATUS          world_utils::msgs::WorldUtilsRequest::STATUS

/// Spawn sphere object 
#define SPHERE          world_utils::msgs::Object::SPHERE
/// Spawn cylinder object 
#define CYLINDER        world_utils::msgs::Object::CYLINDER
/// Spawn box object 
#define BOX             world_utils::msgs::Object::BOX
/// Spawn custom object 
#define CUSTOM          world_utils::msgs::Object::CUSTOM
/// Spawn custom light object 
#define CUSTOM_LIGHT    world_utils::msgs::Object::CUSTOM_LIGHT
/// Spawn a model included in gazebo model path 
#define MODEL           world_utils::msgs::Object::MODEL

/// Provide world state information 
#define INFO            world_utils::msgs::WorldUtilsResponse::INFO
/// Provide specific object state information 
#define PROPERTIES      world_utils::msgs::WorldUtilsResponse::PROPERTIES

//////////////////////////////////////////////////

/// API Topics

/// Topic monitored by the server for incoming camera requests 
#define CAMERA_UTILS_TOPIC          "~/gazebo-utils/camera_utils"
/// Topic for receiving replies from the camera plugin server  
#define CAMERA_UTILS_RESPONSE_TOPIC "~/gazebo-utils/camera_utils/response"
/// Topic monitored by the server for incoming object spawn requests 
#define WORLD_UTILS_TOPIC           "~/gazebo-utils/world_utils"
/// Topic for receiving replies from the object spawner server 
#define WORLD_UTILS_RESPONSE_TOPIC  "~/gazebo-utils/world_utils/response"

/// Message pointer typedefs

typedef const boost::shared_ptr<const world_utils::msgs::WorldUtilsResponse>
    WorldUtilsResponsePtr;
typedef const boost::shared_ptr<const camera_utils::msgs::CameraUtilsResponse>
    CameraUtilsResponsePtr;

//////////////////////////////////////////////////

/// Function prototypes

/// \brief Adds an SDF model to a WorldUtils request
/// \param msg
/// \param file
/// \param custom_texture
/// \param textures
void addModelFromFile(
    world_utils::msgs::WorldUtilsRequest & msg,
    const std::string & file,
    const bool custom_texture,
    std::vector<std::string> & textures);

/// \brief Adds a random custom texture field to a message object
/// \param object
/// \param textures Array of textures
void addCustomTexture(
    world_utils::msgs::Object *object,
    std::vector<std::string> & textures);

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
