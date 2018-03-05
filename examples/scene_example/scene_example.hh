/// \file scene_example/scene_example.hh
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
// Linear algebra
#include <Eigen/Dense>
// INT MAX
#include <climits>

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

// Camera utils

/// \brief Request to move camera to given pose
#define MOVE_REQUEST            camera_utils::msgs::CameraUtilsRequest::MOVE
/// \brief Response acknowledging move camera request
#define MOVE_RESPONSE           camera_utils::msgs::CameraUtilsResponse::MOVE
/// \brief Request to capture a frame and save it to disk
#define CAPTURE_REQUEST         camera_utils::msgs::CameraUtilsRequest::CAPTURE
/// \brief Response acknowledging captured frame
#define CAPTURE_RESPONSE        camera_utils::msgs::CameraUtilsResponse::CAPTURE
/// \brief Request 3D to 2D point projection
#define PROJECTION_REQUEST      camera_utils::msgs::CameraUtilsRequest::PROJECTION
/// \brief Response 3D to 2D point projection
#define PROJECTION_RESPONSE     camera_utils::msgs::CameraUtilsResponse::PROJECTION

// Visual utils

/// \brief Request update
#define UPDATE  visual_utils::msgs::VisualUtilsRequest::UPDATE

// World utils 

/// \brief Spawn entity 
#define SPAWN           world_utils::msgs::WorldUtilsRequest::SPAWN
/// \brief Move entity 
#define WORLD_MOVE      world_utils::msgs::WorldUtilsRequest::MOVE
/// \brief Start or stop physcis simulation 
#define PHYSICS         world_utils::msgs::WorldUtilsRequest::PHYSICS
/// \breief TODO
#define SUCCESS         world_utils::msgs::WorldUtilsResponse::SUCCESS

/// \brief Spawn custom object 
#define CUSTOM          world_utils::msgs::Object::CUSTOM
/// \brief Spawn custom light object 
#define CUSTOM_LIGHT    world_utils::msgs::Object::CUSTOM_LIGHT

//////////////////////////////////////////////////

// API Topics

/// \brief Topic monitored by the server for incoming camera requests 
#define CAMERA_UTILS_TOPIC          "~/gazebo-utils/camera_utils"
/// \brief Topic for receiving replies from the camera plugin server  
#define CAMERA_UTILS_RESPONSE_TOPIC "~/gazebo-utils/camera_utils/response"
/// \brief Topic monitored by the server for incoming commands
#define VISUAL_UTILS_TOPIC          "~/gazebo-utils/visual_utils"
/// \brief Topic monitored by the server for incoming object spawn requests 
#define WORLD_UTILS_TOPIC           "~/gazebo-utils/world_utils"
/// \brief Topic for receiving replies from the object spawner server 
#define WORLD_UTILS_RESPONSE_TOPIC  "~/gazebo-utils/world_utils/response"

// Message pointer typedefs

/// \brief Pointer to Camera Utils response message
typedef const boost::shared_ptr<const camera_utils::msgs::CameraUtilsResponse>
    CameraUtilsResponsePtr;
/// \brief Pointer to World Utils request message
typedef const boost::shared_ptr<const world_utils::msgs::WorldUtilsResponse>
    WorldUtilsResponsePtr;

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

/// \brief TODO
void addMoveObject(
    world_utils::msgs::WorldUtilsRequest & msg,
    const std::string & name,
    const bool is_light,
    const ignition::math::Pose3d & pose);

/// \brief TODO - Cleanup
ignition::math::Pose3d getRandomCameraPose();

/// \brief TODO - Cleanup
ignition::math::Pose3d getRandomLightPose();

/// \brief TODO
void captureScene(gazebo::transport::PublisherPtr pub, int iteration);

/// \brief TODO
bool waitForMove();

/// \brief TODO
bool waitForCamera();

/// \brief TODO
bool waitForProjections();

/// \brief TODO
void addProjections(camera_utils::msgs::CameraUtilsRequest & msg);

/// \brief TODO
void moveCamera(gazebo::transport::PublisherPtr pub);

/// \brief TODO
/// \param _msg Incoming message
void onWorldUtilsResponse(WorldUtilsResponsePtr & _msg);

/// \brief Callback function for CameraUtils response
/// \param _msg Incoming message
void onCameraUtilsResponse(CameraUtilsResponsePtr & _msg);

/// \brief Enables/disables physics engine
/// \param pub WorldUtils publisher ptr
/// \param enable Desired physics engine status
void setPhysics(gazebo::transport::PublisherPtr pub, bool enable);

/// \brief TODO
void visualizeData(const std::string & image_dir, int iteration);

/// \brief TODO
void storeAnnotations(
    const std::string & path,
    const int iteration);
