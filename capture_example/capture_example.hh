/* Gazebo */
#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#if GAZEBO_MAJOR_VERSION < 6
#include <gazebo/gazebo.hh>
#else
#include <gazebo/gazebo_client.hh>
#endif

/* Program options */
#include<boost/program_options.hpp>
/* I/O streams */
#include <iostream>
/* File streams */
#include <fstream>
/* For iterating over the contents of a dir */
#include <boost/filesystem.hpp>
/* For protecting variables */
#include <mutex>
/* For sleeps */
#include <unistd.h>

/*
 * Custom messages
 */

/* Camera utils request */
#include "camera_utils_request.pb.h"
/* Camera utils reply */
#include "camera_utils_response.pb.h"
/* World utils request */
#include "world_utils_request.pb.h"
/* World utils response */
#include "world_utils_response.pb.h"

/*
 * Macros for custom messages
 */

/* Camera utils */

/** Request to capture a frame and save it to disk */
#define CAPTURE         camera_utils::msgs::CameraUtilsRequest::CAPTURE

#define CAMERA_POINT         camera_utils::msgs::CameraUtilsRequest::CAMERA_POINT

/* World utils */

/* Request */

/** Spawn entity */
#define SPAWN           world_utils::msgs::WorldUtilsRequest::SPAWN
/** Move entity */
#define MOVE            world_utils::msgs::WorldUtilsRequest::MOVE
/** Remove entity from the world */
#define REMOVE          world_utils::msgs::WorldUtilsRequest::REMOVE
/** Start or stop physcis simulation */
#define PHYSICS         world_utils::msgs::WorldUtilsRequest::PHYSICS
/** Pause or resume simulation */
#define PAUSE           world_utils::msgs::WorldUtilsRequest::PAUSE
/** Get entity or world information */
#define STATUS          world_utils::msgs::WorldUtilsRequest::STATUS

/** Spawn sphere object */
#define SPHERE          world_utils::msgs::WorldUtilsRequest::SPHERE
/** Spawn cylinder object */
#define CYLINDER        world_utils::msgs::WorldUtilsRequest::CYLINDER
/** Spawn box object */
#define BOX             world_utils::msgs::WorldUtilsRequest::BOX
/** Spawn custom object */
#define CUSTOM          world_utils::msgs::WorldUtilsRequest::CUSTOM
/** Spawn custom light object */
#define CUSTOM_LIGHT    world_utils::msgs::WorldUtilsRequest::CUSTOM_LIGHT
/** Spawn a model included in gazebo model path */
#define MODEL           world_utils::msgs::WorldUtilsRequest::MODEL

/* Response */

/** Provide world state information */
#define INFO            world_utils::msgs::WorldUtilsResponse::INFO
/** Provide specific object state information */
#define PROPERTIES      world_utils::msgs::WorldUtilsResponse::PROPERTIES

/*
 * API Topics
 */

/** Topic monitored by the server for incoming camera requests */
#define CAMERA_UTILS_TOPIC          "~/gazebo-utils/camera_utils"
/** Topic for receiving replies from the camera plugin server  */
#define CAMERA_UTILS_RESPONSE_TOPIC "~/gazebo-utils/camera_utils/response"
/** Topic monitored by the server for incoming object spawn requests */
#define WORLD_UTILS_TOPIC           "~/gazebo-utils/world_utils"
/** Topic for receiving replies from the object spawner server */
#define WORLD_UTILS_RESPONSE_TOPIC  "~/gazebo-utils/world_utils/response"

/* Message pointer typedefs */

typedef const boost::shared_ptr<const world_utils::msgs::WorldUtilsResponse>
    WorldUtilsResponsePtr;
typedef const boost::shared_ptr<const camera_utils::msgs::CameraUtilsResponse>
    CameraUtilsResponsePtr;

/*
 * Function prototypes
 */

void spawnModelFromFile(
    gazebo::transport::PublisherPtr pub,
    const std::string model_path,
    const bool is_light,
    const bool use_custom_pose,
    const bool use_custom_textures,
    std::vector<std::string> textures = std::vector<std::string>(),
    const double & px = 0, 
    const double & py = 0,
    const double & pz = 0,
    const ignition::math::Quaternion<double> & orientation  = ignition::math::Quaternion<double>(0, M_PI/2.0, 0));


std::string spawnRandomObject(
    gazebo::transport::PublisherPtr pub,
    std::vector<std::string> textures,
    unsigned int & x_cell,
    unsigned int & y_cell,
    double & grid_cell_size);

void clearWorld(gazebo::transport::PublisherPtr pub);

void changePhysics(gazebo::transport::PublisherPtr pub, bool enable);

void pauseWorld(gazebo::transport::PublisherPtr pub, bool enable);

void captureScene(gazebo::transport::PublisherPtr pub, int idx);

bool waitForSpawner(int desired_objects);

void queryModelCount(gazebo::transport::PublisherPtr pub);

void queryModelBoundingBox(gazebo::transport::PublisherPtr pub,
    const std::string &model_name);

void query2DcameraPoint(
    gazebo::transport::PublisherPtr pub,
    const ignition::math::Vector3d &point);

void onWorldUtilsResponse(WorldUtilsResponsePtr &_msg);

bool waitForCamera();

void onCameraUtilsResponse(CameraUtilsResponsePtr &_msg);
