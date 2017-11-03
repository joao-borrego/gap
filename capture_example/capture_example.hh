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
#include "camera_utils_reply.pb.h"
/* Object spawner request */
#include "object_spawner_request.pb.h"
/* Object spawner reply */
#include "object_spawner_reply.pb.h"

/*
 * Macros for custom messages
 */

/* Camera utils */

/** Request to capture a frame and save it to disk */
#define CAPTURE     camera_utils_msgs::msgs::CameraRequest::CAPTURE

/* Object Spawner */

/* Request */

/** Spawn object request */
#define SPAWN       object_spawner_msgs::msgs::SpawnRequest::SPAWN
/** Move object request */
#define MOVE        object_spawner_msgs::msgs::SpawnRequest::MOVE
/** Remove all entities from the world request */
#define CLEAR       object_spawner_msgs::msgs::SpawnRequest::CLEAR
/** Toggle physics simulation request */
#define TOGGLE      object_spawner_msgs::msgs::SpawnRequest::TOGGLE
/** Request world state information */
#define STATUS      object_spawner_msgs::msgs::SpawnRequest::STATUS
/** Spawn sphere object */
#define SPHERE      object_spawner_msgs::msgs::SpawnRequest::SPHERE
/** Spawn cylinder object */
#define CYLINDER    object_spawner_msgs::msgs::SpawnRequest::CYLINDER
/** Spawn box object */
#define BOX         object_spawner_msgs::msgs::SpawnRequest::BOX
/** Spawn custom object */
#define CUSTOM      object_spawner_msgs::msgs::SpawnRequest::CUSTOM
/** Spawn a model included in gazebo model path */
#define MODEL       object_spawner_msgs::msgs::SpawnRequest::MODEL
/** Spawn ground plane */
#define GROUND      object_spawner_msgs::msgs::SpawnRequest::GROUND

/* Reply */

/** Provid world state information */
#define INFO        object_spawner_msgs::msgs::Reply::INFO

/*
 * API Topics
 */

/** Topic monitored by the server for incoming camera requests */
#define CAMERA_UTILS_TOPIC          "~/gazebo-utils/camera_utils_plugin"
/** Topic for receiving replies from the camera plugin server  */
#define CAMERA_UTILS_REPLY_TOPIC    "~/gazebo-utils/camera_utils_plugin/reply"
/** Topic monitored by the server for incoming object spawn requests */
#define OBJECT_SPAWNER_TOPIC        "~/gazebo-utils/object_spawner"
/** Topic for receiving replies from the object spawner server */
#define OBJECT_SPAWNER_REPLY_TOPIC  "~/gazebo-utils/object_spawner/reply"

/* Message pointer typedefs */

typedef const boost::shared_ptr<const object_spawner_msgs::msgs::Reply>
    SpawnerReplyPtr;
typedef const boost::shared_ptr<const camera_utils_msgs::msgs::CameraReply>
    CameraReplyPtr;

/*
 * Function prototypes
 */

void spawnModelFromFile(
    gazebo::transport::PublisherPtr pub,
    const std::string model_path,
    const bool use_custom_pose,
    const bool use_custom_textures,
    std::vector<std::string> textures = std::vector<std::string>(),
    const double & px = 0, 
    const double & py = 0,
    const double & pz = 0,
    const ignition::math::Quaternion<double> & orientation  = ignition::math::Quaternion<double>());


void spawnRandomObject(
    gazebo::transport::PublisherPtr pub,
    std::vector<std::string> textures);

void clearWorld(gazebo::transport::PublisherPtr pub);

void disablePhysics(gazebo::transport::PublisherPtr pub);

void captureScene(gazebo::transport::PublisherPtr pub, int idx);

/* Handle object spawner asynchronous behaviour */

bool waitForSpawner(int desired_objects);

void queryModelCount(gazebo::transport::PublisherPtr pub);

void updateModelCount(SpawnerReplyPtr &_msg);

/* Wait for camera to save to file */

bool waitForCamera();

void updateCameraSuccess(CameraReplyPtr &_msg);