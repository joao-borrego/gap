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

/*
 * Custom messages
 */

/* Camera utils request */
#include "camera_utils_request.pb.h"
/* Object spawner request */
#include "object_spawner_request.pb.h"

/*
 * Macros for custom messages
 */

/* Camera utils */

/** Request to capture a frame and save it to disk */
#define CAPTURE 	camera_utils_msgs::msgs::CameraRequest::CAPTURE

/* Object Spawner */

/** Spawn object request */
#define SPAWN       object_spawner_msgs::msgs::SpawnRequest::SPAWN
/** Move object request */
#define MOVE        object_spawner_msgs::msgs::SpawnRequest::MOVE
/** Remove all entities from the world request */
#define CLEAR       object_spawner_msgs::msgs::SpawnRequest::CLEAR
/** @brief Toggle physics simulation request */
#define TOGGLE      object_spawner_msgs::msgs::SpawnRequest::TOGGLE
/** Spawn sphere object */
#define SPHERE      object_spawner_msgs::msgs::SpawnRequest::SPHERE
/** Spawn cylinder object */
#define CYLINDER    object_spawner_msgs::msgs::SpawnRequest::CYLINDER
/** Spawn box object */
#define BOX         object_spawner_msgs::msgs::SpawnRequest::BOX
/** Spawn custom object */
#define CUSTOM      object_spawner_msgs::msgs::SpawnRequest::CUSTOM
/** Spawn a model included in gazebo model path */
#define MODEL      object_spawner_msgs::msgs::SpawnRequest::MODEL
/** Spawn ground plane */
#define GROUND      object_spawner_msgs::msgs::SpawnRequest::GROUND

/*
 * API Topics
 */

/** Topic monitored by the server for incoming camera requests */
#define CAMERA_UTILS_TOPIC "~/gazebo-utils/camera_utils_plugin"
/** Topic monitored by the server for incoming object spawn requests */
#define OBJECT_SPAWNER_TOPIC "~/gazebo-utils/object_spawner"

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
    std::vector<std::string> textures,
    unsigned int & x_cell,
    unsigned int & y_cell,
    double & grid_cell_size);

void clearWorld(gazebo::transport::PublisherPtr pub);

void togglePhysics(gazebo::transport::PublisherPtr pub);

void captureScene(gazebo::transport::PublisherPtr pub);
