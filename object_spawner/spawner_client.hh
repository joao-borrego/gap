/* Gazebo */
#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

/* I/O streams */
#include <iostream>
/* File streams */
#include <fstream>
/* Regular expressions */
#include <regex>
/* For iterating over the contents of a dir */
#include <boost/filesystem.hpp>

/* Custom messages */
#include "object_spawner_request.pb.h"

/**
 * Gazebo's API has changed between major releases. These changes are
 * accounted for with #if..#endif blocks in this file.
 */
#if GAZEBO_MAJOR_VERSION < 6
#include <gazebo/gazebo.hh>
#else
#include <gazebo/gazebo_client.hh>
#endif

/* @brief Command prompt */
#define PROMPT "> "

#define SPAWN object_spawner_msgs::msgs::SpawnRequest::SPAWN
#define REMOVE object_spawner_msgs::msgs::SpawnRequest::REMOVE
#define SPHERE object_spawner_msgs::msgs::SpawnRequest::SPHERE
#define CYLINDER object_spawner_msgs::msgs::SpawnRequest::CYLINDER
#define BOX object_spawner_msgs::msgs::SpawnRequest::BOX

#define OBJECT_SPAWNER_TOPIC "~/gazebo-utils/object_spawner"

#define MODEL_NAME "model"
#define MODEL_DIR "../../media/" MODEL_NAME
#define TEXTURES_DIR MODEL_DIR "/materials/textures/"
#define SCRIPT_DIR MODEL_DIR "/materials/scripts/"
#define SCRIPT_FILE SCRIPT_DIR MODEL_NAME ".material"

#define TEXTURE_URI "model://model/materials/scripts</uri>\
                    <uri>model://model/materials/textures"
