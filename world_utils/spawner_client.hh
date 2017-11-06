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
#include "world_utils_request.pb.h"

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

#define WORLD_UTILS_TOPIC "~/gazebo-utils/world_utils"

#define MEDIA_DIR "media/"
#define TEXTURES_DIR MEDIA_DIR "materials/textures/"
#define SCRIPTS_DIR MEDIA_DIR "materials/scripts/"

#define TEXTURE_URI "file://materials/scripts/plugin.material"
