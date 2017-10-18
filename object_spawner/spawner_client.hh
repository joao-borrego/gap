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

/** @brief Spawn object request */
#define SPAWN       object_spawner_msgs::msgs::SpawnRequest::SPAWN
/** @brief Move object request */
#define MOVE        object_spawner_msgs::msgs::SpawnRequest::MOVE
/** @brief Remove all entities from the world request */
#define CLEAR       object_spawner_msgs::msgs::SpawnRequest::CLEAR
/** @brief Spawn sphere object */
#define SPHERE      object_spawner_msgs::msgs::SpawnRequest::SPHERE
/** @brief Spawn cylinder object */
#define CYLINDER    object_spawner_msgs::msgs::SpawnRequest::CYLINDER
/** @brief Spawn box object */
#define BOX         object_spawner_msgs::msgs::SpawnRequest::BOX
/** @brief Spawn custom object */
#define CUSTOM      object_spawner_msgs::msgs::SpawnRequest::CUSTOM
/* @brief Spawn a model included in gazebo model path */
#define MODEL      object_spawner_msgs::msgs::SpawnRequest::MODEL
/* @brief Spawn ground plane */
#define GROUND      object_spawner_msgs::msgs::SpawnRequest::GROUND

#define OBJECT_SPAWNER_TOPIC "~/gazebo-utils/object_spawner"

#define MEDIA_DIR "../../media/"
#define TEXTURES_DIR MEDIA_DIR "materials/textures/"
#define SCRIPTS_DIR MEDIA_DIR "materials/scripts/"

#define TEXTURE_URI "file://materials/scripts/plugin.material"
