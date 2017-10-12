#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

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