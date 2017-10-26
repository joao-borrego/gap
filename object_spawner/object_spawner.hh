/**
 * @file object_spawner.hh 
 * @brief Object spawner headers
 *  
 * @author João Borrego
 */

#include <iostream>         // io
#include <iomanip>          // setprecision
#include <sstream>          // stringstream
#include <list>             // list of live objects 
#include <string>           // strings
#include <regex>            // regular expressions
#include <gazebo/gazebo.hh> // gazebo

#include <boost/algorithm/string/predicate.hpp>
#include <boost/lexical_cast.hpp>

#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

/* Custom messages */
#include "object_spawner_request.pb.h"

/** @brief Topic monitored for incoming commands */
#define OBJECT_SPAWNER_TOPIC "~/gazebo-utils/object_spawner"

/* Ease of use macros */

/** @brief Spawn object request */
#define SPAWN       object_spawner_msgs::msgs::SpawnRequest::SPAWN
/** @brief Move object request */
#define MOVE        object_spawner_msgs::msgs::SpawnRequest::MOVE
/** @brief Remove all entities from the world request */
#define CLEAR       object_spawner_msgs::msgs::SpawnRequest::CLEAR
/** @brief Toggle physics simulation request */
#define TOGGLE      object_spawner_msgs::msgs::SpawnRequest::TOGGLE
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

namespace gazebo {

    typedef const boost::shared_ptr<const object_spawner_msgs::msgs::SpawnRequest>
        SpawnRequestPtr;

    class ObjectSpawnerPlugin : public WorldPlugin {

        /* Private attributes */
        private:

            /* @brief A pointer to the world */
            physics::WorldPtr world;
            /* @brief Keep track of live objects */
            std::list<std::string> live_objs;
            /* @brief A node used for transport */
            transport::NodePtr node;
            /* @brief A subscriber to a named topic */
            transport::SubscriberPtr sub;
            /* @brief A publisher to the factory topic */
            transport::PublisherPtr factory_pub;

            std::regex script_reg;

            /* Counters for automatic naming */
            int sphere_counter = 0;
            int cylinder_counter = 0;
            int box_counter = 0;
        
        /* Public methods */
        public:
            
            /**
             * @brief      Constructor
             */
            ObjectSpawnerPlugin();

            /**
             * @brief      Plugin setup executed on gazebo server launch
             *
             * @param      _world  The world pointer
             * @param      _sdf    The sdf parameters
             */
            void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

        /* Private methods */
        private:

            /**
             * @brief      Callback function for receiving a message
             *
             * @param      _msg  The message
             */
            void onMsg(SpawnRequestPtr &_msg);

            /**
             * @brief      Prints live objects in the world
             */
            void printLiveObjs();

            /**
             * @brief      Deleters every entity in the world
             */
            void clearWorld();

            /**
             * @brief      Generates SDF string for sphere object
             *
             * @param      model_name   The model name
             * @param      mass         The mass
             * @param      radius       The radius
             * @param      position     The position
             * @param      orientation  The orientation
             *
             * @return     The sphere SDF string
             */
            const std::string genSphere(
                const std::string &model_name,
                const double mass,
                const double radius,
                const ignition::math::Vector3d position,
                const ignition::math::Quaterniond orientation);

            /**
             * @brief      Generates SDF string for cylinder object
             *
             * @param      model_name   The model name
             * @param      mass         The mass
             * @param      radius       The radius
             * @param      length       The length
             * @param      position     The position
             * @param      orientation  The orientation
             *
             * @return     The cylinder SDF string
             */
            const std::string genCylinder(
                const std::string &model_name,
                const double mass,
                const double radius,
                const double length,
                const ignition::math::Vector3d position,
                const ignition::math::Quaterniond orientation);

            /**
             * @brief      Generates SDF string for box object
             *
             * @param      model_name   The model name
             * @param      mass         The mass
             * @param      size         The box dimensions
             * @param      position     The position
             * @param      orientation  The orientation
             *
             * @return     The box SDF string
             */
            const std::string genBox(
                const std::string &model_name,
                const double mass,
                const ignition::math::Vector3d size,
                const ignition::math::Vector3d position,
                const ignition::math::Quaterniond orientation);
    };
}