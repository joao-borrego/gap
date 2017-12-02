/**
 * @file WorldUtils.hh 
 * @brief World utils headers
 * 
 * A gazebo WorldPlugin that provides an interface to programatically interact with the simulator
 * and perform a set of actions including:
 * - Spawning entities
 *  + generic shapes from respective parameters
 *  + from SDF strings, namely light objects
 *  + from model URI
 *  + with custom textures in a known media directory
 * - Moving and removing entities
 * - Get world information
 * - Get specific object information, namely
 *  + 3D bounding box
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
#include "world_utils_request.pb.h"
#include "world_utils_response.pb.h"

namespace WorldUtils {

/** Topic monitored for incoming commands */
#define REQUEST_TOPIC "~/gazebo-utils/world_utils"
/** Topic for publishing replies */
#define RESPONSE_TOPIC "~/gazebo-utils/world_utils/response"

/* Ease of use macros */

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

/* Regex patterns */

/** Matches string enclosed in <script> XML tags */
#define REGEX_XML_SCRIPT "<script>[\\s\\S]*?<\\/script>"
/** Matches string enclosed in <pose> XML tags */
#define REGEX_XML_POSE   "<pose>[\\s\\S]*?<\\/pose>"

}

namespace gazebo {

    typedef const boost::shared_ptr<const world_utils::msgs::WorldUtilsRequest>
        WorldUtilsRequestPtr;

    typedef const boost::shared_ptr<const world_utils::msgs::WorldUtilsResponse>
        WorldUtilsResponsePtr;

    class WorldUtils : public WorldPlugin {

        /* Private attributes */
        private:

            /** A pointer to the world */
            physics::WorldPtr world;
            /** Keep track of live objects */
            std::list<std::string> live_objs;
            
            /** A node used for transport */
            transport::NodePtr node;
            /** A subscriber to the request topic */
            transport::SubscriberPtr sub;
            /** A publisher to the reply topic */
            transport::PublisherPtr pub;
            
            /** A publisher to the factory topic */
            transport::PublisherPtr factory_pub;
            /** A publisher to the light factory topic */
            transport::PublisherPtr factory_light_pub;
            /** A publisher to the gazebo request topic */
            transport::PublisherPtr request_pub;
            /** A subscriber to the gazebo response topic */
            transport::SubscriberPtr response_sub;

            /* Regex patterns */
            std::regex script_reg;
            std::regex pose_reg;

            /* Counters for automatic naming */
            int sphere_counter      {0};
            int cylinder_counter    {0};
            int box_counter         {0};
            int light_counter       {0};

        /* Public methods */
        public:
            
            /**
             * @brief      Constructor
             */
            WorldUtils();

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
            void onRequest(WorldUtilsRequestPtr &_msg);

            /**
             * @brief      Prints live objects in the world
             */
            void printLiveObjs();

            /**
             * @brief      Deletes every model in the world
             */
            void clearWorld();

            /**
             * @brief      Deletes every model with name matching a given substring
             *
             * @param      match  The substring to be matched
             */
            void clearMatching(const std::string &match, const bool is_light);

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