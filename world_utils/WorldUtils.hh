/// \brief \file world_utils/WorldUtils.hh
/// \brief TODO
/// \brief
/// \brief TODO
/// \brief
/// \brief \author João Borrego

// Gazebo
#include <gazebo/common/Events.hh>
#include "gazebo/common/Plugin.hh"
#include <gazebo/msgs/msgs.hh>
#include "gazebo/physics/physics.hh"
#include <gazebo/transport/transport.hh>

#include <mutex>

#include <iostream>
#include <iomanip>
#include <sstream>
#include <list>
#include <string>
#include <regex>

#include <boost/algorithm/string/predicate.hpp>
#include <boost/lexical_cast.hpp>

// Custom messages
#include "world_utils_request.pb.h"
#include "world_utils_response.pb.h"

namespace WorldUtils {

/// \brief Topic monitored for incoming commands
#define REQUEST_TOPIC "~/gazebo-utils/world_utils"
/// \brief Topic for publishing replies
#define RESPONSE_TOPIC "~/gazebo-utils/world_utils/response"

// Ease of use macros

// Request

/// \brief Spawn entity
#define SPAWN           world_utils::msgs::WorldUtilsRequest::SPAWN
/// \brief Move entity
#define MOVE            world_utils::msgs::WorldUtilsRequest::MOVE
/// \brief Remove entity from the world
#define REMOVE          world_utils::msgs::WorldUtilsRequest::REMOVE
/// \brief Start or stop physcis simulation
#define PHYSICS         world_utils::msgs::WorldUtilsRequest::PHYSICS
/// \brief Pause or resume simulation
#define PAUSE           world_utils::msgs::WorldUtilsRequest::PAUSE
/// \brief Get entity or world information
#define STATUS          world_utils::msgs::WorldUtilsRequest::STATUS

/// \brief Spawn sphere object
#define SPHERE          world_utils::msgs::Object::SPHERE
/// \brief Spawn cylinder object
#define CYLINDER        world_utils::msgs::Object::CYLINDER
/// \brief Spawn box object
#define BOX             world_utils::msgs::Object::BOX
/// \brief Spawn custom object
#define CUSTOM          world_utils::msgs::Object::CUSTOM
/// \brief Spawn custom light object
#define CUSTOM_LIGHT    world_utils::msgs::Object::CUSTOM_LIGHT
/// \brief Spawn a model included in gazebo model path
#define MODEL           world_utils::msgs::Object::MODEL

// Response

/// \brief Provide world state information
#define INFO            world_utils::msgs::WorldUtilsResponse::INFO
/// \brief TODO
#define SUCCESS         world_utils::msgs::WorldUtilsResponse::SUCCESS

// Regex patterns

/// \brief Matches string enclosed in <script> XML tags
#define REGEX_XML_SCRIPT "<script>[\\s\\S]*?<\\/script>"
/// \brief Matches string enclosed in <pose> XML tags
#define REGEX_XML_POSE   "<pose>[\\s\\S]*?<\\/pose>"

}

namespace gazebo {

    typedef const boost::shared_ptr<const world_utils::msgs::WorldUtilsRequest>
        WorldUtilsRequestPtr;

    typedef const boost::shared_ptr<const world_utils::msgs::WorldUtilsResponse>
        WorldUtilsResponsePtr;

    /// \brief TODO
    class WorldUtils : public WorldPlugin {

        /// \brief TODO
        public: std::mutex mutex;

        /// \brief A pointer to the world
        private: physics::WorldPtr world;
        /// \brief Connection to World Update events
        private: event::ConnectionPtr updateConnection;

        /// \brief A node used for transport
        private: transport::NodePtr node;
        /// \brief A subscriber to the request topic
        private: transport::SubscriberPtr sub;
        /// \brief A publisher to the reply topic
        private: transport::PublisherPtr pub;

        /// \brief A publisher to the gazebo request topic
        private: transport::PublisherPtr request_pub;
        /// \brief A subscriber to the gazebo response topic
        private: transport::SubscriberPtr response_sub;

        // Regex patterns
        private: std::regex script_reg;
        private: std::regex pose_reg;

        // Counters for automatic naming
        private: int sphere_counter      {0};
        private: int cylinder_counter    {0};
        private: int box_counter         {0};
        private: int light_counter       {0};

        /// \brief TODO
        private: std::queue<int> type;
        /// \brief Queue of objects with pending move operations
        private: std::queue<std::string> move;
        /// \brief TODO
        private: std::queue<ignition::math::Pose3d> poses;

        // Public methods
        public:
            
            /// \brief TODO
            WorldUtils();

            /// \brief TODO
            void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

            /// \brief TODO
            void onUpdate();

        // Private methods
        private:

            /// \brief TODO
            void onRequest(WorldUtilsRequestPtr &_msg);

            /// \brief TODO
            void printLiveObjs();

            /// \brief TODO
            void clearWorld();

            /// \brief TODO
            void clearMatching(const std::string &match, const bool is_light);

            /// \brief TODO
            const std::string genSphere(
                const std::string &model_name,
                const double mass,
                const double radius,
                const ignition::math::Vector3d position,
                const ignition::math::Quaterniond orientation);

            /// \brief TODO
            const std::string genCylinder(
                const std::string &model_name,
                const double mass,
                const double radius,
                const double length,
                const ignition::math::Vector3d position,
                const ignition::math::Quaterniond orientation);

            /// \brief TODO
            const std::string genBox(
                const std::string &model_name,
                const double mass,
                const ignition::math::Vector3d size,
                const ignition::math::Vector3d position,
                const ignition::math::Quaterniond orientation);
    };
}