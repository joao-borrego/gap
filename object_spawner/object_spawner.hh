/**
 * @file
 * @brief
 * 
 * 
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

#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

/* Custom messages */
#include "object_spawner_request.pb.h"

#define OBJECT_SPAWNER_TOPIC "~/gazebo-utils/object_spawner"

namespace gazebo {

    typedef const boost::shared_ptr<const object_spawner_msgs::msgs::SpawnRequest>
        SpawnRequestPtr;

    class ObjectSpawnerPlugin : public WorldPlugin
    {
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

            /* DEBUG - aux counter */
            int sphere_counter = 0;
        
        /* Public methods */
        public:
            
            /**
             * @brief      Constructor
             */
            ObjectSpawnerPlugin();

            /**
             * @brief      Plugin setup executed on gazebo server launch
             *
             * @param[in]  _world  The world pointer
             * @param[in]  _sdf    The sdf parameters
             */
            void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

        /* Private methods */
        private:

            /**
             * @brief      { function_description }
             *
             * @param      _msg  The message
             */
            void onMsg(SpawnRequestPtr &_msg);

            /**
             * @brief      Prints live objects in the world
             */
            void printLiveObjs();

            /**
             * @brief      { function_description }
             *
             * @param[in]  model_name    The model name
             * @param[in]  radius        The radius
             * @param[in]  mass          The mass
             * @param[in]  px            { parameter_description }
             * @param[in]  py            { parameter_description }
             * @param[in]  pz            { parameter_description }
             * @param[in]  texture_uri   The texture uri
             * @param[in]  texture_name  The texture name
             */
            void spawnSphere(
                const std::string &model_name,
                const double radius,
                const double mass,
                const double px,
                const double py,
                const double pz,
                const std::string &texture_uri,
                const std::string &texture_name);
    };
}