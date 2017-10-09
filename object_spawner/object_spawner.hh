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
#include <gazebo/gazebo.hh> // gazebo

#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#define OBJECT_SPAWNER_TOPIC "~/gazebo-utils/object_spawner"

namespace gazebo {

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
             * @brief      Callback function for when messages are received
             * 
             * @param      _msg  The message
             */
            void onMsg(ConstVector3dPtr &_msg);

            /**
             * @brief      Prints live objects in the world
             */
            void printLiveObjs();

            /**
             * @brief      Spawns a sphere
             *
             * @param[in]  model_name  The model name
             * @param[in]  radius      The radius
             * @param[in]  px          Position in x
             * @param[in]  py          Position in y
             * @param[in]  pz          Position in z
             */
            void spawnSphere(
                const std::string &model_name,
                double radius,
                double px,
                double py,
                double pz);
    };
}