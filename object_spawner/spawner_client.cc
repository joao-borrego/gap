#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

/**
 * Gazebo's API has changed between major releases. These changes are
 * accounted for with #if..#endif blocks in this file.
 */
#if GAZEBO_MAJOR_VERSION < 6
#include <gazebo/gazebo.hh>
#else
#include <gazebo/gazebo_client.hh>
#endif

/**
 * @brief      Client app for object spawner interface
 *
 * @param[in]  _argc  The argc
 * @param      _argv  The argv
 *
 * @return     0
 */
int main(int _argc, char **_argv)
{
    /* Load gazebo as a client */
    #if GAZEBO_MAJOR_VERSION < 6
    gazebo::setupClient(_argc, _argv);
    #else
    gazebo::client::setup(_argc, _argv);
    #endif

    /* Create the communication node */
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    /* Publish to the object spawner topic */
    gazebo::transport::PublisherPtr pub =
    node->Advertise<gazebo::msgs::Vector3d>("~/gazebo-utils/object_spawner");

    /* Wait for a subscriber to connect to this publisher */
    pub->WaitForConnection();

    /* Create a a vector3 message (placeholder message) */
    /* TODO - create custom messages */
    gazebo::msgs::Vector3d msg;

    #if GAZEBO_MAJOR_VERSION < 6
    gazebo::msgs::Set(&msg, gazebo::math::Vector3(0, 0, 0));
    #else
    gazebo::msgs::Set(&msg, ignition::math::Vector3d(0, 0, 0));
    #endif

    /* Send the message */
    pub->Publish(msg);

    /* Shut down */
    #if GAZEBO_MAJOR_VERSION < 6
    gazebo::shutdown();
    #else
    gazebo::client::shutdown();
    #endif

    return 0;
}