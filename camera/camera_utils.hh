/**
 * @file camera_plugin.hh 
 * @brief Camera plugin header
 * 
 * A custom gazebo plugin that provides an interface to 
 * programatically collect data from cameras at specific
 * times.
 *  
 * @author João Borrego
 */

/* Gazebo libraries */
#include "gazebo/gazebo.hh"
#include "gazebo/plugins/CameraPlugin.hh"

/* Custom messages */
#include "camera_utils_request.pb.h"

/** Topic monitored for incoming commands */
#define CAMERA_UTILS_TOPIC "~/gazebo-utils/camera_utils_plugin"

namespace gazebo{

    typedef const boost::shared_ptr<const camera_utils_msgs::msgs::CameraRequest>
        CameraRequestPtr;

    class CameraUtils : public CameraPlugin {

        /* Private attributes */
        private:

            /** A node used for transport */
            transport::NodePtr node;

            /** A subscriber to a named topic */
            transport::SubscriberPtr sub;

        /* Public methods */
        public:

            /**
             * @brief      Constructs the object
             */
            CameraUtils();

            /**
             * @brief      Loads the object
             *
             * @param[in]  _parent  The parent
             * @param[in]  _sdf     The sdf
             */
            void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

            void onMsg(CameraRequestPtr &_msg);

            /**
             * @brief      { function_description }
             *
             * @param[in]  _image   The image
             * @param[in]  _width   The width
             * @param[in]  _height  The height
             * @param[in]  _depth   The depth
             * @param[in]  _format  The format
             */
            void OnNewFrame(
                const unsigned char *_image,
                unsigned int _width,
                unsigned int _height,
                unsigned int _depth,
                const std::string &_format);
    };
}