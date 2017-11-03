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

/* Strings */
#include <string>
/* Gazebo */
#include "gazebo/common/Plugin.hh"
#include "gazebo/sensors/CameraSensor.hh"
#include "gazebo/rendering/Camera.hh"
#include "gazebo/util/system.hh"
/* To create directories */
#include <boost/filesystem.hpp>

/* Custom messages */
#include "camera_utils_request.pb.h"
#include "camera_utils_reply.pb.h"

namespace CameraUtils {

/** Topic monitored for incoming commands */
#define REQUEST_TOPIC   "~/gazebo-utils/camera_utils_plugin"
/** Topic for replying to commands */
#define REPLY_TOPIC     "~/gazebo-utils/camera_utils_plugin/reply"

/** Request to capture a frame and save it to disk */
#define CAPTURE camera_utils_msgs::msgs::CameraRequest::CAPTURE

/* Default parameters */
#define DEFAULT_WORLD       (const std::string) "default"
#define DEFAULT_OUTPUT_DIR  (const std::string) "/tmp/camera_utils_output/"
#define DEFAULT_EXTENSION   (const std::string) ".png"

}

namespace gazebo{

    typedef const boost::shared_ptr<const camera_utils_msgs::msgs::CameraRequest>
        CameraRequestPtr;

    typedef const boost::shared_ptr<const camera_utils_msgs::msgs::CameraReply>
        CameraReplyPtr;

    // Forward declaration of private data class
    class CameraUtilsPrivate;

    class CameraUtils : public SensorPlugin {

        /* Private attributes */
        private:

            /** Class with private attributes */
            std::unique_ptr<CameraUtilsPrivate> dataPtr;
            /** Directory for saving output */
            std::string output_dir;
            /** Saved frames counter */
            int saved_counter = 0;            

        /* Protected attributes */
        protected:

            /** Pointer to camera sensor */
            sensors::CameraSensorPtr parentSensor;
            /** Pointer to camera boject */
            rendering::CameraPtr camera;
            /** Image dimensions */
            unsigned int width, height, depth;
            /** Image format */
            std::string format;
            /** Exported image extension */
            std::string extension;

        /* Public methods */
        public:

            /**
             * @brief      Constructs the object
             */
            CameraUtils();

            /**
             * @brief      Destroys the object.
             */
            virtual ~CameraUtils();

            /**
             * @brief      Loads the object
             *
             * @param[in]  _parent  The parent
             * @param[in]  _sdf     The sdf
             */
            virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);
        
        /* Private methods */
        private:

            /**
             * @brief      { function_description }
             *
             * @param      _msg  The message
             */
            void onMsg(CameraRequestPtr &_msg);
    };
}