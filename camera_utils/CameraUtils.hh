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

/* Benchmark timer */
#include <ctime>

/* Custom messages */
#include "camera_utils_request.pb.h"
#include "camera_utils_response.pb.h"

namespace CameraUtils {

/** Topic monitored for incoming commands */
#define REQUEST_TOPIC   "~/gazebo-utils/camera_utils"
/** Topic for replying to commands */
#define RESPONSE_TOPIC  "~/gazebo-utils/camera_utils/response"

/** Request to capture a frame and save it to disk */
#define CAMERA_INFO_REQUEST     camera_utils::msgs::CameraUtilsRequest::CAMERA_INFO
#define CAMERA_INFO_RESPONSE    camera_utils::msgs::CameraUtilsResponse::CAMERA_INFO

#define CAPTURE_REQUEST         camera_utils::msgs::CameraUtilsRequest::CAPTURE
#define CAPTURE_RESPONSE        camera_utils::msgs::CameraUtilsResponse::CAPTURE

#define CAMERA_POINT_REQUEST    camera_utils::msgs::CameraUtilsRequest::CAMERA_POINT
#define CAMERA_POINT_RESPONSE   camera_utils::msgs::CameraUtilsResponse::CAMERA_POINT

/* Default parameters */
#define DEFAULT_OUTPUT_DIR  (const std::string) "/tmp/camera_utils_output/"
#define DEFAULT_EXTENSION   (const std::string) ".png"

}

namespace gazebo{

    typedef const boost::shared_ptr<const camera_utils::msgs::CameraUtilsRequest>
        CameraUtilsRequestPtr;

    typedef const boost::shared_ptr<const camera_utils::msgs::CameraUtilsResponse>
        CameraUtilsReplyPtr;

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
            /** File name for next capture */
            std::string next_file_name;
            /** Internal flag for saving on next update */
            bool save_on_update = false;

            event::ConnectionPtr newFrameConnection;

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

            /**
             * @brief      Callback function for handling frame updates
             *
             * @param[in]  _image   The image
             * @param[in]  _width   The width
             * @param[in]  _height  The height
             * @param[in]  _depth   The depth
             * @param[in]  _format  The format
             */
            void OnNewFrame(const unsigned char *_image,
                unsigned int _width, unsigned int _height,
                unsigned int _depth, const std::string &_format);
        
        /* Private methods */
        private:

            /**
             * @brief      Callback function for handling incoming requests
             *
             * @param      _msg  The message
             */
            void onRequest(CameraUtilsRequestPtr &_msg);
    };
}
