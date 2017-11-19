/* Gazebo */
#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#if GAZEBO_MAJOR_VERSION < 6
#include <gazebo/gazebo.hh>
#else
#include <gazebo/gazebo_client.hh>
#endif

/* Program options */
#include<boost/program_options.hpp>
/* I/O streams */
#include <iostream>
/* File streams */
#include <fstream>
/* For iterating over the contents of a dir */
#include <boost/filesystem.hpp>
/* For protecting variables */
#include <mutex>
/* For sleeps */
#include <unistd.h>

/*
 * Custom messages
 */

/* Camera utils request */
#include "camera_utils_request.pb.h"
/* Camera utils reply */
#include "camera_utils_response.pb.h"
/* World utils request */
#include "world_utils_request.pb.h"
/* World utils response */
#include "world_utils_response.pb.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc.hpp"
/*
 * Macros for custom messages
 */

/* Camera utils */

/** Request to capture a frame and save it to disk */
#define CAMERA_INFO_REQUEST         camera_utils::msgs::CameraUtilsRequest::CAMERA_INFO
#define CAMERA_INFO_RESPONSE        camera_utils::msgs::CameraUtilsResponse::CAMERA_INFO

#define CAPTURE_REQUEST          camera_utils::msgs::CameraUtilsRequest::CAPTURE
#define CAPTURE_RESPONSE         camera_utils::msgs::CameraUtilsResponse::CAPTURE

#define CAMERA_POINT_REQUEST         camera_utils::msgs::CameraUtilsRequest::CAMERA_POINT
#define CAMERA_POINT_RESPONSE        camera_utils::msgs::CameraUtilsResponse::CAMERA_POINT
/* World utils */

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
#define SPHERE          world_utils::msgs::Object::SPHERE

/** Spawn cylinder object */
#define CYLINDER        world_utils::msgs::Object::CYLINDER

/** Spawn box object */
#define BOX             world_utils::msgs::Object::BOX

/** Spawn custom object */
#define CUSTOM          world_utils::msgs::Object::CUSTOM

/** Spawn custom light object */
#define CUSTOM_LIGHT    world_utils::msgs::Object::CUSTOM_LIGHT
/** Spawn a model included in gazebo model path */
#define MODEL           world_utils::msgs::Object::MODEL

/* Response */

/** Provide world state information */
#define INFO            world_utils::msgs::WorldUtilsResponse::INFO
/** Provide specific object state information */
#define PROPERTIES      world_utils::msgs::WorldUtilsResponse::PROPERTIES

/*
 * API Topics
 */

/** Topic monitored by the server for incoming camera requests */
#define CAMERA_UTILS_TOPIC          "~/gazebo-utils/camera_utils"
/** Topic for receiving replies from the camera plugin server  */
#define CAMERA_UTILS_RESPONSE_TOPIC "~/gazebo-utils/camera_utils/response"
/** Topic monitored by the server for incoming object spawn requests */
#define WORLD_UTILS_TOPIC           "~/gazebo-utils/world_utils"
/** Topic for receiving replies from the object spawner server */
#define WORLD_UTILS_RESPONSE_TOPIC  "~/gazebo-utils/world_utils/response"

/* Classes */
#define CYLINDER_ID       1
#define SPHERE_ID       0
#define BOX_ID       2
const std::map<int,std::string> classes_map{
   {CYLINDER_ID, "cylinder"},
   {SPHERE_ID, "sphere"},
   {BOX_ID, "box"}
};



class bounding_box_3d{
	public:
	bounding_box_3d(ignition::math::Vector3d & center_,ignition::math::Vector3d & size_) : center(center_), size(size_)
	{};
	ignition::math::Vector3d center;
	ignition::math::Vector3d size;
};


class Object {
	public:
	Object(std::string & _name, int & _type) : name(_name), type(_type)
	{};

	std::string name;
	int type;
	cv::Rect bounding_box;
};

class CameraInfo {
	public:
	CameraInfo(double _width, double _height, double _depth) : width(_width), height(_height), depth(_depth)
	{};
	double width, height, depth;

};


/* Message pointer typedefs */

typedef const boost::shared_ptr<const world_utils::msgs::WorldUtilsResponse>
    WorldUtilsResponsePtr;
typedef const boost::shared_ptr<const camera_utils::msgs::CameraUtilsResponse>
    CameraUtilsResponsePtr;


typedef std::multimap<std::string,bounding_box_3d> BoundingBox3d;
typedef std::multimap<std::string,ignition::math::Vector2d> BoundingBox2d;







/*
 * Function prototypes
 */

ignition::math::Pose3d getRandomCameraPose(const ignition::math::Vector3d & camera_position);

void spawnModelFromFile(
    world_utils::msgs::WorldUtilsRequest & msg,
    const std::string model_path,
    const bool is_light,
    const bool use_custom_pose,
    const bool use_custom_textures,
    std::vector<std::string> textures = std::vector<std::string>(),
    const ignition::math::Vector3d & position  = ignition::math::Vector3d(2.5, 2.5, 3.5),
    const ignition::math::Quaternion<double> & orientation  = ignition::math::Quaternion<double>(0, M_PI/2.0, 0),
    const std::string & name = std::string(""));


void spawnRandomObject(
    world_utils::msgs::WorldUtilsRequest & msg,
    std::vector<std::string> textures,
    double & grid_cell_size_x,
    double & grid_cell_size_y,
    int & num_objects,
    std::vector<Object> & objects);

void storeAnnotations(const std::vector<Object> & objects, const std::string & path, const std::string & file_name, const std::string & image_name);

void clearWorld(gazebo::transport::PublisherPtr pub,  std::vector<std::string> object_names = std::vector<std::string>());

void changePhysics(gazebo::transport::PublisherPtr pub, bool enable);

void pauseWorld(gazebo::transport::PublisherPtr pub, bool enable);

void captureScene(gazebo::transport::PublisherPtr pub, int idx);

bool waitForSpawner(int desired_objects);

void queryModelCount(gazebo::transport::PublisherPtr pub);

void queryModelBoundingBox(gazebo::transport::PublisherPtr pub,
    const std::vector<Object> & objects);

void query2DcameraPoint(
    gazebo::transport::PublisherPtr pub,
    const std::vector<Object> & objects);

void queryCameraParameters(gazebo::transport::PublisherPtr pub);

void onWorldUtilsResponse(WorldUtilsResponsePtr &_msg);

bool waitForCamera();

void onCameraUtilsResponse(CameraUtilsResponsePtr &_msg);
