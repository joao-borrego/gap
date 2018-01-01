/* Gazebo */
#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#if GAZEBO_MAJOR_VERSION < 6
#include <gazebo/gazebo.hh>
#else
#include <gazebo/gazebo_client.hh>
#endif

/* Utilities */
#include "utils.hh"

/* I/O streams */
#include <iostream>
/* File streams */
#include <fstream>
/* For iterating over the contents of a dir */
#include <boost/filesystem.hpp>
/* For protecting variables */
#include <mutex>
/* For sleeps */
#include <chrono>
#include <thread>
/* TODO */
#include <Eigen/Dense>

/* Custom messages */

/* Camera utils request */
#include "camera_utils_request.pb.h"
/* Camera utils reply */
#include "camera_utils_response.pb.h"
/* World utils request */
#include "world_utils_request.pb.h"
/* World utils response */
#include "world_utils_response.pb.h"

/* OpenCV 2 */
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc.hpp"

/*
 * Macros for custom messages
 */

/* Camera utils */

/** Request for camera object info */
#define CAMERA_INFO_REQUEST     camera_utils::msgs::CameraUtilsRequest::CAMERA_INFO
/** Request to capture a frame and save it to disk */
#define CAPTURE_REQUEST         camera_utils::msgs::CameraUtilsRequest::CAPTURE
/** Request for projection of 3D point in world to 2D camera reference plane */
#define CAMERA_POINT_REQUEST    camera_utils::msgs::CameraUtilsRequest::CAMERA_POINT

// TODO
#define CAMERA_INFO_RESPONSE    camera_utils::msgs::CameraUtilsResponse::CAMERA_INFO
#define CAPTURE_RESPONSE        camera_utils::msgs::CameraUtilsResponse::CAPTURE
#define CAMERA_POINT_RESPONSE   camera_utils::msgs::CameraUtilsResponse::CAMERA_POINT

/* World utils */

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

typedef enum {sphere = 0, cylinder, box} ObjectType;

const world_utils::msgs::Object::ModelType classes_id[3] = {SPHERE, CYLINDER, BOX};
const char classes_name[3][100] = {"sphere", "cylinder", "box"};
const char class_instance_names[3][100] =
    {"plugin_sphere_", "plugin_cylinder_", "plugin_box_"};
unsigned int class_instance_counters[3] = {0};

const double ANGLE_STEP=10.0;
const double TOTAL_STEPS=360.0/ANGLE_STEP;
class BoundingBox3dClass{

    public:

        BoundingBox3dClass(
            ignition::math::Vector3d & center_,
            ignition::math::Vector3d & size_
        ) : center(center_), size(size_){};

        ignition::math::Vector3d center;
        ignition::math::Vector3d size;
};

class Object {

    public:

        Object(
            std::string & _name,
            int & _type,
            ignition::math::Pose3d _pose,
	    const std::vector<double> & _parameters
        ) : name(_name), type(_type), pose(_pose), parameters(_parameters){
		// Generate surface points from object parameters

		if(type==cylinder)
		{
			double radius=fabs(parameters[0]);
			for(int a=0; a<TOTAL_STEPS;++a)
			{
				float x,y,z;
				x=cos(ANGLE_STEP*a)*radius;
				y=sin(ANGLE_STEP*a)*radius;

				z=0.5;
				ignition::math::Vector3d point_bottom(x,y,z);
				object_points.push_back(point_bottom);
				z=-0.5;
				ignition::math::Vector3d point_top(x,y,z);
				object_points.push_back(point_top);
			}
		}
		else if(type==sphere)
		{
			double radius=fabs(parameters[0]);
			for(int a=0; a<TOTAL_STEPS;++a)
			{
				for(int b=0; b<TOTAL_STEPS;++b)
				{
					float x,y,z;
					x=cos(ANGLE_STEP*a)*radius;
					y=sin(ANGLE_STEP*a)*radius;
				}
			}
		}
		else if(type==box)
		{
			std::cout << "box" << std::endl;
		}
		
	};

        std::string name;
        int type;
        cv::Rect bounding_box;
        ignition::math::Pose3d pose;
	std::vector<double> parameters;
	std::vector<ignition::math::Vector3d> object_points;
};

class CameraInfo {
    
    public:
        CameraInfo(
            double _width,
            double _height,
            double _depth
        ) : width(_width), height(_height), depth(_depth){};
    
        double width, height, depth;
};


// Message pointer tyoedefs

typedef const boost::shared_ptr<const world_utils::msgs::WorldUtilsResponse>
    WorldUtilsResponsePtr;
typedef const boost::shared_ptr<const camera_utils::msgs::CameraUtilsResponse>
    CameraUtilsResponsePtr;

// Bounding box maps

typedef std::multimap<std::string,BoundingBox3dClass> BoundingBox3d;
typedef std::multimap<std::string,ignition::math::Vector2d> BoundingBox2d;

/*
 * Function prototypes
 */

ignition::math::Pose3d getRandomCameraPose(
    const ignition::math::Vector3d & camera_position);

void addModelToMsg(
    world_utils::msgs::WorldUtilsRequest & msg,
    std::vector<Object> & objects,
    const std::string & model_path,
    const bool is_light,
    const bool is_random,
    const bool use_custom_textures,
    const int cell_x,
    const int cell_y,
    std::vector<std::string> & textures);

void genRandomObjectInGrid(
    std::vector<Object> & objects,
    world_utils::msgs::Object *object,
    const int cell_x,
    const int cell_y);

void storeAnnotations(
    const std::vector<Object> & objects,
    const ignition::math::Pose3d & camera_pose,
    const std::string & path,
    const std::string & file_name,
    const std::string & image_name);

void moveObject(
    gazebo::transport::PublisherPtr pub,
    const std::string &name,
    const ignition::math::Pose3d &pose);

void clearWorld(
    gazebo::transport::PublisherPtr pub,
    std::vector<std::string> &object_names);

void changePhysics(gazebo::transport::PublisherPtr pub, bool enable);

void pauseWorld(gazebo::transport::PublisherPtr pub, bool enable);

void captureScene(gazebo::transport::PublisherPtr pub, int idx);

bool waitForSpawner(int desired_objects);

bool waitForBoundingBox(int desired_objects);

void queryModelCount(gazebo::transport::PublisherPtr pub);

void queryModelBoundingBox(
    gazebo::transport::PublisherPtr pub,
    const std::vector<Object> & objects);

void query2DcameraPoint(
    gazebo::transport::PublisherPtr pub,
    const std::vector<Object> & objects);

bool waitFor2DPoints(int desired_points);

void obtain2DBoundingBoxes(
    std::vector<Object> & objects,
    std::vector<cv::Rect> & bounding_rectangles);

void queryCameraParameters(gazebo::transport::PublisherPtr pub);

void onWorldUtilsResponse(WorldUtilsResponsePtr & _msg);

bool waitForCamera();

void onCameraUtilsResponse(CameraUtilsResponsePtr & _msg);

void visualizeData(
    const std::string & image_dir,
    const std::string & image_name,
    int num_objects,
    BoundingBox2d & points_2d,
    std::vector<cv::Rect> & bound_rect);
