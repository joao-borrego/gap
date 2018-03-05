/// \file capture_example/object.hh
/// \brief Object class headers

/// WorldUtils request
#include "world_utils_request.pb.h"

/// Utilities
#include "utils.hh"

/// Gazebo
#include <gazebo/gazebo_client.hh>
/// OpenCV 2
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc.hpp"
/// Eigen
#include <Eigen/Dense>

/// Spawn sphere object
#define SPHERE    0
/// Spawn cylinder object
#define CYLINDER  1
/// Spawn box object
#define BOX       2

/// \class Object
/// TODO
class Object
{
    public:

        /// \brief Object type
        int type;
        /// \brief Object name
        std::string name;
        /// \brief Object 3D world pose
        ignition::math::Pose3d pose;
        /// \brief Object scale vector
        ignition::math::Vector3d scale;
        /// \brief Object parameter values
        std::vector<double> parameters;
        /// \brief Object surface 3D points
        std::vector<Eigen::Vector4f> points;
        /// \brief Object 2D bounding box
        std::vector<double> bounding_box;
        
    private:

        /// \brief Sample 3D points on object surface
        const double ANGLE_STEP_C = 45.0;
        /// \brief Sample 3D points on object surface
        const double TOTAL_STEPS_C = 360.0 / ANGLE_STEP_C;
        /// \brief Sample 3D points on object surface
        const double ANGLE_STEP_S = 30.0;
        /// \brief Sample 3D points on object surface
        const double TOTAL_STEPS_S = 360.0 / ANGLE_STEP_S;

    /// \brief Constructor
    /// \param TODO
    public: Object(
        int & _type,
        const std::string & _name,
        const ignition::math::Pose3d & _pose,
        const ignition::math::Vector3d & _scale,
        const std::vector<double> & _parameters
    );

    /// \brief Sample 3D points on object surface
    public: void sampleSurface();

};

/// \class Object 2D grid
class ObjectGrid
{
    public:

        /// Array of grid cells
        std::vector<int> cells;
        /// Size of x dimension
        float grid_x;
        /// Size of y dimension
        float grid_y;
        /// Number of cells in x dimension
        int num_cells_x;
        /// Number of cells in y dimension
        int num_cells_y;
        /// Size of each cell in x dimension
        float cell_x;
        /// Size of each cell in y dimension
        float cell_y;
        /// Height of each cell
        float cell_z;
        /// List of objects in grid
        std::vector<Object> objects;
        /// Array of counters, one per object type
        int counters[3] = {0};

        /// \brief Object types string vector
        const std::vector<std::string> TYPES = {"sphere", "cylinder","box"};

    /// \brief Constructor
    /// \param TODO
    /// \param TODO
    /// \param TODO
    /// \param TODO
    /// \param TODO
    public: ObjectGrid(
        int num_x,
        int num_y,
        float size_x,
        float size_y,
        float size_z);

    /// \brief Populates a grid with random objects
    /// \param num_objects Desired number of objects
    public: void populate(int num_objects);

    /// TODO
    private: void addRandomObject(int x, int y);

};
