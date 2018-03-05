#include <string>

// Gazebo
#include "gazebo/common/Plugin.hh"

class MoveObject
{
    /// \brief TODO
    public: std::string name;
    /// \brief TODO
    public: bool is_light;
    /// \brief TODO
    public: ignition::math::Pose3d pose;

    /// \brief Constructor
    public: MoveObject(
        std::string & _name,
        bool _is_light,
        ignition::math::Pose3d & _pose);
};
