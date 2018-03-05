/*!
    \file world_utils/MoveObject.hh
    \brief Move Object class

    Class for object with a pending move operation

    \author João Borrego : jsbruglie
    \author Rui Figueiredo : ruipimentelfigueiredo
*/

// Gazebo
#include "gazebo/common/Plugin.hh"

#include <string>

/// \class Object with a pending move operation
class MoveObject
{
    /// \brief Object name
    public: std::string name;
    /// \brief Whether object is light
    public: bool is_light;
    /// \brief Object new pose
    public: ignition::math::Pose3d pose;

    /// \brief Constructs the object
    /// \param _name        Object name
    /// \param _is_light    Whether the object is a light
    /// \param _pose        Object pose
    public: MoveObject(
        std::string & _name,
        bool _is_light,
        ignition::math::Pose3d & _pose);
};
