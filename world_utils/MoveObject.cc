/*!
    \file world_utils/MoveObject.cc
    \brief Move Object class implementation

    Class for object with a pending move operation

    \author João Borrego : jsbruglie
    \author Rui Figueiredo : ruipimentelfigueiredo
*/

#include "MoveObject.hh"

MoveObject::MoveObject(
    std::string & _name,
    bool _is_light,
    ignition::math::Pose3d & _pose) :
        name(_name), is_light(_is_light), pose(_pose)
{
}
