#include "MoveObject.hh"

MoveObject::MoveObject(
    std::string & _name,
    bool _is_light,
    ignition::math::Pose3d & _pose) :
        name(_name), is_light(_is_light), pose(_pose)
{
}
