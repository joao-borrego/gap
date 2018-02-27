/// \file visual_utils/VisualUtils.cc
/// \brief TODO
///
/// TODO
///
/// \author João Borrego

#include "VisualUtils.hh"

namespace gazebo {

    /// \class VisualUtils VisualUtils.hh
    /// \brief Private data for the VisualUtils class
    class VisualUtilsPrivate
    {
        /// \brief Visual to which the plugin is attached
        public: rendering::VisualPtr visual;
    };

    /// Register this plugin with the simulator 
    GZ_REGISTER_VISUAL_PLUGIN(VisualUtils)

    /////////////////////////////////////////////////
    VisualUtils::VisualUtils(): VisualPlugin(), dataPtr(new VisualUtilsPrivate)
    {
        std::cout << "[VisualUtils] Loaded visual tools." << std::endl;
    }

    /////////////////////////////////////////////////
    VisualUtils::~VisualUtils()
    {
        std::cout << "[VisualUtils] Unloaded visual tools." << std::endl;
    }

    /////////////////////////////////////////////////
    void VisualUtils::Load(rendering::VisualPtr _visual, sdf::ElementPtr _sdf)
    {
        // Check if attached to valid visual
        if (!_visual || !_sdf){
            gzerr << "[VisualUtils] Invalid visual or SDF element." << std::endl;
            return;
        }

        this->dataPtr->visual = _visual;
        this->dataPtr->visual->SetMaterial("Plugin/perlin_3");
    }
}
