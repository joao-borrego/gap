/// \file visual_utils/VisualUtils.hh
/// \brief TODO
///
/// TODO
///
/// \author João Borrego

// Gazebo 
#include "gazebo/common/Plugin.hh"
#include <gazebo/rendering/Visual.hh>
#include <gazebo/msgs/msgs.hh>

// Custom messages 

namespace VisualUtils {

// Default parameters

}

namespace gazebo{

    // Forward declaration of private data class
    class VisualUtilsPrivate;

    class VisualUtils : public VisualPlugin {

        /// \brief Constructs the object
        public: VisualUtils();

        /// \brief Destroys the object
        public: virtual ~VisualUtils();

        /// \brief Loads the plugin
        /// \param _visual  The visual to which the plugin is attached
        /// \param _sdf     The SDF element with plugin parameters
        public: virtual void Load(
            rendering::VisualPtr _visual,
            sdf::ElementPtr _sdf);

        /// \brief Private data pointer
        private: std::unique_ptr<VisualUtilsPrivate> dataPtr;
    };
}
