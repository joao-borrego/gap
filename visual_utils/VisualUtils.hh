/*!
    \file visual_utils/VisualUtils.hh
    \brief Visual Utils plugin

    A custom gazebo plugin that provides an interface to programatically
    change the visual properties of an object.

    \author João Borrego : jsbruglie
    \author Rui Figueiredo : ruipimentelfigueiredo
*/

// Gazebo
#include <gazebo/common/Events.hh>
#include "gazebo/common/Plugin.hh"
#include <gazebo/msgs/msgs.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/transport/Node.hh>

// Boost - for convenient string split
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
// Mutex
#include <mutex>
// TODO - Change to decent RNG
#include <time.h>
#include <stdlib.h>

// Custom messages
#include "visual_utils_request.pb.h"

namespace VisualUtils {

/// Topic monitored for incoming commands
#define REQUEST_TOPIC   "~/gazebo-utils/visual_utils"

/// Request update
#define UPDATE          visual_utils::msgs::VisualUtilsRequest::UPDATE
/// Set default pose
#define DEFAULT_POSE    visual_utils::msgs::VisualUtilsRequest::DEFAULT_POSE

// Default parameters

/// Default unique name
#define DEFAULT_NAME    "default"

}

namespace gazebo{

    /// Shared pointer declaration for request message type
    typedef const boost::shared_ptr<const visual_utils::msgs::VisualUtilsRequest>
        VisualUtilsRequestPtr;

    // Forward declaration of private data class
    class VisualUtilsPrivate;

    /// \brief A custom gazebo plugin that provides an interface to programatically
    /// alter visuals during simulation.
    ///
    /// Materials are assumed to be loaded and name <pattern><index>
    /// See the example usage below:
    ///
    /// \code{.xml}
    ///    <plugin name="visual_utils" filename="libVisualUtils.so">
    ///     <!-- Unique name identifier -->
    ///     <uid>box_1</uid>
    ///     <!-- Prefix patterns for material names, separated by whitespace -->
    ///     <patterns>Plugin/flat_ Plugin/gradient_ ... </patterns>
    ///     <!-- Number of variants per prefix pattern -->
    ///     <variants>100</variants>
    ///    </plugin>
    /// \endcode
    ///
    /// See worlds/visual.world for a complete example.
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

        /// \brief Update once per simulation iteration.
        public: void Update();

        /// \brief Callback function for handling incoming requests
        /// \param _msg  The message
        public: void onRequest(VisualUtilsRequestPtr & _msg);

        /// \brief Randomly generates a new material name.
        /// \param name Output random material name
        public: void randomMaterialName(std::string & name);

        /// \brief Private data pointer
        private: std::unique_ptr<VisualUtilsPrivate> dataPtr;
    };
}
