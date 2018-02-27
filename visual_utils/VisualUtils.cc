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
    /// \brief Connects to rendering update event
    public: event::ConnectionPtr updateConnection;

    /// Gazebo transport node 
    public: transport::NodePtr node;
    /// Visual utils topic subscriber 
    public: transport::SubscriberPtr sub;

    /// TODO
    public: bool flag;

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
    this->dataPtr->sub.reset();
    this->dataPtr->node->Fini();
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

    // Connect to the world update signal
    this->dataPtr->updateConnection = event::Events::ConnectPreRender(
        std::bind(&VisualUtils::Update, this));

     // Setup transport node 
    this->dataPtr->node = transport::NodePtr(new transport::Node());
    this->dataPtr->node->Init();
    // Subcribe to the monitored requests topic 
    this->dataPtr->sub = this->dataPtr->node->Subscribe(REQUEST_TOPIC,
        &VisualUtils::onRequest, this);

    // TEST
    this->dataPtr->flag = false;
}

/////////////////////////////////////////////////
void VisualUtils::Update()
{
    // TEST
    if (this->dataPtr->flag){
        this->dataPtr->visual->SetMaterial("Plugin/perlin_3");
    }
}

/////////////////////////////////////////////////
void VisualUtils::onRequest(VisualUtilsRequestPtr &_msg)
{
    // TEST
    this->dataPtr->flag = true;
}

}
