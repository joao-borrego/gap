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

    /// Material name patterns
    public: std::vector<std::string> patterns;
    /// Number of material type variants 
    public: int variants; 

    /// TODO
    public: bool update;
    /// TODO
    public: int counter; 

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

    // Update flag
    this->dataPtr->update = false;
    // Reset counter
    this->dataPtr->counter = 0;

    // Possible patterns for material names
    if (_sdf->HasElement("patterns")){
        std::string patterns_arg(_sdf->Get<std::string>("patterns"));
        boost::split(this->dataPtr->patterns, patterns_arg,
            boost::is_any_of(" "), boost::token_compress_on);
    } else {
        // TODO
    }
    
    // Number of possible variants for each pattern
    this->dataPtr->variants = 0;
    if (_sdf->HasElement("variants")){
        this->dataPtr->variants = _sdf->Get<int>("variants");
    }

    // Geometry type

    // TODO - Change to decent RNG
    srand(time(NULL));
}

/////////////////////////////////////////////////
void VisualUtils::Update()
{
    // TEST
    if (this->dataPtr->update){
        
        // Change material
        std::string material;
        this->randomMaterialName(material);
        this->dataPtr->visual->SetMaterial(material);
        this->dataPtr->update = false;
        // Change pose
        int x = this->dataPtr->visual->Pose().Pos().X();
        int z = (this->dataPtr->counter++ % 2);
        ignition::math::Pose3d pose(x,0,z,0,0,0);
        this->dataPtr->visual->SetPose(pose);
        // Change scale
        ignition::math::Vector3d scale(1,0.5,0.5);
        this->dataPtr->visual->SetScale(scale);
    }
}

/////////////////////////////////////////////////
void VisualUtils::onRequest(VisualUtilsRequestPtr &_msg)
{
    // TEST
    this->dataPtr->update = true;
}

/////////////////////////////////////////////////
void VisualUtils::randomMaterialName(std::string &name)
{
    // TODO - Change to decent RNG
    int r = rand() % this->dataPtr->patterns.size();
    name = this->dataPtr->patterns.at(r);
    r = rand() % this->dataPtr->variants;
    name = name + std::to_string(r);
}

}
