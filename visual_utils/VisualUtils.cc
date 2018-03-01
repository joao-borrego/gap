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
    /// \brief Gazebo transport node
    public: transport::NodePtr node;
    /// \brief Visual utils topic subscriber
    public: transport::SubscriberPtr sub;

    /// \brief Unique name
    public: std::string name;
    /// \brief Material name patterns
    public: std::vector<std::string> patterns;
    /// \brief Number of material type variants
    public: int variants;
    /// \brief Default pose
    public: ignition::math::Pose3d default_pose;

    /// \brief Flag to update pose
    public: bool update_pose {false};
    /// \brief Flag to update material
    public: bool update_material {false};
    /// \brief Flag to update scale
    public: bool update_scale {false};

    /// \brief New pose
    public: ignition::math::Pose3d new_pose;
    /// \brief New material
    public: std::string new_material;
    /// \brief New scale
    public: ignition::math::Vector3d new_scale;
};

/// Register this plugin with the simulator
GZ_REGISTER_VISUAL_PLUGIN(VisualUtils)

/////////////////////////////////////////////////
VisualUtils::VisualUtils(): VisualPlugin(), dataPtr(new VisualUtilsPrivate)
{
}

/////////////////////////////////////////////////
VisualUtils::~VisualUtils()
{
    this->dataPtr->sub.reset();
    this->dataPtr->node->Fini();
    gzmsg << "[VisualUtils] Unloaded visual tools: " << this->dataPtr->name << std::endl;
}

/////////////////////////////////////////////////
void VisualUtils::Load(rendering::VisualPtr _visual, sdf::ElementPtr _sdf)
{
    // Check if attached to valid visual
    if (!_visual || !_sdf) {
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

    // Plugin parameters

    // Unique name
    if (_sdf->HasElement("uid")) {
        this->dataPtr->name = _sdf->Get<std::string>("uid");
    } else {
        this->dataPtr->name = DEFAULT_NAME;
    }

    // Possible patterns for material names
    if (_sdf->HasElement("patterns")) {
        std::string patterns_arg(_sdf->Get<std::string>("patterns"));
        boost::split(this->dataPtr->patterns, patterns_arg,
            boost::is_any_of(" "), boost::token_compress_on);
    } else {
        // TODO
    }

    // Number of possible variants for each pattern
    this->dataPtr->variants = 0;
    if (_sdf->HasElement("variants")) {
        this->dataPtr->variants = _sdf->Get<int>("variants");
    }

    // Default pose
    this->dataPtr->default_pose = _visual->Pose();

    // TODO - Change to decent RNG
    srand(time(NULL));

    gzmsg << "[VisualUtils] Loaded visual tools: " << this->dataPtr->name << std::endl;
}

/////////////////////////////////////////////////
void VisualUtils::Update()
{
    // Update scale
    if (this->dataPtr->update_scale) {
        this->dataPtr->visual->SetScale(this->dataPtr->new_scale);
        this->dataPtr->update_scale = false;
    }
    // Update pose
    if (this->dataPtr->update_pose) {
        this->dataPtr->visual->SetPose(this->dataPtr->new_pose);
        this->dataPtr->update_pose = false;
    }
    // Update material
    if (this->dataPtr->update_material) {
        this->dataPtr->visual->SetMaterial(this->dataPtr->new_material);
        this->dataPtr->update_material = false;
    }
}

/////////////////////////////////////////////////
void VisualUtils::onRequest(VisualUtilsRequestPtr &_msg)
{
    // Relevant index of commands in message for the current visual
    int index = -1;

    // Validate msg structure
    if (!_msg->has_type()) {
        gzwarn <<" [VisualUtils] Invalid request received" << std::endl;
        return;
    }

    // Check if current visual is targeted
    for (int i = 0; i < _msg->targets_size(); i++) {
        if (this->dataPtr->name == _msg->targets(i)) {
            index = i; break;
        }
    }

    if (_msg->type() == UPDATE)
    {
        if (index == -1) {
            // Ĩf visual is not targeted, set new pose to default pose
            this->dataPtr->new_pose = this->dataPtr->default_pose;
            this->dataPtr->update_pose = true;
        } else {
            if (index < _msg->poses_size()) {
                this->dataPtr->new_pose = gazebo::msgs::ConvertIgn(_msg->poses(index));
                this->dataPtr->update_pose = true;
            }
            if (index < _msg->scale_size()) {
                this->dataPtr->new_scale = gazebo::msgs::ConvertIgn(_msg->scale(index));
                this->dataPtr->update_scale = true;
            }
            randomMaterialName(this->dataPtr->new_material);
            this->dataPtr->update_material = true;
        }
    }
    else if (_msg->type() == DEFAULT_POSE)
    {
        if (index != -1) {
            if (index < _msg->poses_size()) {
                this->dataPtr->default_pose = gazebo::msgs::ConvertIgn(_msg->poses(index));
            }
        }
    }
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
