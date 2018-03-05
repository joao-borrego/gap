/*!
    \file visual_utils/VisualUtils.hh
    \brief Visual Utils plugin implementation

    A custom gazebo plugin that provides an interface to programatically
    change the visual properties of an object.

    \author João Borrego : jsbruglie
    \author Rui Figueiredo : ruipimentelfigueiredo
*/

#include "VisualUtils.hh"

namespace gazebo {

/// \class VisualUtils VisualUtils.hh
/// \brief Private data for the VisualUtils class
class VisualUtilsPrivate
{
    /// Visual to which the plugin is attached
    public: rendering::VisualPtr visual;
    /// Connects to rendering update event
    public: event::ConnectionPtr updateConnection;
    /// Gazebo transport node
    public: transport::NodePtr node;
    /// Visual utils topic subscriber
    public: transport::SubscriberPtr sub;
    /// A publisher to the reply topic
    public: transport::PublisherPtr pub;

    /// Unique name
    public: std::string name;
    /// Material name patterns
    public: std::vector<std::string> patterns;
    /// Number of material type variants
    public: int variants;
    /// Default pose
    public: ignition::math::Pose3d default_pose;

    /// Mutex
    public: std::mutex mutex;

    /// Flag to update pose
    public: bool update_pose {false};
    /// Flag to update material
    public: bool update_material {false};
    /// Flag to update scale
    public: bool update_scale {false};

    /// New pose
    public: ignition::math::Pose3d new_pose;
    /// New material
    public: std::string new_material;
    /// New scale
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
    // Setup publisher for the response topic
    this->dataPtr->pub = this->dataPtr->node->
        Advertise<visual_utils::msgs::VisualUtilsResponse>(RESPONSE_TOPIC);

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
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

    visual_utils::msgs::VisualUtilsResponse msg;
    bool updated = false;

    // Update scale
    if (this->dataPtr->update_scale) {
        if (this->dataPtr->visual->Scale() != this->dataPtr->new_scale) {
            this->dataPtr->visual->SetScale(this->dataPtr->new_scale);
        }
        this->dataPtr->update_scale = false;
        updated = true;
    }
    // Update pose
    if (this->dataPtr->update_pose) {
        if (this->dataPtr->visual->WorldPose() != this->dataPtr->new_pose) {
            this->dataPtr->visual->SetWorldPose(this->dataPtr->new_pose);
        }
        this->dataPtr->update_pose = false;
        updated = true;
    }
    // Update material
    if (this->dataPtr->update_material) {
        this->dataPtr->visual->SetMaterial(this->dataPtr->new_material);
        this->dataPtr->update_material = false;
        updated = true;
    }

    // Notify subscribers to the response topic that visual was updated
    if (updated) {
        msg.set_type(UPDATED);
        msg.set_origin(this->dataPtr->name);
        this->dataPtr->pub->Publish(msg);
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
        std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

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
        std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

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
