#include "camera_utils.hh"

namespace gazebo {

    /* Register this plugin with the simulator */
    GZ_REGISTER_SENSOR_PLUGIN(CameraUtils)

    /**
     * @brief      Class for private camera utils plugin data.
     */
    class CameraUtilsPrivate
    {
        public: 

            /** Gazebo transport node */
            public: transport::NodePtr node;
            /** Camera utils topic subscriber */
            public: transport::SubscriberPtr sub;
            /** Camera utils topic publisher */
            public: transport::PublisherPtr pub;
    };

    CameraUtils::CameraUtils()
        : SensorPlugin(), dataPtr(new CameraUtilsPrivate){
        
        std::cout << "[PLUGIN] Loaded camera tools.\n";
    }

    CameraUtils::~CameraUtils(){
        std::cout << "[PLUGIN] Unloaded camera tools.\n";
        this->parentSensor.reset();
        this->camera.reset();
        this->dataPtr->sub.reset();
        this->dataPtr->node->Fini();
    }

    void CameraUtils::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf){
        
         
        if (!_sensor)
            gzerr << "Invalid sensor pointer.\n";

        /* Camera sensor */
        this->parentSensor =
            std::dynamic_pointer_cast<sensors::CameraSensor>(_sensor);
        this->camera = this->parentSensor->Camera();
        this->width = this->camera->ImageWidth();
        this->height = this->camera->ImageHeight();
        this->depth = this->camera->ImageDepth();
        this->format = this->camera->ImageFormat();
        this->parentSensor->SetActive(true);

        /* Plugin parameters */

        std::string world_name;

        if (_sdf->HasElement("world")){
            world_name = _sdf->Get<std::string>("world");
        } else {
            world_name = DEFAULT_WORLD;
        }
        if (_sdf->HasElement("output_dir")){
            this->output_dir = _sdf->Get<std::string>("output_dir"); 
        } else {
            this->output_dir = DEFAULT_OUTPUT_DIR;
        }
        if (_sdf->HasElement("extension")){
            this->extension = _sdf->Get<std::string>("extension"); 
        } else {
            this->extension = DEFAULT_EXTENSION;
        }

        /* Subscriber setup */
        this->dataPtr->node = transport::NodePtr(new transport::Node());
        this->dataPtr->node->Init(world_name);

        /* Create a topic for listening to requests */
        std::string topic_name = REQUEST_TOPIC;
        /* Subcribe to the topic */
        this->dataPtr->sub = this->dataPtr->node->Subscribe(topic_name,
            &CameraUtils::onMsg, this);
        /* Setup publisher for the reply topic */
        this->dataPtr->pub = this->dataPtr->node->Advertise<camera_utils_msgs::msgs::CameraReply>(REPLY_TOPIC);

        /* Create output directory */
        boost::filesystem::path dir(output_dir);
        boost::filesystem::create_directories(dir);
    }

    void CameraUtils::onMsg(CameraRequestPtr &_msg){
        
        std::string file_name;

        if (_msg->type() == CAPTURE){

            if (_msg->has_file_name()){
                file_name = _msg->file_name() + extension;
            } else {
                file_name = "tmp_" + std::to_string(saved_counter++) + extension;
            }

            this->camera->Update();
            bool success = this->camera->SaveFrame(output_dir + file_name);
            std::cout << "Saving frame as [" << output_dir << file_name << "]\n";

            camera_utils_msgs::msgs::CameraReply msg;
            msg.set_success(success);
            this->dataPtr->pub->Publish(msg);
        }
    }
}
