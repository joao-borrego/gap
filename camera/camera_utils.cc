#include "camera_utils.hh"

namespace gazebo {

    /* Register this plugin with the simulator */
    GZ_REGISTER_SENSOR_PLUGIN(CameraUtils)
    
    CameraUtils::CameraUtils() : CameraPlugin(){
        std::cout << "[PLUGIN] Loaded camera tools.\n";
    }

    void CameraUtils::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf){
        CameraPlugin::Load(_parent, _sdf);
        
        std::string world_name;

        /* Plugin parameters */
        if (_sdf->HasElement("world")){
            world_name = _sdf->Get<std::string>("world");
        } else {
            world_name = "default";
        }
        if (_sdf->HasElement("output_dir")){
            this->output_dir = _sdf->Get<std::string>("output_dir"); 
        } else {
            this->output_dir = "/tmp/gazebo_camera/";
        }

        /* Initialize attributes */
        this->camera = this->parentSensor->Camera();
        this->img_fmt = this->camera->ImageFormat();

        /* Subscriber setup */
        this->node = transport::NodePtr(new transport::Node());
        this->node->Init(world_name);

        /* Create a topic for listening to requests */
        std::string topic_name = CAMERA_UTILS_TOPIC;
        /* Subcribe to the topic */
        this->sub = this->node->Subscribe(topic_name,
            &CameraUtils::onMsg, this);
    }

    void CameraUtils::onMsg(CameraRequestPtr &_msg){

        std::string file_name;

        if (_msg->type() == CAPTURE){

            if (_msg->has_file_name()){
                file_name = _msg->file_name();
            } else {
                file_name = "tmp_" + std::to_string(saved_counter++) + img_ext;
            }
            this->parentSensor->SaveFrame(output_dir + file_name);
            std::cout << "Saving frame as [" << output_dir << file_name << "]\n";
        }
    }

    void CameraUtils::OnNewFrame(
        const unsigned char *_image,
        unsigned int _width,
        unsigned int _height,
        unsigned int _depth,
        const std::string &_format){

        /*
        char tmp[1024];
        snprintf(tmp, sizeof(tmp), "/tmp/%s-%04d.jpg",
                this->parentSensor->GetCamera()->GetName().c_str(), this->saveCount);

        if (this->saveCount < 10)
        {
            this->parentSensor->GetCamera()->SaveFrame(
                    _image, _width, _height, _depth, _format, tmp);
            gzmsg << "Saving frame [" << this->saveCount
                        << "] as [" << tmp << "]\n";
            this->saveCount++;
        }
        */
    }
}
