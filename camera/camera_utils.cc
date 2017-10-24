#include "camera_utils.hh"

namespace gazebo {

    /* Register this plugin with the simulator */
    GZ_REGISTER_SENSOR_PLUGIN(CameraUtils)
    
    CameraUtils::CameraUtils() : CameraPlugin(){
        std::cout << "[PLUGIN] Loaded camera tools.\n";
    }

    void CameraUtils::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf){
        CameraPlugin::Load(_parent, _sdf);
        
        /* Subscriber setup */
        this->node = transport::NodePtr(new transport::Node());
        this->node->Init("default");

        /* Create a topic for listening to requests */
        std::string topic_name = CAMERA_UTILS_TOPIC;
        /* Subcribe to the topic */
        this->sub = this->node->Subscribe(topic_name,
            &CameraUtils::onMsg, this);
    }

    void CameraUtils::onMsg(CameraRequestPtr &_msg){

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
        gzmsg << "[PLUGIN] Loaded camera tools.\n";
    }
}
