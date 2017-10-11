/**
 * @file
 * @brief
 * 
 * 
 * 
 * @author João Borrego
 */

#include "object_spawner.hh"

namespace gazebo
{
    ObjectSpawnerPlugin::ObjectSpawnerPlugin() : WorldPlugin(){
        std::cout << "[PLUGIN] Loaded object spawner." << std::endl;
    }

    void ObjectSpawnerPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf){
        
        this->world = _world;

        /* Subscriber setup */
        this->node = transport::NodePtr(new transport::Node());

        #if GAZEBO_MAJOR_VERSION < 8
        this->node->Init(this->world->GetName());
        #else
        this->node->Init(this->world->Name());
        #endif

        /* Create a topic for listening to requests */
        std::string topic_name = OBJECT_SPAWNER_TOPIC;
        /* Subcribe to the topic */
        this->sub = this->node->Subscribe(topic_name,
            &ObjectSpawnerPlugin::onMsg, this);

        /* Setup publisher for the factory topic */
        this->factory_pub = this->node->Advertise<msgs::Factory>("~/factory");
        
        /* Example parameter passing */
        /*
        if (_sdf->HasElement("param1"))
            double param = _sdf->Get<double>("param");
        */
    }

    /* Private methods */

    void ObjectSpawnerPlugin::onMsg(SpawnRequestPtr &_msg){
        
        switch(_msg->type()){
            case object_spawner_msgs::msgs::SpawnRequest::SPHERE : {

                /* DEBUG */
                std::cout << "[INFO] Spawning sphere" << std::endl;
                
                std::string name = _msg->has_name()?
                    _msg->name() : "plugin_sphere_" + std::to_string(this->sphere_counter++);

                double x, y, z;
                x = y = z = 0.0;

                if (_msg->has_pose()){
                    msgs::Vector3d pos = _msg->pose().position();
                    x = pos.x();
                    y = pos.y();
                    z = pos.z();
                }

                double radius = _msg->has_radius()?
                    _msg->radius() : 1.0;
                double mass = _msg->has_mass()?
                    _msg->mass() : 2.0;

                spawnSphere(name, radius, mass, x, y, z);
                break;
            }
            case object_spawner_msgs::msgs::SpawnRequest::CYLINDER : {
                std::cout << "[INFO] Spawning cylinder" << std::endl;
                break;
            }
            case object_spawner_msgs::msgs::SpawnRequest::CUBE : {
                std::cout << "[INFO] Spawning cube" << std::endl;
                break;
            }
            case object_spawner_msgs::msgs::SpawnRequest::REMOVE : {
                std::cout << "[INFO] Removing object" << std::endl;
                break;
            }
            default:
                break;
        }
    }

    void ObjectSpawnerPlugin::printLiveObjs(){
        std::cout << "[PLUGIN] Printing live object list" << std::endl;
        /*
        for (auto v : this->live_objs)
            std::cout << v << std::endl;
        */
        int model_count = this->world->GetModelCount();
        std::cout << model_count << std::endl;

        for (int idx = 0; idx < model_count; idx++){
            physics::ModelPtr model = this->world->GetModel(idx);
            std::cout << model->GetName() << std::endl;
        }
    }

    void ObjectSpawnerPlugin::spawnSphere(
        const std::string &model_name,
        const double radius,
        const double mass,
        const double px,
        const double py,
        const double pz){

        msgs::Model model;
        model.set_name(model_name);
        msgs::Set(model.mutable_pose(), ignition::math::Pose3d(px, py, pz, 0, 0, 0));
        
        msgs::AddSphereLink(model, mass, radius);
        
        //visual_msg.mutable_material(material);
        //visual.set_material(material);
        //model.set_visual(visual);

        std::ostringstream new_model_str;
        new_model_str << "<sdf version='" << SDF_VERSION << "'>"
        << msgs::ModelToSDF(model)->ToString("")
        << "</sdf>";

        /* Send the model to the gazebo server */
        msgs::Factory msg;
        msg.set_sdf(new_model_str.str());
        this->factory_pub->Publish(msg);
    }

    GZ_REGISTER_WORLD_PLUGIN(ObjectSpawnerPlugin)
}