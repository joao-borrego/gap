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
    std::string getSphereSDF(double radius,
        double px=0, double py=0, double pz=0,
        double rr=0, double rp=0, double ry=0){
        std::stringstream sphere_sdf;
        sphere_sdf <<
        "<sdf version ='1.4'>\
          <model name ='sphere'>\
            <pose>" <<
            std::fixed << std::setprecision(5) << px <<
            std::fixed << std::setprecision(5) << py <<
            std::fixed << std::setprecision(5) << pz <<
            std::fixed << std::setprecision(5) << rr <<
            std::fixed << std::setprecision(5) << rp <<
            std::fixed << std::setprecision(5) << ry << 
            "</pose>\
            <link name ='link'>\
              <pose>0 0 " <<
              std::fixed << std::setprecision(5) << radius <<
              " 0 0 0</pose>\
              <collision name ='collision'>\
                <geometry>\
                  <sphere><radius>" <<
                  std::fixed << std::setprecision(5) << radius <<
                  "</radius></sphere>\
                </geometry>\
              </collision>\
              <visual name ='visual'>\
                <geometry>\
                  <sphere><radius>" <<
                  std::fixed << std::setprecision(5) << radius <<
                  "</radius></sphere>\
                </geometry>\
              </visual>\
            </link>\
          </model>\
        </sdf>";
        return sphere_sdf.str();
    }

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

        /* Create a topic name */
        std::string topic_name = OBJECT_SPAWNER_TOPIC;
        /* Subcribe to the topic */
        this->sub = this->node->Subscribe(topic_name,
            &ObjectSpawnerPlugin::onMsg, this);

        /* Example parameter passing */
        if (_sdf->HasElement("param1"))
            double param = _sdf->Get<double>("param");
    }

    /* Private methods */

    void ObjectSpawnerPlugin::onMsg(ConstVector3dPtr &_msg){
        
        /* Spawns a sphere */
        std::string sphere_name = "sphere_" + std::to_string(this->sphere_counter);
        ObjectSpawnerPlugin::spawnSphere(sphere_name, 1.0, 0.5, 0.4, 9.0);
        ObjectSpawnerPlugin::printLiveObjs();
        this->sphere_counter++;

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
        double radius,
        double px,
        double py,
        double pz){

        sdf::SDF sphereSDF;
        sphereSDF.SetFromString(getSphereSDF(radius,px,py,pz));
        sdf::ElementPtr model = sphereSDF.Root()->GetElement("model");
        model->GetAttribute("name")->SetFromString(model_name);
        this->world->InsertModelSDF(sphereSDF);
    }

    GZ_REGISTER_WORLD_PLUGIN(ObjectSpawnerPlugin)
}