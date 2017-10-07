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

    class WorldPluginTutorial : public WorldPlugin
    {
    public: WorldPluginTutorial() : WorldPlugin()
        {

        }

    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
        {
            double radius = 1.0;
            double px = 0.5, py=0.4, pz=9.0;

            sdf::SDF sphereSDF;
            sphereSDF.SetFromString(getSphereSDF(radius,px,py,pz));
            sdf::ElementPtr model = sphereSDF.Root()->GetElement("model");
            model->GetAttribute("name")->SetFromString("sphere_0");
            _world->InsertModelSDF(sphereSDF);
        }
    };
    GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
}