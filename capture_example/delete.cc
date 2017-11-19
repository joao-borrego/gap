void query2DcameraPoint(
    gazebo::transport::PublisherPtr pub,
    const std::vector<Object> &objects){

    camera_utils::msgs::CameraUtilsRequest msg;
    msg.set_type(CAMERA_POINT_REQUEST);

    for (int i = 0; i < objects.size(); i++){

        std::pair <BoundingBox3d::iterator, BoundingBox3d::iterator> ret;
        ret = bbs_3d.equal_range(objects[i].name);
    
        for (BoundingBox3d::iterator it=ret.first; it!=ret.second; ++it){

            // Generate all prossible combinations for center +- coordinate
            for (int x = 1; x >= -1; x -= 2){
                for (int y = 1; y >= -1; y -= 2){
                    for (int z = 1; z >= -1; z -= 2){

                        ignition::math::Vector3d point = it->second.center;
                        point.X() += x * it->second.size.X() / 2.0;
                        point.Y() += y * it->second.size.Y() / 2.0;
                        point.Z() += z * it->second.size.Z() / 2.0;

                        gazebo::msgs::Vector3d *point_msg = new gazebo::msgs::Vector3d();
                        point_msg->set_x(point.X());
                        point_msg->set_y(point.Y());
                        point_msg->set_z(point.Z());
                        debugPrintTrace(point);

                        camera_utils::msgs::BoundingBoxCamera *bounding_box = 
                            msg.add_bounding_box();
                        bounding_box->set_name(objects[i].name);
                        bounding_box->set_allocated_point3d(point_msg);
                    }
                }
            }
        }
    }
    pub->Publish(msg,false);
}

addModelToRequestMsg(
    world_utils::msgs::WorldUtilsRequest &msg,
    int model_type,
    const double grid_cell_size_x,
    const double grid_cell_size_y,
    std::vector<Object> &objects,
    const std::string &name,
    const bool is_light,
    const bool use_custom_textures,
    const bool use_custom_pose,
    std::vector<std::string> &textures,
    const ignition::math::Vector3d &position,
    const ignition::math::Quaternion<double> &orientation){
    
    world_utils::msgs::Object* object = msg.add_object();

    // Handle model type
    if (model_type == MODEL){
        
        // Load sdf model from file
        std::ifstream infile {model_path};
        std::string model_sdf { 
            std::istreambuf_iterator<char>(infile), std::istreambuf_iterator<char>() };
        
        if (is_light){
            object->set_model_type(CUSTOM_LIGHT);
        } else {
            object->set_model_type(CUSTOM);
        }
        object->set_sdf(model_sdf);
    
    } else {

        // Generate random object from possible types
        int object_type = getRandomInt(0, 2);
        object->set_model_type(classes_id[object_type]);
        object_name = class_instance[model_type] + 
            std::to_string(class_instance_counter[object_type] ++);
        object->set_name(object_name);
    }

    if (use_custom_textures){

        int i = getRandomInt(0, textures.size() - 1);

    }


}