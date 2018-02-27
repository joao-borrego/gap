/**
 * @file object.hh
 * @brief Object class headers
 */

#include "object.hh"

//////////////////////////////////////////////////
Object::Object(
    std::string & _name,
    int & _type,
    ignition::math::Pose3d _pose,
    const std::vector<double> & _parameters
    ) : name(_name), type(_type), pose(_pose), parameters(_parameters)
{
    sampleSurface();
}

//////////////////////////////////////////////////
Object::Object(
    float x,
    float y
    )
{
    type = getRandomInt(0, 2);
}

//////////////////////////////////////////////////
void Object::sampleSurface()
{
    // Generate surface points from object parameters
    if (type == CYLINDER) {

        double radius = fabs(parameters[0]);
        double length = fabs(parameters[1]);
        
        // Generate points on the circular edge of the cylinder
        for(int i = 0; i < TOTAL_STEPS_C ; i++){

            float x,y,z;
            x = cos(ANGLE_STEP_C * i) * radius;
            y = sin(ANGLE_STEP_C * i) * radius;
            z = 0.5*length;

            Eigen::Vector4f point1;
            point1 << x, y, z, 1.0;
            object_points.push_back(point1);

            z= -0.5*length;
            Eigen::Vector4f point2; 
            point2 << x, y, z, 1.0;
            object_points.push_back(point2);
        }

    } else if (type == SPHERE) {

        double radius = fabs(parameters[0]);
        
        for (int i = 0; i < TOTAL_STEPS_S; i++){
            for (int j = 0; j < TOTAL_STEPS_S; j++){
                
                float x,y,z;
                x = radius*sin(ANGLE_STEP_S*i)*cos(ANGLE_STEP_S*j);
                y = radius*sin(ANGLE_STEP_S*i)*sin(ANGLE_STEP_S*j);
                z = radius*cos(ANGLE_STEP_S*i);

                Eigen::Vector4f point;
                point << x, y, z, 1.0;
                object_points.push_back(point);
            }
        }
    
    } else if (type == BOX) {

        double size_x = fabs(parameters[0]);
        double size_y = fabs(parameters[1]);
        double size_z = fabs(parameters[2]);

        float x,y,z;
        for (int i=-1; i < 2 ; i+= 2){
            for (int j=-1; j < 2; j+= 2){
                for (int k=-1; k < 2; k+= 2){

                    x = i * size_x / 2.0;
                    y = j * size_y / 2.0;
                    z = k * size_z / 2.0;
                    Eigen::Vector4f point;
                    point << x, y, z, 1.0;
                    object_points.push_back(point);
                }
            }
        }
    }

    // Transform Ignition to Eigen
    Eigen::Matrix3f rot;
    rot = Eigen::Quaternionf(pose.Rot().W(),pose.Rot().X(),pose.Rot().Y(),pose.Rot().Z());

    Eigen::Matrix4f transf;
    transf.block(0,0,3,3) = rot;
    transf.block(0,3,3,1) = Eigen::Vector3f(pose.Pos().X(),pose.Pos().Y(),pose.Pos().Z());

    for (int i = 0; i < object_points.size(); i++){
        object_points[i] = transf * object_points[i];
    }
}

//////////////////////////////////////////////////
ObjectGrid::ObjectGrid(
    int num_x,
    int num_y,
    float size_x,
    float size_y):
        num_cells_x(num_x),
        num_cells_y(num_y),
        grid_x(size_x),
        grid_y(size_y)
{
    // Create cell array
    int num_cells = num_cells_x * num_cells_y;
    for (int i = 0; i < num_cells; i++){
        cells.push_back(i);
    }

    // Compute each cell dimensions
    cell_x = grid_x / num_cells_x;
    cell_y = grid_y / num_cells_y;
}

//////////////////////////////////////////////////
void ObjectGrid::populate(int num_objects)
{
    int cell_id, x, y;

    // Shuffle grid cells, for random placement
    shuffleIntVector(cells);
    for (int i = 0; i < num_objects; i++){
        cell_id = cells[i];
        x = floor(cell_id / num_cells_x);
        y = floor(cell_id - x * cell_x);
    }
}
