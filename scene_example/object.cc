/// \file capture_example/object.cc
/// \brief Object class implementation

#include "object.hh"

//////////////////////////////////////////////////
Object::Object(
    int & _type,
    const std::string & _name,
    const ignition::math::Pose3d & _pose,
    const ignition::math::Vector3d & _scale,
    const std::vector<double> & _parameters) :
        type(_type), name(_name), pose(_pose), scale(_scale), parameters(_parameters)
{
    // Store 3D points at the object surface
    sampleSurface();
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
    float size_y,
    float size_z):
        num_cells_x(num_x),
        num_cells_y(num_y),
        grid_x(size_x),
        grid_y(size_y),
        cell_z(size_z)
{
    // Create cell array
    int num_cells = num_cells_x * num_cells_y;
    for (int i = 0; i < num_cells; i++) {
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

    // Clear existing objects
    objects.clear();
    // Reset object counters
    for (int i = 0; i < 3; i++) counters[i] = 0;
    
    // Shuffle grid cells, for random placement
    shuffleIntVector(cells);
    for (int i = 0; i < num_objects; i++) {
        // Get random cell coordinates
        cell_id = cells[i];
        x = floor(cell_id / num_cells_x);
        y = floor(cell_id - x * num_cells_x);
        // Create random object at cell (x,y)
        addRandomObject(x,y);
    }
}

//////////////////////////////////////////////////
void ObjectGrid::addRandomObject(int x, int y)
{
    // Type and name
    int type;
    std::string name;
    // Dimensions
    double min;
    double radius,length, box_x, box_y, box_z;
    std::vector<double> parameters;
    // Pose
    double p_x, p_y, p_z;
    // Offsets 
    // TODO - Turn into parameters
    double o_x = 2, o_y = 2, o_z = -3;
    // Scale vector
    double s_x, s_y, s_z;

    // Object type and name
    type = getRandomInt(0, 2);
    this->counters[type]++;
    name = TYPES[type] + "_" + std::to_string(counters[type]);
    
    // TODO - Orientation

    // Auxiliar calculations
    min = std::min(cell_x, cell_y);

    // Dimensions
    if (type == SPHERE || type == CYLINDER)
    {
        // Radius
        radius = getRandomDouble(0.1 * min, 0.45 * min);
        parameters.push_back(radius);

        if (type == CYLINDER) {
            // Length
            length = getRandomDouble(0.1 * min, 0.45 * min);
            parameters.push_back(length);
        }
    }
    else if (type == BOX)
    {
        // Box size in x,y,z
        box_x = getRandomDouble(0.1 * cell_x, 0.8 * cell_x);
        box_y = getRandomDouble(0.1 * cell_y, 0.8 * cell_y);
        box_z = getRandomDouble(0.5 * cell_z, 1.0 * cell_z);
        parameters.push_back(box_x);
        parameters.push_back(box_y);
        parameters.push_back(box_z);
    }

    // Pose
    p_x = (x + 0.5) * (cell_x);
    p_y = (y + 0.5) * (cell_y);
    if (type == SPHERE) {
        p_z = radius * 0.5;
    } else if (type == CYLINDER) {
        p_z = length * 0.5;
    } else if (type == BOX) {
        p_z = box_z * 0.5;
    }

    // Apply offset to pose
    ignition::math::Pose3d pose(p_x - o_x, p_y - o_y, p_z - o_z, 0, 0, 0);
    
    // Scale vector
    if (type == SPHERE) {
        s_x = s_y = s_z = radius;
    } else if (type == CYLINDER) {
        s_x = s_y = radius;
        s_z = length;
    } else if (type == BOX) {
        s_x = box_x;
        s_y = box_y;
        s_z = box_z;
    }
    ignition::math::Vector3d scale(s_x, s_y, s_z);

    this->objects.emplace_back(type, name, pose, scale, parameters);
}