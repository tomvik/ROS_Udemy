#include "rumba_turtle/geometry.h"

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"

namespace geometry {

geometry_msgs::Vector3 getVector3(const double x, const double y, const double z) {
    geometry_msgs::Vector3 vector3;
    vector3.x = x;
    vector3.y = y;
    vector3.z = z;
    return vector3;
}

geometry_msgs::Twist getTwist(const geometry_msgs::Vector3& linear_vector,
                              const geometry_msgs::Vector3& angular_vector) {
    geometry_msgs::Twist twist;
    twist.linear = linear_vector;
    twist.angular = angular_vector;
    return twist;
}

geometry_msgs::Twist getTwist(const double linear_x, const double linear_y,
                              const double linear_z, const double angular_x,
                              const double angular_y, const double angular_z) {
    return getTwist(getVector3(linear_x, linear_y, linear_z),
                    getVector3(angular_x, angular_y, angular_z));
}

};  // namespace geometry
