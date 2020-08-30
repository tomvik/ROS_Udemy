#ifndef SRC_RUMBA_TURTLE_INCLUDE_RUMBA_TURTLE_GEOMETRY_H_
#define SRC_RUMBA_TURTLE_INCLUDE_RUMBA_TURTLE_GEOMETRY_H_

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

};  // namespace geometry

#endif  // SRC_RUMBA_TURTLE_INCLUDE_RUMBA_TURTLE_GEOMETRY_H_
