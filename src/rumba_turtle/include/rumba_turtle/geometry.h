#ifndef SRC_RUMBA_TURTLE_INCLUDE_RUMBA_TURTLE_GEOMETRY_H_
#define SRC_RUMBA_TURTLE_INCLUDE_RUMBA_TURTLE_GEOMETRY_H_

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"

namespace geometry {

geometry_msgs::Vector3 getVector3(const double x = 0, const double y = 0, const double z = 0);

geometry_msgs::Twist getTwist(const geometry_msgs::Vector3& linear_vector,
                              const geometry_msgs::Vector3& angular_vector);

geometry_msgs::Twist getTwist(const double linear_x = 0, const double linear_y = 0,
                              const double linear_z = 0, const double angular_x = 0,
                              const double angular_y = 0, const double angular_z = 0);

};  // namespace geometry

#endif  // SRC_RUMBA_TURTLE_INCLUDE_RUMBA_TURTLE_GEOMETRY_H_
