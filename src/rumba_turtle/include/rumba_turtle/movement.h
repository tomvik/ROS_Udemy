#ifndef SRC_RUMBA_TURTLE_INCLUDE_RUMBA_TURTLE_MOVEMENT_H_
#define SRC_RUMBA_TURTLE_INCLUDE_RUMBA_TURTLE_MOVEMENT_H_

#include "ros/ros.h"
#include "turtlesim/Pose.h"

namespace movement {

// Returns the radian equivalent of a degree.
double degrees2radians(const double angle_in_degrees);

// Publishes a Twist message to the sent publisher, and does that until the distance
// traveled has been reached. The checking frequency can be changed, and its value is in Hz.
// For the moment it is calculated as Distance = speed * time.
// Some good later work would be to add odometry.
void moveStraight(const ros::Publisher& velocity_publisher, const double speed,
                  const double desired_distance, const bool forward,
                  const int loop_frequency = 100);

// Rotates the turtle on its position for the amount given by relative_angle.
// Positive rotation is considered counter-clockwise.
void rotate(const ros::Publisher& velocity_publisher, const double angular_speed,
            const double desired_relative_angle, const bool clockwise,
            const int loop_frequency = 100);

void setDesiredOrientation(double desired_angle_radians);
void poseCallback(const turtlesim::Pose::ConstPtr& pose_message);
void moveGoal(turtlesim::Pose goal_pose, double distance_tolerance);
void gridClean();
void spiralClean();

};  // namespace movement

#endif  // SRC_RUMBA_TURTLE_INCLUDE_RUMBA_TURTLE_MOVEMENT_H_
