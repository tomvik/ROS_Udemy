#ifndef SRC_RUMBA_TURTLE_INCLUDE_RUMBA_TURTLE_MOVEMENT_H_
#define SRC_RUMBA_TURTLE_INCLUDE_RUMBA_TURTLE_MOVEMENT_H_

#include "ros/ros.h"
#include "turtlesim/Pose.h"

namespace movement {

constexpr double x_min = 0.0;
constexpr double y_min = 0.0;
constexpr double x_max = 11.0;
constexpr double y_max = 11.0;

constexpr double PI = 3.14159265359;

// Returns the radian equivalent of a degree.
double deg2Rad(const double angle_in_degrees);

// Returns the radian angle but in betweeen -PI and PI.
double properRad(const double angle);

// Publishes a Twist message to the sent publisher, and does that until the distance
// traveled has been reached. The checking frequency can be changed, and its value is in Hz.
// For the moment it is calculated as Distance = speed * time.
// Some good later work would be to add odometry.
void moveStraight(const ros::Publisher& velocity_publisher, const double speed,
                  const double desired_distance, const bool forward,
                  const int loop_frequency = 100);

// Rotates the turtle on its position for the amount given by relative_angle.
// Positive rotation is considered counter-clockwise.
void rotateRelative(const ros::Publisher& velocity_publisher, const double angular_speed,
                    const double desired_relative_angle, const bool clockwise,
                    const int loop_frequency = 100);

// Rotates the turtle on its position until it reaches the desired angle.
// It decides on wheter to go clockwise or counter-clockwise depending on the minimum distance.
void rotateAbsolute(const ros::Publisher& velocity_publisher, const double angular_speed,
                    double desired_angle_radians, const int loop_frequency = 100);

// Prints the info message and updates the turtle_pose variable.
void poseCallback(const turtlesim::Pose::ConstPtr& pose_message);

// Returns true if the euclidean distance is smaller than the tolerance distance.
bool withinRange(const double delta_x, const double delta_y, const double distance_tolerance);

// Moves the turtle within the tolerance distance of the goal pose, while using a K
// controller for the velocity and turn.
void goToGoal(const ros::Publisher& velocity_publisher, const turtlesim::Pose& goal_pose,
              const double distance_tolerance, const int loop_frequency);

// Starting in the bottom left corner, goes throughout the whole screen "cleaning"
// its way going on a zig-zag fashion.
void gridClean(const ros::Publisher& velocity_publisher, const int loop_frequency);

// Starting in the middle, it "cleans" most of the screen, going in an outward spiral
// ever increasing until it is near the edge.
void spiralClean(const ros::Publisher& velocity_publisher);

};  // namespace movement

#endif  // SRC_RUMBA_TURTLE_INCLUDE_RUMBA_TURTLE_MOVEMENT_H_
