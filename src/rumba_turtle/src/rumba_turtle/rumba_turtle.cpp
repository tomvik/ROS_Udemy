/*
 * Author: Tomas Lugo from RoBorregos
 * Year: 2020
 *
 */
#include <sstream>

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "turtlesim/Pose.h"

ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;
turtlesim::Pose turtlesim_pose;

constexpr double x_min = 0.0;
constexpr double y_min = 0.0;
constexpr double x_max = 11.0;
constexpr double y_max = 11.0;

constexpr double PI = 3.14159265359;

const char* const kSubscriberTopic = "/turtle1/pose";
const char* const kPublisherTopic = "/turtle1/cmd_vel";
const int kQueueSize = 1000;
const int kMoveRateFrequency = 100;

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

// Publishes a Twist message to the sent publisher, and does that until the distance
// traveled has been reached. The checking frequency can be changed, and its value is in Hz.
// For the moment it is calculated as Distance = speed * time.
// Some good later work would be to add odometry.
void moveStraight(const ros::Publisher& velocity_publisher, const double speed,
                  const double desired_distance, const bool forward,
                  const int loop_frequency = 100) {
    const auto& linear_vel = getVector3(forward ? speed : -1 * speed, 0, 0);
    const auto& twist_msg = getTwist(linear_vel, getVector3(0, 0, 0));

    double initial_time = ros::Time::now().toSec();
    double traveled_distance = 0;
    ros::Rate loop_rate(loop_frequency);
    do {
        velocity_publisher.publish(twist_msg);
        loop_rate.sleep();

        const double current_time = ros::Time::now().toSec();

        traveled_distance += speed * (current_time - initial_time);
        initial_time = current_time;
        // ROS_INFO("[DISTANCE] distance: %f", traveled_distance);
    } while (desired_distance >= traveled_distance);
}

void rotate(double angular_speed, double angle, bool clockwise);
double degrees2radians(double angle_in_degrees);
void setDesiredOrientation(double desired_angle_radians);
void poseCallback(const turtlesim::Pose::ConstPtr& pose_message);
void moveGoal(turtlesim::Pose goal_pose, double distance_tolerance);
void gridClean();
void spiralClean();

void poseCallback(const turtlesim::Pose::ConstPtr& msg) {
    std::stringstream ss;
    ss << "\nx: " << msg->x << "\ny: " << msg->y << "\ntheta: " << msg->theta
       << "\nlinear_velocity: " << msg->linear_velocity
       << "\nangular_velocity: " << msg->angular_velocity;
    ROS_INFO("[Listener] I heard: [%s]\n", ss.str().c_str());
}

int main(int argc, char** argv) {
    // Initiate a new ROS node named "listener"
    ros::init(argc, argv, "rumba_turtle");
    // create a node handle: it is reference assigned to a new node
    ros::NodeHandle node;

    ros::Subscriber sub = node.subscribe(kSubscriberTopic, kQueueSize, poseCallback);
    ros::Publisher pub = node.advertise<geometry_msgs::Twist>(kPublisherTopic, kQueueSize);

    ros::Rate loop_rate(0.25);

    bool direction = true;
    while (ros::ok()) {
        ros::spinOnce();

        ROS_INFO("[Publisher] I wrote something");
        moveStraight(pub, 1, 5, direction);
        direction = !direction;

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
