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

const char* const kSubscriberTopic = "/turtle1/pose";
const char* const kPublisherTopic = "/turtle1/cmd_vel";
const int kQueueSize = 1000;

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

void poseCallback(const turtlesim::Pose::ConstPtr& msg) {
    std::stringstream ss;
    ss << "\nx: " << msg->x << "\ny: " << msg->y << "\ntheta: " << msg->theta
       << "\nlinear_velocity: " << msg->linear_velocity
       << "\nangular_velocity: " << msg->angular_velocity;
    ROS_INFO("[Listener] I heard: [%s]\n", ss.str().c_str());
}

int main(int argc, char** argv) {
    // Initiate a new ROS node named "listener"
    ros::init(argc, argv, "turtle_motion_pose");
    // create a node handle: it is reference assigned to a new node
    ros::NodeHandle node;

    ros::Subscriber sub = node.subscribe(kSubscriberTopic, kQueueSize, poseCallback);
    ros::Publisher pub = node.advertise<geometry_msgs::Twist>(kPublisherTopic, kQueueSize);

    ros::Rate loop_rate(1);

    const geometry_msgs::Vector3& linear_vel = getVector3(1, 0, 0);
    const geometry_msgs::Vector3& angular_vel = getVector3(0, 0, 1);

    const geometry_msgs::Twist twist = getTwist(linear_vel, angular_vel);

    while (ros::ok()) {
        ROS_INFO("[Publisher] I wrote something");

        pub.publish(twist);

        // Need to call this function often to allow ROS to process incoming messages.
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
