/*
 * Author: Tomas Lugo from RoBorregos
 * Year: 2020
 *
 */
#include <sstream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "rumba_turtle/movement.h"

ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;
turtlesim::Pose turtlesim_pose;

const char* const kSubscriberTopic = "/turtle1/pose";
const char* const kPublisherTopic = "/turtle1/cmd_vel";
const int kQueueSize = 1000;
const int kMoveRateFrequency = 100;

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
        movement::moveStraight(pub, 1, 5, direction);
        direction = !direction;

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
