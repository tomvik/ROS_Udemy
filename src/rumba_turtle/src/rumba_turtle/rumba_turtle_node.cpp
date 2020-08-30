/*
 * Author: Tomas Lugo from RoBorregos
 * Year: 2020
 *
 */
#include <sstream>

#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "rumba_turtle/movement.h"
#include "std_msgs/String.h"
#include "turtlesim/Pose.h"

const char* const kSubscriberTopic = "/turtle1/pose";
const char* const kPublisherTopic = "/turtle1/cmd_vel";
const int kQueueSize = 1000;
const int kMoveRateFrequency = 100;

int main(int argc, char** argv) {
    // Initiate a new ROS node named "listener"
    ros::init(argc, argv, "rumba_turtle");
    // create a node handle: it is reference assigned to a new node
    ros::NodeHandle node;

    ros::Subscriber sub = node.subscribe(kSubscriberTopic, kQueueSize, movement::poseCallback);
    ros::Publisher pub = node.advertise<geometry_msgs::Twist>(kPublisherTopic, kQueueSize);

    ros::Rate loop_rate(0.25);

    bool direction = true;
    while (ros::ok()) {
        ros::spinOnce();

        ROS_INFO("[Publisher] I wrote something");
        movement::moveStraight(pub, 1, 3, direction);
        direction = !direction;

        movement::rotateAbsolute(pub, 1, movement::deg2Rad(90));
        movement::moveStraight(pub, 1, 3, true);
        movement::rotateAbsolute(pub, 1, movement::deg2Rad(180));
        movement::moveStraight(pub, 1, 3, true);
        movement::rotateAbsolute(pub, 1, movement::deg2Rad(270));
        movement::moveStraight(pub, 1, 3, true);
        movement::rotateAbsolute(pub, 1, movement::deg2Rad(0));
        movement::moveStraight(pub, 1, 3, true);
        movement::rotateAbsolute(pub, 1, movement::deg2Rad(270));
        movement::moveStraight(pub, 1, 3, true);
        movement::rotateAbsolute(pub, 1, movement::deg2Rad(180));
        movement::moveStraight(pub, 1, 3, true);
        movement::rotateAbsolute(pub, 1, movement::deg2Rad(90));
        movement::moveStraight(pub, 1, 3, true);
        movement::rotateAbsolute(pub, 1, movement::deg2Rad(0));
        movement::moveStraight(pub, 1, 3, true);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
