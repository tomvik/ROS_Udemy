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
constexpr int kQueueSize = 1000;
constexpr int kMoveRateFrequency = 100;

int main(int argc, char** argv) {
    // Initiate a new ROS node named "listener"
    ros::init(argc, argv, "rumba_turtle");
    // create a node handle: it is reference assigned to a new node
    ros::NodeHandle node;

    ros::Subscriber sub = node.subscribe(kSubscriberTopic, kQueueSize, movement::poseCallback);
    ros::Publisher pub = node.advertise<geometry_msgs::Twist>(kPublisherTopic, kQueueSize);

    ros::Rate loop_rate(0.25);

    bool direction = true;

    turtlesim::Pose goal_pose;
    std::vector<std::vector<double>> poses{{1, 1, 0}, {9, 9, 0}, {9, 1, 0}, {1, 10, 0}, {5, 5, 0}};
    int index = 0;

    movement::gridClean(pub, kMoveRateFrequency);
    // movement::spiralClean(pub);

    /*

    while (ros::ok() && index < poses.size()) {
        ros::spinOnce();

        ROS_INFO("[Publisher] I wrote something");
        goal_pose.x = poses[index][0];
        goal_pose.y = poses[index][1];
        goal_pose.theta = poses[index][2];
        ++index;

        movement::goToGoal(pub, goal_pose, 0.3, kMoveRateFrequency);

        ros::spinOnce();
        loop_rate.sleep();
    }
    */

    return 0;
}
