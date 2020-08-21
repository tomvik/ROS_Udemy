/*
 * Author: Anis Koubaa for Gaitech EDU
 * Year: 2016
 *
 */
#include <sstream>

#include "ros/ros.h"
#include "ros_essentials_cpp/IoTSensor.h"
#include "std_msgs/String.h"

// Topic messages callback
void chatterCallback(const ros_essentials_cpp::IoTSensor::ConstPtr& msg) {
    std::stringstream ss;
    ss << "\nid: " << msg->id << "\nname: " << msg->name << "\nhumidity: " << msg->humidity
       << "\ntemperature: " << msg->temperature;
    ROS_INFO("[Listener] I heard: [%s]\n", ss.str().c_str());
}

int main(int argc, char** argv) {
    int a = 1;
    // Initiate a new ROS node named "listener"
    ros::init(argc, argv, "listener_node");
    // create a node handle: it is reference assigned to a new node
    ros::NodeHandle node;

    // Subscribe to a given topic, in this case "chatter".
    // chatterCallback: is the name of the callback function that will be executed each time a
    // message is received.
    ros::Subscriber sub = node.subscribe("iot_sensor_topic", 1000, chatterCallback);

    // Enter a loop, pumping callbacks
    ros::spin();

    return 0;
}
