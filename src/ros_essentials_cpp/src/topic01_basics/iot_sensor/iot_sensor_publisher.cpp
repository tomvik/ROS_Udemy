#include <sstream>

#include "ros/ros.h"
#include "ros_essentials_cpp/IoTSensor.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "iot_sensor_talker_node");

    ros::NodeHandle n;

    ros::Publisher chatter_publisher =
        n.advertise<ros_essentials_cpp::IoTSensor>("iot_sensor_topic", 1000);

    ros::Rate loop_rate(0.5);  // 1 message per second

    const int id = 1;
    const std::string name = "iot_1";
    int humidity = 0;
    int temperature = 0;

    while (ros::ok()) {
        // create a new String ROS message.
        // Message definition in this link http://docs.ros.org/api/std_msgs/html/msg/String.html
        ros_essentials_cpp::IoTSensor msg;

        msg.id = id, msg.name = name, msg.humidity = humidity, msg.temperature = temperature;

        std::stringstream ss;
        ss << "\nid: " << msg.id << "\nname: " << msg.name << "\nhumidity: " << msg.humidity
           << "\ntemperature: " << msg.temperature;
        ROS_INFO("[Talker] I published: [%s]\n", ss.str().c_str());

        // Publish the message
        chatter_publisher.publish(msg);

        ros::spinOnce();  // Need to call this function often to allow ROS to process incoming
                          // messages

        loop_rate.sleep();  // Sleep for the rest of the cycle, to enforce the loop rate
        ++humidity;
        temperature += 2;
    }
    return 0;
}
