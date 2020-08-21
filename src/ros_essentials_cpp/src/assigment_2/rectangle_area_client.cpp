#include <cstdlib>

#include "ros/ros.h"
#include "ros_essentials_cpp/RectangleArea.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "add_two_ints_client");
    if (argc != 3) {
        ROS_ERROR("usage: add_two_ints_client X Y");
        return 1;
    }

    ros::NodeHandle n;
    ros::ServiceClient client =
        n.serviceClient<ros_essentials_cpp::RectangleArea>("rectangle_area_service");
    ros_essentials_cpp::RectangleArea srv;
    srv.request.width = atof(argv[1]);
    srv.request.height = atof(argv[2]);
    if (client.call(srv)) {
        ROS_INFO("Sum: %f", srv.response.area);
    } else {
        ROS_ERROR("Failed to call service rectangle_area_service");
        return 1;
    }

    return 0;
}
