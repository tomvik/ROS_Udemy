#include "ros/ros.h"
#include "ros_essentials_cpp/RectangleArea.h"

bool calculate_area(ros_essentials_cpp::RectangleArea::Request &req,
                    ros_essentials_cpp::RectangleArea::Response &res) {
    try {
        res.area = req.width * req.height;
        ROS_INFO("request: width=%f, height=%f", req.width, req.height);
        ROS_INFO("sending back response: [%f]", res.area);
        return true;
    } catch (...) {
        ROS_ERROR("ERROR OCURRES");
        return false;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "rectangle_area_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("rectangle_area_service", calculate_area);
    ROS_INFO("Ready to calculate a rectangle area.");
    ros::spin();

    return 0;
}
