#include "rumba_turtle/movement.h"

#include "rumba_turtle/geometry.h"

namespace movement {

constexpr double x_min = 0.0;
constexpr double y_min = 0.0;
constexpr double x_max = 11.0;
constexpr double y_max = 11.0;

constexpr double PI = 3.14159265359;

double degrees2radians(const double angle_in_degrees) { return angle_in_degrees * PI / 180; }

void moveStraight(const ros::Publisher& velocity_publisher, const double speed,
                  const double desired_distance, const bool forward, const int loop_frequency) {
    const auto& twist_msg = geometry::getTwist(forward ? abs(speed) : -abs(speed));

    double initial_time = ros::Time::now().toSec();
    double traveled_distance = 0;
    ros::Rate loop_rate(loop_frequency);
    do {
        velocity_publisher.publish(twist_msg);
        loop_rate.sleep();

        const double current_time = ros::Time::now().toSec();

        traveled_distance += speed * (current_time - initial_time);
        initial_time = current_time;

        ros::spinOnce();
        // ROS_INFO("[DISTANCE] distance: %f", traveled_distance);
    } while (desired_distance >= traveled_distance);
}

void rotate(const ros::Publisher& velocity_publisher, const double angular_speed,
            const double desired_relative_angle, const bool clockwise, const int loop_frequency) {
    const auto& twist_msg =
        geometry::getTwist(0, 0, 0, 0, 0, clockwise ? -abs(angular_speed) : abs(angular_speed));

    double initial_time = ros::Time::now().toSec();
    double traveled_angle = 0;
    ros::Rate loop_rate(loop_frequency);
    do {
        velocity_publisher.publish(twist_msg);
        loop_rate.sleep();

        const double current_time = ros::Time::now().toSec();

        traveled_angle += angular_speed * (current_time - initial_time);
        initial_time = current_time;

        ros::spinOnce();
        // ROS_INFO("[ROTATION] rotation: %f", traveled_angle);
    } while (desired_relative_angle >= traveled_angle);
}

};  // namespace movement
