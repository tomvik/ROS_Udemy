#include "rumba_turtle/movement.h"

#include "rumba_turtle/geometry.h"

namespace movement {

turtlesim::Pose turtle_pose;

double deg2Rad(const double angle_in_degrees) { return angle_in_degrees * PI / 180; }

double properRad(const double angle) {
    if (abs(angle) <= PI) {
        return angle;
    }

    const int complete_turns = angle / (2 * PI);
    const int incomplete_turns = static_cast<int>(angle / PI) % 2;

    return angle - ((complete_turns + incomplete_turns) * 2 * PI);
}

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
    } while (traveled_distance < desired_distance);
}

void rotateRelative(const ros::Publisher& velocity_publisher, const double angular_speed,
                    const double desired_relative_angle, const bool clockwise,
                    const int loop_frequency) {
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
    } while (traveled_angle < desired_relative_angle);
}

void rotateAbsolute(const ros::Publisher& velocity_publisher, const double angular_speed,
                    double desired_angle_radians, const int loop_frequency) {
    double delta_angle = properRad(desired_angle_radians - turtle_pose.theta);

    /*
    ROS_INFO("[ROTATION] original_difference: %f new_difference: %f desired: %f original: %f",
             desired_angle_radians - turtle_pose.theta, delta_angle, desired_angle_radians,
             turtle_pose.theta);
    */
    const bool clockwise = delta_angle < 0;

    rotateRelative(velocity_publisher, angular_speed, abs(delta_angle), clockwise, loop_frequency);
}

void poseCallback(const turtlesim::Pose::ConstPtr& pose_message) {
    turtle_pose.x = pose_message->x;
    turtle_pose.y = pose_message->y;
    turtle_pose.theta = pose_message->theta;
    /*
    std::stringstream ss;
    ss << "\nx: " << pose_message->x << "\ny: " << pose_message->y
       << "\ntheta: " << pose_message->theta
       << "\nlinear_velocity: " << pose_message->linear_velocity
       << "\nangular_velocity: " << pose_message->angular_velocity;
    ROS_INFO("[Listener] I heard: [%s]\n", ss.str().c_str());
    */
}

};  // namespace movement
