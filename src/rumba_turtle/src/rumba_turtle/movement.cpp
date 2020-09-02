#include "rumba_turtle/movement.h"

#include <algorithm>

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
    const double delta_angle = properRad(desired_angle_radians - turtle_pose.theta);

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

bool withinRange(const double delta_x, const double delta_y, const double distance_tolerance) {
    return sqrt(pow(delta_x, 2) + pow(delta_y, 2)) < distance_tolerance;
}

void goToGoal(const ros::Publisher& velocity_publisher, const turtlesim::Pose& goal_pose,
              const double distance_tolerance, const int loop_frequency) {
    const double max_linear_vel = 8;
    const double min_linear_vel = 0.3;
    const double max_angular_vel = 18;
    const double kPLinear = max_linear_vel / x_max;
    const double kPAngular = max_angular_vel / PI;

    double delta_x, delta_y;

    ros::Rate loop_rate(loop_frequency);
    do {
        delta_x = goal_pose.x - turtle_pose.x;
        delta_y = goal_pose.y - turtle_pose.y;

        const double delta_angle =
            properRad(atan2(delta_y, delta_x) - turtle_pose.theta) * kPAngular;
        const double delta_linear = sqrt(pow(delta_x, 2) + pow(delta_y, 2)) * kPLinear;

        const bool clockwise = delta_angle < 0;

        const double angular_speed = std::min(delta_angle, max_angular_vel);
        const double linear_speed =
            std::min(std::max(delta_linear, min_linear_vel), max_linear_vel);

        const auto& twist_msg = geometry::getTwist(
            linear_speed, 0, 0, 0, 0, clockwise ? -abs(angular_speed) : abs(angular_speed));

        velocity_publisher.publish(twist_msg);
        loop_rate.sleep();

        ros::spinOnce();
    } while (!withinRange(delta_x, delta_y, distance_tolerance));
    velocity_publisher.publish(geometry::getTwist());
}

void gridClean(const ros::Publisher& velocity_publisher, const int loop_frequency) {
    const std::vector<std::vector<double>> positions{
        {{1, 1, 0},  {10, 1, 0}, {10, 2, 0}, {1, 2, 0},  {1, 3, 0},   {10, 3, 0}, {10, 4, 0},
         {1, 4, 0},  {1, 5, 0},  {10, 5, 0}, {10, 6, 0}, {1, 6, 0},   {1, 7, 0},  {10, 7, 0},
         {10, 8, 0}, {1, 8, 0},  {1, 9, 0},  {10, 9, 0}, {10, 10, 0}, {1, 10, 0}}};
    turtlesim::Pose pose;
    for (int i = 0; i < positions.size(); ++i) {
        pose.x = positions[i][0];
        pose.y = positions[i][1];
        pose.theta = positions[i][2];

        goToGoal(velocity_publisher, pose, 0.5, loop_frequency);
    }
}

void spiralClean(const ros::Publisher& velocity_publisher) {
    const double angular_speed = 4;

    const double linear_increment = 0.5;
    double linear_speed = 0.5;

    const double final_x = 9, final_y = 9, final_tolerance = 1;
    double delta_x, delta_y;

    ros::Rate loop_rate(1);
    do {
        delta_x = final_x - turtle_pose.x;
        delta_y = final_y - turtle_pose.y;

        const auto& twist_msg = geometry::getTwist(linear_speed, 0, 0, 0, 0, angular_speed);
        linear_speed += linear_increment;

        velocity_publisher.publish(twist_msg);
        loop_rate.sleep();

        ros::spinOnce();
    } while (!withinRange(delta_x, delta_y, final_tolerance));
}

};  // namespace movement
