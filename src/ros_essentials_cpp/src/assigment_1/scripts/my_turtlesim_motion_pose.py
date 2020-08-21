#!/usr/bin/env python
from typing import Tuple
import rospy
from geometry_msgs.msg import Twist, Vector3
from turtlesim.msg import Pose

def parse_pose_msg(pose_msg):
    # type: (Pose) -> Tuple[float, float, float, float, float]
    """ Parses the Pose message and returns its values as a tuple """
    return (pose_msg.x, pose_msg.y, pose_msg.theta, pose_msg.linear_velocity, pose_msg.angular_velocity)

def get_vector3_from_tuple(tup):
    # type: (Tuple[float, float, float]) -> Vector3
    return Vector3(tup[0], tup[1], tup[2])

def set_twist_msg(linear_vel, angular_vel):
    # type: (Tuple[float, float, float], Tuple[float, float, float]) -> Twist
    """ Sets the linear_vel and angular_vel for the Twist """
    return set_twist_msg_from_vector3(get_vector3_from_tuple(linear_vel), get_vector3_from_tuple(angular_vel))

def set_twist_msg_from_vector3(linear_vel, angular_vel):
    # type: (Vector3, Vector3) -> Twist
    """ Sets the linear_vel and angular_vel for the Twist using Vector3 """
    twist = Twist()
    
    twist.linear = linear_vel
    twist.angular = angular_vel

    return twist

def pose_callback(pose_msg):
    # type: (Pose) -> None
    """get_caller_id(): Get fully resolved name of local node1"""
    
    # x, y, theta, linear_velocity, angular_velocity = parse_pose_msg(pose_msg)

    parsed_msg = '\nx: {0[0]}\ny: {0[1]}\ntheta: {0[2]}\nlinear_velocity: {0[3]}\nangular_velocity: {0[4]}\n'.format(parse_pose_msg(pose_msg))

    rospy.loginfo(rospy.get_caller_id() + parsed_msg)
    
def listener():
    # type: () -> None
    """ Initializes the subscriber and makes it listen"""

    rospy.Subscriber("/turtle1/pose", Pose, pose_callback)

    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()

def talker():
    # type: () -> None
    """ Initializes the publisher and makes it talk"""
    # Create a new publisher.
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    
    #rospy.init_node('turtle_vel_talker')

    # Set the loop rate.
    rate = rospy.Rate(10)

    # Keep publishing until a Ctrl-C is pressed
    while not rospy.is_shutdown():
        twist_msg = set_twist_msg((1.5, 0, 0), (0, 0, 2))
        rospy.loginfo(twist_msg)
        pub.publish(twist_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('turtle_pose_listener_and_talker') # if anonymous=True, many listeners can exist
        listener()
        talker()
    except rospy.ROSInterruptException:
        pass
