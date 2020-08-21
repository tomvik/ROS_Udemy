#!/usr/bin/env python

import sys
import rospy
from ros_essentials_cpp.srv import RectangleArea

def rectangle_area_client(width, height):
    # type: (float, float) -> float
    """
    Connects to service, sends a request and returns the value of the response.
    """
    rospy.wait_for_service('rectangle_area_service')
    try:
        rectangle_area_service = rospy.ServiceProxy('rectangle_area_service', RectangleArea)
        resp1 = rectangle_area_service(width, height)
        return resp1.area
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [width height]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        width = float(sys.argv[1])
        height = float(sys.argv[2])
    else:
        print usage()
        sys.exit(1)
    print "Requesting %s * %s"%(width, height)
    area = rectangle_area_client(width, height)
    print "%s * %s = %s"%(width, height, area)