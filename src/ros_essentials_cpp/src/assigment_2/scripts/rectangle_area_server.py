#!/usr/bin/env python

import rospy
from ros_essentials_cpp.srv import RectangleArea, RectangleAreaRequest, RectangleAreaResponse

def handle_rectangle_area(req):
    # type: (RectangleAreaRequest) -> RectangleAreaResponse
    """
    Handles the server request and returns the area of the rectangle as a response
    """
    print "Returning [%s * %s = %s]"%(req.width, req.height, (req.width * req.height))
    return RectangleAreaResponse(req.width * req.height)

def rectangle_area_server():
    # type: () -> None
    """
    Initializes the server and spins until Ctrl+C
    """
    rospy.init_node('rectangle_area_server')
    s = rospy.Service('rectangle_area_service', RectangleArea, handle_rectangle_area)
    print "Ready to calculate a rectangle area"
    s.spin()
    
if __name__ == "__main__":
    try:
        rectangle_area_server()
    except:
        pass