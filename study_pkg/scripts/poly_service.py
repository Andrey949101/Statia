#!/usr/bin/env python

from study_pkg.srv import Poly, PolyResponse
import rospy

def handle_poly_srv(req):
    result = req.x + req.x ** 2
    rospy.loginfo("Returning [%s + %s^2 = %s]" % (req.x, req.x, result))
    
    resp = PolyResponse()
    resp.y = result
    
    return resp

def poly_server():
    rospy.init_node('poly_server')
    s = rospy.Service('poly', Poly, handle_poly_srv)
    rospy.loginfo("Ready to calc polynomial.")
    rospy.spin()

poly_server()