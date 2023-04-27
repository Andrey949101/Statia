#!/usr/bin/env python
import rospy
from study_pkg.srv import Poly, PolyRequest, PolyResponse

rospy.wait_for_service('poly_server')

try:
    poly_srv = rospy.ServiceProxy('poly_server', Poly)
    req = PolyRequest(x=5)
    resp = poly_srv(req)

    rospy.loginfo('Response: %s' % resp.y)
except rospy.ServiceException:
    rospy.logerr("Service call failed: %s" % e)