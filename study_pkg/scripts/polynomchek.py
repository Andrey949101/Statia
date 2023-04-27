#!/usr/bin/env python
import rospy
#from std_msgs.msg import Int64
from study_pkg.msg import Control

def callback(msg):
    
    serv = Control()
    serv.x = msg.x
    serv.y = msg.y ** 2
    serv.z = msg.z ** 3
    pub = rospy.Publisher('raschet', Control, queue_size=10)
    pub.publish(serv)
    rospy.loginfo("Polynomchek is = %d, %d, %d", serv.x, serv.y, serv.z)

rospy.init_node('Polynomcheeek')
rospy.Subscriber('vvod', Control, callback, queue_size=10)

rospy.spin()
