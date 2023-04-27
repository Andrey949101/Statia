#!/usr/bin/env python
import rospy
from study_pkg.msg import Control
from std_msgs.msg import Int64

def callback(msg):
    itog = msg.x + msg.y
    pub = rospy.Publisher('result', Int64, queue_size=10)
    pub.publish(itog)
    rospy.loginfo("Itog = %d", itog)



rospy.init_node('Summmmm')
rospy.Subscriber('raschet', Control, callback, queue_size=10)
rospy.spin()