#!/usr/bin/env python
import rospy
from study_pkg.msg import Control

def topic_cb(msg):
    rospy.loginfo('Speed: %d / Steer: %d' % (msg.speed, msg.steer))

rospy.init_node('listener')
rospy.Subscriber('my_topic', Control, topic_cb, queue_size=10)
rospy.spin()