#!/usr/bin/env python
import rospy
from study_pkg.msg import Control

rospy.init_node('talker')
pub = rospy.Publisher('my_topic', Control, queue_size=10)
rate = rospy.Rate(10)

def start_talker():
    msg = Control()
    i=10
    j=40
    while not rospy.is_shutdown():
        
        rospy.loginfo('Speed: %d / Steer: %d' % (msg.speed, msg.steer))

        i=i+1
        j=j+1

        msg.speed=i
        msg.steer=j
        pub.publish(msg)

        rate.sleep()

try:
    start_talker()
except (rospy.ROSInterruptException, KeyboardInterrupt):
    rospy.logerr('Exception catched')
