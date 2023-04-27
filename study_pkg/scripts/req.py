#!/usr/bin/env python
import rospy
from std_msgs.msg import Int64
from study_pkg.msg import Control

def start_talker():
    serv = Control()
    serv.x = int(input("Первое число х= "))
    serv.y = int(input("Второе число у= "))
    serv.z = int(input("Ага, еще и 3 есть z= "))
    while not rospy.is_shutdown():
        pub.publish(serv)
        rospy.Subscriber('result', Int64, callback, queue_size=10)
        rate.sleep()

def callback(msg):
    rospy.loginfo("Result =  %d", msg.data)

rospy.init_node('Req')
pub = rospy.Publisher('vvod', Control, queue_size=10)
rate = rospy.Rate(1)

try:
    start_talker()
except (rospy.ROSInterruptException, KeyboardInterrupt):
    rospy.logerr('Exception catched')
