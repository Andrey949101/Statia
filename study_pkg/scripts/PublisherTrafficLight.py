#!/usr/bin/env python3

# msg1 - знак светофора
# msg2 - расстояние до светофора в сантиметрах
# msg1=1 - красный
# msg1=2 - зелёный

import rospy
from std_msgs.msg import Int64
from std_msgs.msg import Int64MultiArray

rospy.init_node('TrafficLightSender')
pub = rospy.Publisher('TrafficLight', Int64MultiArray, queue_size=10)
rate = rospy.Rate(1)

def start_talker():
    msg = Int64MultiArray()
    msg1 = Int64()
    msg2 = Int64()
    while not rospy.is_shutdown():
        msg1 = int(input("Введите TrafficLight: "))
        msg2 = int(input("Введите Lenght: "))
        msg.data = msg1,msg2
        rospy.loginfo("send msg1: %d, msg2: %d", msg1, msg2)
        pub.publish(msg)
        rate.sleep()


try:
    start_talker()
    rospy.spin()
except (rospy.ROSInterruptException, KeyboardInterrupt):
    rospy.logerr('Exception catched')