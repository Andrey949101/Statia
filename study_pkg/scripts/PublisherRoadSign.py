#!/usr/bin/env python3

# msg1 - дорожный знак
# msg2 - расстояние до дорожного знака в сантиметрах
# msg1=000 - стоп
# msg1=001 - только направо
# msg1=010 - только прямо
# msg1=100 - только налево
# msg1=011 - прямо и направо
# msg1=110 - прямо и налево
# msg1=101 - налево и направо


import rospy
from std_msgs.msg import Int64
from std_msgs.msg import Int64MultiArray

rospy.init_node('RoadSignSender')
pub = rospy.Publisher('RoadSign', Int64MultiArray, queue_size=10)
rate = rospy.Rate(1)

def start_talker():
    msg = Int64MultiArray()
    msg1 = Int64()
    msg2 = Int64()
    while not rospy.is_shutdown():
        msg1 = int(input("Введите RoadSign: "))
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