#!/usr/bin/env python3

# msg1 - знак светофора
# msg2 - расстояние до светофора в сантиметрах
# msg1=1 - красный
# msg1=2 - зелёный

import rospy
from std_msgs.msg import Int64
from std_msgs.msg import Int64MultiArray
import cv2
import numpy as np
import pyrealsense2 as rs
from realsense import Realsense
from detection import *

rospy.init_node('TrafficLightSender')
pub = rospy.Publisher('TrafficLight', Int64MultiArray, queue_size=10)
rate = rospy.Rate(1)

def start_talker():
    msg = Int64MultiArray()
    msg1 = Int64()
    msg2 = Int64()
    realsense = Realsense()
    realsense.start_stream()
    resize_factor = 1
    circle_params = dict(dp=1, minDist=100, param1=30, param2=50, minRadius=19, maxRadius=51)

    while not rospy.is_shutdown():
        # msg1 = int(input("Введите TrafficLight: "))
        # msg2 = int(input("Введите Lenght: "))

        color_frame, depth_frame = realsense.get_frames()

        detected_light, circle = detect_traffic_light_color(color_frame, circle_params)

        if detected_light and circle is not None:
            x, y, r = circle

            if detected_light=='red':
                msg1=1
            elif detected_light=='yellow':
                msg1=1
            elif detected_light=='green':
                msg1=2
            else:
                msg1=2

            color_frame = draw_traffic_light_circle(color_frame, detected_light, circle)

            distance = depth_frame[int(y), int(x)] * realsense.depth_scale
            msg.data=msg1,int(distance*100)
            pub.publish(msg)
            rospy.loginfo(msg.data)
            if distance > 0:
                command = calculate_braking_command(detected_light, distance)
                # print(f"Distance to traffic light: {distance:.2f} meters")
                color_frame = draw_command_on_frame(color_frame, command)
                cv2.putText(color_frame, f"Distance: {distance:.2f} meters", (20, 90), cv2.FONT_HERSHEY_DUPLEX, 1, (0, 0, 255), 1, cv2.LINE_AA)

        depth_colored = cv2.applyColorMap(cv2.convertScaleAbs(depth_frame, alpha=0.12), cv2.COLORMAP_JET)

        depth_colored_resized = cv2.resize(depth_colored, (
            int(depth_colored.shape[1] * resize_factor), int(depth_colored.shape[0] * resize_factor)))

        color_frame_resized = cv2.resize(color_frame, (
            int(color_frame.shape[1] * resize_factor), int(color_frame.shape[0] * resize_factor)))

        combined_frame = cv2.hconcat([color_frame_resized, depth_colored_resized])
        cv2.imshow("Detect and Depth", combined_frame)
        key = cv2.waitKey(1) & 0xFF

        # msg.data = msg1,msg2
        # rospy.loginfo("send msg1: %d, msg2: %d", msg1, msg2)
        # pub.publish(msg)
        # rate.sleep()

        if key == ord('q') or key == ord('Q'):
            break

    realsense.stop_stream()
    cv2.destroyAllWindows()


try:
    start_talker()
    rospy.spin()
except (rospy.ROSInterruptException, KeyboardInterrupt):
    rospy.logerr('Exception catched')