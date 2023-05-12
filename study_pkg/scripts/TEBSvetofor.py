#!/usr/bin/env python3

# msg1 - знак светофора
# msg2 - расстояние до светофора в сантиметрах
# msg1=1 - красный
# msg1=2 - зелёный

import rospy
from dynamic_reconfigure.client import Client
from std_msgs.msg import Int64MultiArray

def callback(msg):
    rospy.loginfo("Polynomial heard: TrafficLight %d; Lenght %d", msg.data[0], msg.data[1])
    if msg.data[0]==1 and msg.data[1]<=50:
        client = Client("/move_base/DWAPlannerROS/")
        client.update_configuration({'max_vel_x': 0.0})
        client.update_configuration({'min_vel_x': 0.0})
        exit()

    else:
        client = Client("/move_base/DWAPlannerROS/")
        client.update_configuration({'max_vel_x': 0.26})
        client.update_configuration({'min_vel_x': -0.26})
        exit()
    
    

if __name__ == '__main__':
    # Инициализация ноды
    rospy.init_node('TEBSvetofor')

    # Создание экземпляра класса Client для взаимодействия со службой rqt_reconfigure
    
    # Изменение значения параметра в rqt_reconfigure
    rospy.Subscriber('TrafficLight', Int64MultiArray, callback, queue_size=10)
    rospy.spin()