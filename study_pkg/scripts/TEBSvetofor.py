#!/usr/bin/env python3

import rospy
from dynamic_reconfigure.client import Client

if __name__ == '__main__':
    # Инициализация ноды
    rospy.init_node('node_name')

    # Создание экземпляра класса Client для взаимодействия со службой rqt_reconfigure
    client = Client("/move_base/DWAPlannerROS/")

    # Изменение значения параметра в rqt_reconfigure
    client.update_configuration({'max_vel_x': 0.0})
    client.update_configuration({'min_vel_x': 0.0})