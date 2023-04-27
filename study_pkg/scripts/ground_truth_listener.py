#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from math import sqrt

x_prev, y_prev = None, None # предыдущие координаты робота
distance = 0 # общее пройденное расстояние робота

def callback(data, file):
    global x_prev, y_prev, distance

    x = data.pose.pose.position.x
    y = data.pose.pose.position.y

    # если это первое сообщение, сохраняем координаты без вычисления расстояния
    if x_prev is None or y_prev is None:
        x_prev, y_prev = x, y
        return

    # вычисляем расстояние между текущими и предыдущими координатами
    dx, dy = x - x_prev, y - y_prev
    distance += sqrt(dx*dx + dy*dy)

    # сохраняем текущие координаты как предыдущие для следующего шага
    x_prev, y_prev = x, y

    # записываем текущие координаты в файл position.txt
    with open(file, 'a') as f:
        f.write("{},{}\n".format(x, y))

    print("Robot position: ({}, {}), Total distance: {:.2f} m".format(x, y, distance))

def listener():
    rospy.init_node('amcl_pose_listener', anonymous=True)

    # создаем файл position.txt и записываем заголовок
    workspace_path = "catkin_ws" #os.environ['CATKIN_WORKSPACE']
    file_path = '/home/ubuntupc/catkin_ws/src/filesposition/TEBDiff.txt'
    with open(file_path, 'w') as file:
        file.write("x,y\n")

    # подписываемся на топик /amcl_pose
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, callback, file_path)
    print('x',x_prev,'            y',y_prev)
    rospy.spin()

    # записываем общее расстояние движения в файл position.txt
    with open(file_path, 'a') as f:
        f.write("Total distance: {:.2f} m".format(distance))

if __name__ == '__main__':
    listener()