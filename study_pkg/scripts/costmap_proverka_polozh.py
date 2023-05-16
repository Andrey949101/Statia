#!/usr/bin/env python3

# # msg1 - дорожный знак
# # msg2 - расстояние до дорожного знака в сантиметрах
# # msg1=31 - стоп
# # msg1=412 - только направо
# # msg1=411 - только прямо
# # msg1=413 - только налево
# # msg1=414 - прямо и направо
# # msg1=415 - прямо и налево


import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
from std_msgs.msg import Int64
from std_msgs.msg import Int64MultiArray
from dynamic_reconfigure.client import Client
from time import sleep
import math

RoadSign=0
distance=0
angle=90

class CostmapEditor:
    def __init__(self):
        rospy.init_node('costmap_editor')
        rospy.Subscriber('/map', OccupancyGrid, self.costmap_callback)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)
        self.map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)
        self.map = None
        self.resolution = 0.0
        self.robot_pose = None

    def costmap_callback(self, msg):
        self.map = msg
        self.resolution = msg.info.resolution

    def pose_callback(self, msg):
        self.robot_pose = msg.pose.pose

    def indices_to_coords(self, min_x, min_y, max_x, max_y):
        x_range = np.arange(min_x, max_x, self.resolution)
        y_range = np.arange(min_y, max_y, self.resolution)
        coords = [(x, y) for x in x_range for y in y_range]
        return coords

    def coords_to_indices(self, coords):
        indices = []
        for coord in coords:
            x = int(round((coord[0] - self.map.info.origin.position.x) / self.resolution))
            y = int(round((coord[1] - self.map.info.origin.position.y) / self.resolution))
            index = self.coord_to_index(x, y)
            indices.append(index)
        return indices

    def coord_to_index(self, x, y):
        map_width = self.map.info.width
        index = y * map_width + x
        return index

    def left(self,value, angle):
        if self.robot_pose is None:
            rospy.logwarn('Robot pose is not available yet.')
            return

        x = self.robot_pose.position.x
        y = self.robot_pose.position.y

        min_x = x - 0.51
        min_y = y + 0.5
        max_x = x - 0.5
        max_y = y + 0.51
        min_x_prev=min_x
        min_y_prev=min_y
        max_x_prev=max_x
        max_y_prev=max_y
        print("Left")
        for i in range(150):
            coords = self.indices_to_coords(min_x, min_y, max_x, max_y)
            indices = self.coords_to_indices(coords)

            new_map = OccupancyGrid()
            new_map.header.stamp = rospy.Time.now()
            new_map.header.frame_id = self.map.header.frame_id
            new_map.info = self.map.info
            new_map.data = list(self.map.data)
            for index in indices:
                new_map.data[index] = value
                min_x = min_x_prev + 0.01*math.cos(math.radians(angle))
                min_y = min_y_prev + 0.01*math.sin(math.radians(angle))
                max_x = max_x_prev + 0.01*math.cos(math.radians(angle))
                max_y = max_y_prev + 0.01*math.sin(math.radians(angle))
            self.map_pub.publish(new_map)
            min_x_prev=min_x
            min_y_prev=min_y
            max_x_prev=max_x
            max_y_prev=max_y
        print("done")
        exit()

    def right(self, value, angle):
        if self.robot_pose is None:
            rospy.logwarn('Robot pose is not available yet.')
            return

        x = self.robot_pose.position.x
        y = self.robot_pose.position.y

        min_x = x + 0.5
        min_y = y + 0.5
        max_x = x + 0.51
        max_y = y + 0.51
        min_x_prev=min_x
        min_y_prev=min_y
        max_x_prev=max_x
        max_y_prev=max_y
        print("Right")
        for i in range(150):
            coords = self.indices_to_coords(min_x, min_y, max_x, max_y)
            indices = self.coords_to_indices(coords)

            new_map = OccupancyGrid()
            new_map.header.stamp = rospy.Time.now()
            new_map.header.frame_id = self.map.header.frame_id
            new_map.info = self.map.info
            new_map.data = list(self.map.data)
            for index in indices:
                new_map.data[index] = value
                min_x = min_x_prev + 0.01*math.cos(math.radians(angle))
                min_y = min_y_prev + 0.01*math.sin(math.radians(angle))
                max_x = max_x_prev + 0.01*math.cos(math.radians(angle))
                max_y = max_y_prev + 0.01*math.sin(math.radians(angle))
            self.map_pub.publish(new_map)
            min_x_prev=min_x
            min_y_prev=min_y
            max_x_prev=max_x
            max_y_prev=max_y
        print("done")


    def front(self,value,angle):
        angle_front=angle+90
        if self.robot_pose is None:
            rospy.logwarn('Robot pose is not available yet.')
            return

        x = self.robot_pose.position.x
        y = self.robot_pose.position.y

        min_x = x + 0.5 + 1.5*math.cos(math.radians(angle))
        min_y = y + 0.5 + 1.5*math.sin(math.radians(angle))
        max_x = x + 0.51 + 1.5*math.cos(math.radians(angle))
        max_y = y + 0.51 + 1.5*math.sin(math.radians(angle))
        min_x_prev=min_x
        min_y_prev=min_y
        max_x_prev=max_x
        max_y_prev=max_y
        print("Front")
        for i in range(150):
            coords = self.indices_to_coords(min_x, min_y, max_x, max_y)
            indices = self.coords_to_indices(coords)

            new_map = OccupancyGrid()
            new_map.header.stamp = rospy.Time.now()
            new_map.header.frame_id = self.map.header.frame_id
            new_map.info = self.map.info
            new_map.data = list(self.map.data)
            for index in indices:
                new_map.data[index] = value
                min_x = min_x_prev + 0.01*math.cos(math.radians(angle_front))
                min_y = min_y_prev + 0.01*math.sin(math.radians(angle_front))
                max_x = max_x_prev + 0.01*math.cos(math.radians(angle_front))
                max_y = max_y_prev + 0.01*math.sin(math.radians(angle_front))
            self.map_pub.publish(new_map)
            min_x_prev=min_x
            min_y_prev=min_y
            max_x_prev=max_x
            max_y_prev=max_y
        print("done")
        exit()

def callback1(msg):
    global RoadSign
    RoadSign = msg.data[0]
    global distance
    distance = msg.data[1]
    global angle
    angle = 0
    sopostavlenie()
    
def sopostavlenie():
    global RoadSign, distance, angle
    
    if RoadSign==31 and distance<=50:
        client = Client("/move_base/DWAPlannerROS/")
        client.update_configuration({'max_vel_x': 0.0})
        client.update_configuration({'min_vel_x': 0.0})
        sleep(10)
        client = Client("/move_base/DWAPlannerROS/")
        client.update_configuration({'max_vel_x': 0.26})
        client.update_configuration({'min_vel_x': -0.26})
        exit()
    
    if RoadSign==411 and distance<=50:
        value=100
        costmap_editor.left(value, angle)
        sleep(10)
        value=-1
        costmap_editor.left(value,angle)
        exit()

    if RoadSign==415 and distance<=50:
        value = 100
        costmap_editor.right(value,angle)
        print("продолжаем")
        costmap_editor.front(value,angle)
        sleep(10)
        value=-1
        costmap_editor.right(value,angle)
        costmap_editor.front(value,angle)
        exit()

if __name__ == '__main__':
    costmap_editor = CostmapEditor()
    while not rospy.is_shutdown():
        rospy.Subscriber('RoadSign', Int64MultiArray, callback1, queue_size=10)
        rospy.spin()
