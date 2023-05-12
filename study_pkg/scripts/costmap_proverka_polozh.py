#!/usr/bin/env python3

# msg1 - дорожный знак
# msg2 - расстояние до дорожного знака в сантиметрах
# msg1=31 - стоп
# msg1=412 - только направо
# msg1=411 - только прямо
# msg1=413 - только налево
# msg1=414 - прямо и направо
# msg1=415 - прямо и налево

import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
from std_msgs.msg import Int64
from std_msgs.msg import Int64MultiArray
from dynamic_reconfigure.client import Client
from time import sleep

class CostmapEditor:
    def __init__(self):
        rospy.init_node('costmap_editor', anonymous=True)
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

    def edit_map411(self, value):
        if self.robot_pose is None:
            rospy.logwarn('Robot pose is not available yet.')
            return

        x = self.robot_pose.position.x
        y = self.robot_pose.position.y

        min_x = x + 0.5
        min_y = y + 0.3
        max_x = x + 1
        max_y = y + 1

        coords = self.indices_to_coords(min_x, min_y, max_x, max_y)
        indices = self.coords_to_indices(coords)

        new_map = OccupancyGrid()
        new_map.header.stamp = rospy.Time.now()
        new_map.header.frame_id = self.map.header.frame_id
        new_map.info = self.map.info
        new_map.data = list(self.map.data)
        for index in indices:
            new_map.data[index] = value
        self.map_pub.publish(new_map)

    def edit_map412(self, value):
        if self.robot_pose is None:
            rospy.logwarn('Robot pose is not available yet.')
            return

        x = self.robot_pose.position.x
        y = self.robot_pose.position.y

        min_x = x 
        min_y = y 
        max_x = x + 0.5
        max_y = y + 0.5

        coords = self.indices_to_coords(min_x, min_y, max_x, max_y)
        indices = self.coords_to_indices(coords)

        new_map = OccupancyGrid()
        new_map.header.stamp = rospy.Time.now()
        new_map.header.frame_id = self.map.header.frame_id
        new_map.info = self.map.info
        new_map.data = list(self.map.data)
        for index in indices:
            new_map.data[index] = value
        self.map_pub.publish(new_map)

    def edit_map413(self, value):
        if self.robot_pose is None:
            rospy.logwarn('Robot pose is not available yet.')
            return

        x = self.robot_pose.position.x
        y = self.robot_pose.position.y

        min_x = x 
        min_y = y 
        max_x = x + 0.5
        max_y = y + 0.5

        coords = self.indices_to_coords(min_x, min_y, max_x, max_y)
        indices = self.coords_to_indices(coords)

        new_map = OccupancyGrid()
        new_map.header.stamp = rospy.Time.now()
        new_map.header.frame_id = self.map.header.frame_id
        new_map.info = self.map.info
        new_map.data = list(self.map.data)
        for index in indices:
            new_map.data[index] = value
        self.map_pub.publish(new_map)

    def edit_map414(self, value):
        if self.robot_pose is None:
            rospy.logwarn('Robot pose is not available yet.')
            return

        x = self.robot_pose.position.x
        y = self.robot_pose.position.y

        min_x = x 
        min_y = y 
        max_x = x + 0.5
        max_y = y + 0.5

        coords = self.indices_to_coords(min_x, min_y, max_x, max_y)
        indices = self.coords_to_indices(coords)

        new_map = OccupancyGrid()
        new_map.header.stamp = rospy.Time.now()
        new_map.header.frame_id = self.map.header.frame_id
        new_map.info = self.map.info
        new_map.data = list(self.map.data)
        for index in indices:
            new_map.data[index] = value
        self.map_pub.publish(new_map)

    def edit_map415(self, value):
        if self.robot_pose is None:
            rospy.logwarn('Robot pose is not available yet.')
            return

        x = self.robot_pose.position.x
        y = self.robot_pose.position.y

        min_x = x 
        min_y = y 
        max_x = x + 0.5
        max_y = y + 0.5

        coords = self.indices_to_coords(min_x, min_y, max_x, max_y)
        indices = self.coords_to_indices(coords)

        new_map = OccupancyGrid()
        new_map.header.stamp = rospy.Time.now()
        new_map.header.frame_id = self.map.header.frame_id
        new_map.info = self.map.info
        new_map.data = list(self.map.data)
        for index in indices:
            new_map.data[index] = value
        self.map_pub.publish(new_map)
 

def callback(msg):
    rospy.loginfo("RoadSign %d; Lenght %d", msg.data[0], msg.data[1])
    if msg.data[0]==31 and msg.data[1]<=50:
        client = Client("/move_base/DWAPlannerROS/")
        client.update_configuration({'max_vel_x': 0.0})
        client.update_configuration({'min_vel_x': 0.0})
        exit()
    else:
        client = Client("/move_base/DWAPlannerROS/")
        client.update_configuration({'max_vel_x': 0.26})
        client.update_configuration({'min_vel_x': -0.26})
    
    value = 100

    if msg.data[0]==411 and msg.data[1]<=50:
        costmap_editor.edit_map411(value)
        sleep(90)
        value=-1
        costmap_editor.edit_map411(value)
        exit()
    elif msg.data[0]==412 and msg.data[1]<=50:
        costmap_editor.edit_map412(value)
        sleep(90)
        value=-1
        costmap_editor.edit_map412(value)
        exit()
    elif msg.data[0]==413 and msg.data[1]<=50:
        costmap_editor.edit_map413(value)
        sleep(90)
        value=-1
        costmap_editor.edit_map413(value)
        exit()
    elif msg.data[0]==414 and msg.data[1]<=50:
        costmap_editor.edit_map414(value)
        sleep(90)
        value=-1
        costmap_editor.edit_map414(value)
        exit()
    elif msg.data[0]==415 and msg.data[1]<=50:
        costmap_editor.edit_map415(value)
        sleep(90)
        value=-1
        costmap_editor.edit_map415(value)
        exit()

if __name__ == '__main__':
    costmap_editor = CostmapEditor()
    while not rospy.is_shutdown():
        rospy.Subscriber('RoadSign', Int64MultiArray, callback, queue_size=10)
        rospy.spin()