#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np

class CostmapEditor:
    def __init__(self):
        rospy.init_node('costmap_editor', anonymous=True)
        rospy.Subscriber('/map', OccupancyGrid, self.costmap_callback)
        self.map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)
        self.map = None
        self.resolution = 0.0

    def costmap_callback(self, msg):
        self.map = msg
        self.resolution = msg.info.resolution

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

    def edit_map(self, min_x, min_y, max_x, max_y, value):
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

if __name__ == '__main__':
    costmap_editor = CostmapEditor()

    while not rospy.is_shutdown():
        min_x = float(input("Enter min x coordinate:"))
        min_y = float(input("Enter min y coordinate:"))
        max_x = float(input("Enter max x coordinate:"))
        max_y = float(input("Enter max y coordinate:"))
        value = int(input("Enter new cell value (-1 for unknown, 0 for free, 100 for occupied):"))
        costmap_editor.edit_map(min_x, min_y, max_x, max_y, value)
