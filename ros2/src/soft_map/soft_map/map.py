#!/usr/bin/env python3
import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

# byte structure
# 0: undiscoverd
# 1: empty
# 2: is wall

# in meters
length = 1
width = 1

# size of cell in meters
# eg 0.001 is 1mm
grid_scale = 0.001
grid_length = int(length/grid_scale)
grid_width = int(width/grid_scale)

# how many cells wide is a pixel
img_scale = 1
img_height = grid_length//img_scale
img_width = grid_width//img_scale

# robot coordinates at cell (0, 0)
offset_x = -0.5
offset_y = -0.5

cells = np.zeros((grid_width, grid_length), dtype=np.uint8)

# in bgr
color_map = {
    0: (0, 0, 0),   # Black for 0
    1: (0, 200, 0), # Green for 1
    2: (0, 0, 200)  # Red for 2
}

def dist(x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    return math.sqrt(dx * dx + dy * dy)

# dimentions all in terms of cells
def set_explored(cell_x, cell_y, rad):
    start_x = cell_x - rad
    end_x = cell_x + rad
    start_y = cell_y - rad
    end_y = cell_y + rad

    start_x = max(0, start_x)
    end_x = min(grid_width-1, end_x)
    start_y = max(0, start_y)
    end_y = min(grid_length-1, end_y)

    for x in range(start_x, end_x):
        for y in range(start_y, end_y):
            if dist(x, y, cell_x, cell_y) <= rad:
                global cells
                cells[x][y] = 1

def draw_cells():
    image = np.zeros((img_width, img_height, 3), dtype=np.uint8)
    downsampled = cells[::img_scale, ::img_scale]
    for value, color in color_map.items():
        image[downsampled == value] = color
    
    cv2.imshow("Map", image)
    cv2.waitKey(1)

class MapNode(Node):
    def __init__(self):
        # This is the actual name of the node that will be in ros
        super().__init__("map")

        self.tcp_sub = self.create_subscription(Float32MultiArray, "/tcp", self.tcp_callback, 10)
        self.interpret_sub = self.create_subscription(Float32MultiArray, "/interpret", self.interpret_callback, 10)

        self.center_x = 0.5
        self.center_y = 0.5

        self.r = 0
        self.disp = 0
    
    def interpret_callback(self, msg: Float32MultiArray):
        # Called on every interpret update
        idata = msg.data

        self.r = idata[0]

        # x = idata[0]*math.cos(idata[1])
        # y = idata[0]*math.sin(idata[1])

        # x *= 5000
        # y *= 5000
        # set_explored(500+int(x), 500+int(y), 10)
        # draw_cells()
    
    def tcp_callback(self, msg: Float32MultiArray):
        tcp_data = msg.data
        self.disp = dist(tcp_data[0], tcp_data[1], self.center_x, self.center_y)

def main(args=None):
    rclpy.init(args=args)
    node = MapNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# test_x = 500
# test_y = 500
# vel_x = 0
# vel_y = 0
# while True:
#     set_explored(int(test_x), int(test_y), 10)
#     draw_cells()
#     vel_x += np.random.normal(loc=0, scale=0.3)
#     vel_y += np.random.normal(loc=0, scale=0.3)
#     vel_x -= 0.0002*(test_x-500)
#     vel_y -= 0.0002*(test_y-500)
#     vel_x *= 0.995
#     vel_y *= 0.995
#     test_x += vel_x
#     test_y += vel_y
#     time.sleep(0.003)