#!/usr/bin/env python3
import math
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rtde_control import RTDEControlInterface as RTDEControl
from std_msgs.msg import Float32MultiArray

# byte structure
# 0: undiscoverd
# 1: empty
# 2: is wall

# in meters
length = 0.6
width = 0.6

# size of cell in meters
# eg 0.001 is 1mm
grid_scale = 0.001
grid_length = int(length/grid_scale)
grid_width = int(width/grid_scale)

# how many cells wide is a pixel
img_scale = 1
img_height = grid_length//img_scale
img_width = grid_width//img_scale

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

# dimentions in cells
def set_cells(cellx, celly, rad, state):
    cellx = int(cellx)
    celly = int(celly)
    rad = int(rad)
    start_x = cellx - rad
    end_x = cellx + rad
    start_y = celly - rad
    end_y = celly + rad

    start_x = max(0, start_x)
    end_x = min(grid_width-1, end_x)
    start_y = max(0, start_y)
    end_y = min(grid_length-1, end_y)

    for x in range(start_x, end_x):
        for y in range(start_y, end_y):
            if dist(x, y, cellx, celly) <= rad:
                global cells
                cells[x][y] = state

def draw_cells():
    image = np.zeros((img_width, img_height, 3), dtype=np.uint8)
    downsampled = cells[::img_scale, ::img_scale]
    for value, color in color_map.items():
        image[downsampled == value] = color
    
    cv2.imshow("Map", image)
    cv2.waitKey(1)

def draw_maze():
    image = np.zeros((img_width, img_height, 3), dtype=np.uint8)
    # TODO
    
    cv2.imshow("Maze", image)
    cv2.waitKey(1)

class MapNode(Node):
    def __init__(self):
        # This is the actual name of the node that will be in ros
        super().__init__("maze")

        self.tcp_sub = self.create_subscription(Float32MultiArray, "/tcp", self.tcp_callback, 10)
        self.interpret_sub = self.create_subscription(Float32MultiArray, "/interpret", self.interpret_callback, 10)
        self.ur_control = RTDEControl("192.168.1.101")

        # range x (-0.8, -0.2)
        # range y (-0.3, 0.3)
        self.minX = -0.8
        self.minY = -0.3

        self.pos = None
        self.lattice = None

        self.x = 0
        self.run = True
        self.homing = False

        # home = [-0.5, 0, 0.5, 0, math.pi, 0]
        # self.ur_control.moveL(home, 0.2, 0.1)
        start = [-0.5, 0, 0.187, 0, math.pi, 0]
        self.ur_control.moveL(start, 0.2, 0.1)

        # start
        pos = [-0.5, 0.03, 0.187, 0, math.pi, 0]
        self.ur_control.moveL(pos, 0.2, 0.1, asynchronous = True)

    
    def tcp_callback(self, msg: Float32MultiArray):
        tcp_data = msg.data
        # read robot position and offset for maze
        self.pos = [tcp_data[0]-self.minX, tcp_data[1]-self.minY]
    
    def interpret_callback(self, msg: Float32MultiArray):
        idata = msg.data
        self.lattice = idata

        if not self.pos or not self.lattice:
            return
        
        self.parse_data()

    def parse_data(self):
        centerx = self.pos[0] - self.lattice[0]*math.cos(self.lattice[1])
        centery = self.pos[1] - self.lattice[0]*math.sin(self.lattice[1])
        cx = centerx/grid_scale
        cy = centery/grid_scale

        # lattice_radius = 0.032 # meters
        lattice_radius = 0.03
        lattice_cradius = lattice_radius/grid_scale
        
        set_cells(cx, cy, lattice_cradius*0.75, 1)

        if self.lattice[0] > 0.002:
            print("bob")
            self.ur_control.stopL(2)
            start = [-0.5, 0, 0.187, 0, math.pi, 0]
            self.ur_control.moveL(start, 0.2, 0.1)

            # parameters
            spot_radius = 0.002 # meters
            sample_dist = 0.001 # meters
            span = 0.3 # radians +- from theta

            # precalcs
            spot_cradius = spot_radius/grid_scale
            ang_inc = sample_dist/lattice_radius # radians

            ang = self.lattice[1]-span
            while ang < self.lattice[1]+span:
                ang += ang_inc
                set_cells(cx+lattice_cradius*math.cos(ang), cy+lattice_cradius*math.sin(ang), spot_cradius, 2)

            # one last time for ending
            set_cells(cx+lattice_cradius*math.cos(ang), cy+lattice_cradius*math.sin(ang), spot_cradius, 2)

        draw_cells()
    

def main(args=None):
    rclpy.init(args=args)
    node = MapNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
