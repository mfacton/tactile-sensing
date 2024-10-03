#!/usr/bin/env python3
import math
import random
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rtde_control import RTDEControlInterface as RTDEControl
from soft_msgs.srv import Calibrate
from std_msgs.msg import Float32MultiArray


def dist(x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    return math.sqrt(dx * dx + dy * dy)

class Grid():
    def __init__(self, width, height, grid_scale, img_scale=1):
        # in meters
        self.width = width
        self.height = height
        
        # size of cell in meters
        # eg 0.001 is 1mm
        self.grid_scale = grid_scale

        # how many cells wide is a pixel
        self.img_scale = img_scale

        self.grid_width = int(self.width/self.grid_scale)
        self.grid_height = int(self.height/self.grid_scale)

        self.img_height = int(self.grid_height/self.img_scale)
        self.img_width = int(self.grid_width/self.img_scale)

        # 0: undiscoverd
        # 1: empty
        # 2: is wall
        self.cells = np.zeros((self.grid_width, self.grid_height), dtype=np.uint8)

        # in bgr
        self.color_map = {
            0: (0, 0, 0),   # Black for 0
            1: (0, 200, 0), # Green for 1
            2: (0, 0, 200)  # Red for 2
        }
    
    # dimentions in cells
    def set_cells(self, posx, posy, rad, state):
        posx += self.width / 2
        posy += self.height / 2

        start_x = int((posx - rad) / self.grid_scale)
        end_x = int((posx + rad) / self.grid_scale)
        start_y = int((posy - rad) / self.grid_scale)
        end_y = int((posy + rad) / self.grid_scale)
        cellx = posx / self.grid_scale
        celly = posy / self.grid_scale
        rad = int(rad / self.grid_scale)

        for x in range(start_x, end_x):
            for y in range(start_y, end_y):
                if dist(x, y, cellx, celly) > rad:
                    continue
                if x < 0 or y < 0 or x >= self.grid_width or y >= self.grid_height:
                    continue 
                if self.cells[x][y] == 2:
                    continue
                
                self.cells[x][y] = state
                
    
    def draw_cells(self):
        image = np.zeros((self.img_height, self.img_width, 3), dtype=np.uint8)
        downsampled = self.cells[::self.img_scale, ::self.img_scale]
        for value, color in self.color_map.items():
            image[downsampled == value] = color
        
        # image = np.transpose(image, (1, 0, 2))
        image = np.flip(image, 1)

        image = np.flip(image, 0)

        cv2.imshow("Map", image)
        cv2.waitKey(1)

class Maze():
    def __init__(self, width, height, grid_scale, img_scale):
        # how many maze cells
        self.width = width
        self.height = height

        # how many meters is a grid cell
        self.grid_scale = grid_scale

        # how many pixels per grid
        # eg 100 pixels
        self.img_scale = img_scale

        # walls
        self.wallH = [[0 for y in range(self.height+1)] for x in range(self.width)]
        self.wallV = [[0 for y in range(self.height)] for x in range(self.width+1)]

        for i in range(self.width):
            self.wallH[i][0] = 2
            self.wallH[i][self.height] = 2
        for i in range(self.height):
            self.wallV[0][i] = 2
            self.wallV[self.width][i] = 2


        self.img_width = (self.width+1)*self.img_scale
        self.img_height = (self.height+1)*self.img_scale

        self.probe_color = (200, 100, 0)
        self.stack_color = (100, 0, 100)

        # in bgr
        self.color_map = {
            0: (50, 50, 0),   # Blue for 0
            1: (0, 100, 0), # Green for 1
            2: (200, 200, 200)  # Red for 2
        }

    def set_wall(self, pos, dir, state):
        pos = (pos[0]+3, pos[1]+3)
        if dir == (1, 0):
            # right, affects vertical segments
            self.wallV[pos[0]+1][pos[1]] = state
        elif dir == (-1, 0):
            # left, affects vertical segments
            self.wallV[pos[0]][pos[1]] = state
        elif dir == (0, 1):
            # up, affects horizontal segments
            self.wallH[pos[0]][pos[1]+1] = state
        else:
            # down, affects horizontal segments
            self.wallH[pos[0]][pos[1]] = state

    def get_wall(self, pos, dir):
        pos = (pos[0]+3, pos[1]+3)
        if dir == (1, 0):
            # right, affects vertical segments
            return self.wallV[pos[0]+1][pos[1]]
        elif dir == (-1, 0):
            # left, affects vertical segments
            return self.wallV[pos[0]][pos[1]]
        elif dir == (0, 1):
            # up, affects horizontal segments
            return self.wallH[pos[0]][pos[1]+1]
        else:
            # down, affects horizontal segments
            return self.wallH[pos[0]][pos[1]]


    # coordinates in meters
    def draw_maze(self, centerx, centery, stack):
        image = np.zeros((self.img_height, self.img_width, 3), dtype=np.uint8)
        
        # horizontal
        for x in range(0, self.width):
            for y in range(0, self.height+1):
                cv2.line(image, (int((x+0.6)*self.img_scale), self.img_height-int((y+0.5)*self.img_scale)), (int((x+1.4)*self.img_scale), self.img_height-int((y+0.5)*self.img_scale)), self.color_map[self.wallH[x][y]], max(self.img_scale//20, 1))
        
        # vertical
        for x in range(0, self.width+1):
            for y in range(0, self.height):
                cv2.line(image, (int((x+0.5)*self.img_scale), self.img_height-int((y+0.6)*self.img_scale)), (int((x+0.5)*self.img_scale), self.img_height-int((y+1.4)*self.img_scale)), self.color_map[self.wallV[x][y]], max(self.img_scale//20, 1))

        for p in stack:
            cv2.circle(image, ((p[0]+4)*self.img_scale, self.img_height-(p[1]+4)*self.img_scale), self.img_scale//10, self.stack_color, -1)
        for i in range(0, len(stack)-1):
            p1 = stack[i]
            p2 = stack[i+1]
            cv2.line(image, ((p1[0]+4)*self.img_scale, self.img_height-(p1[1]+4)*self.img_scale), ((p2[0]+4)*self.img_scale, self.img_height-(p2[1]+4)*self.img_scale), self.stack_color, max(self.img_scale//20, 1))
        cv2.circle(image, (int((centerx/self.grid_scale+4)*self.img_scale), self.img_height-int((centery/self.grid_scale+4)*self.img_scale)), self.img_scale//5, self.probe_color, -1)

        image = np.transpose(image, (1, 0, 2))
        image = np.flip(image, 0)

        cv2.imshow("Maze", image)
        cv2.waitKey(1)


class MapNode(Node):
    def __init__(self):
        # This is the actual name of the node that will be in ros
        super().__init__("maze")

        self.tcp_sub = self.create_subscription(Float32MultiArray, "/tcp", self.tcp_callback, 10)
        self.interpret_sub = self.create_subscription(Float32MultiArray, "/interpret", self.interpret_callback, 10)
        self.client = self.create_client(Calibrate, 'calibrate')
        self.ur_control = RTDEControl("192.168.1.101")

        self.grid = Grid(0.6, 0.6, 0.0007)
        self.maze = Maze(7, 7, 0.08, 100)

        # range x (-0.8, -0.2)
        # range y (-0.3, 0.3)

        self.pos = None
        self.lattice = None

        # map variables
        self.stack = [(0, 0)]
        self.direction = None
        self.exploring = False
        self.backup = False
        self.finished = False

        cv2.namedWindow('Maze', cv2.WINDOW_NORMAL)
        cv2.namedWindow('Map', cv2.WINDOW_NORMAL)
        # do it twice to get rid of glitch
        # for x in range(2):
        self.maze.draw_maze(0, 0, self.stack)
        self.grid.draw_cells()

        # delay
        self.move_robot(0, 0, 0.1, speed=0.2, accel=0.2)
        # for t in range(300):
        #     self.maze.draw_maze(0, 0, self.stack)
        #     self.grid.draw_cells()
        #     time.sleep(0.1)

        self.request_calibration(100)
        time.sleep(1)
        self.move_robot(0, 0, 0, speed=0.2, accel=0.2)

    def move_robot(self, x, y, z, asyn = False, speed=0.04, accel=0.4):
        home = [-0.503+y, -x, z+0.1875, 0, math.pi, 0]
        self.ur_control.moveL(home, speed, accel, asyn)

    def request_calibration(self, measurements):
        request = Calibrate.Request()
        request.measurements = measurements
        self.client.call_async(request)

    def interpret_callback(self, msg: Float32MultiArray):
        idata = msg.data
        self.lattice = idata

    def tcp_callback(self, msg: Float32MultiArray):
        tcp_data = msg.data
        # read robot position and offset for maze
        self.pos = [-tcp_data[1], tcp_data[0]+0.503]

        self.update()

    def update(self):
        if not self.pos or not self.lattice:
            return
        
        ### Map ###
        centerx = self.pos[0] - self.lattice[0]*math.cos(self.lattice[1])
        centery = self.pos[1] - self.lattice[0]*math.sin(self.lattice[1])

        # parameters
        lattice_radius = 0.03 # meters
        spot_radius = 0.002 # meters
        sample_dist = 0.001 # meters
        span = 0.3 # radians +- from theta

        disp_thresh = 0.0015 # meters

        ang_inc = sample_dist/lattice_radius # radians
        
        self.grid.set_cells(centerx, centery, lattice_radius-spot_radius, 1)

        if self.lattice[0] > disp_thresh:
            ang = self.lattice[1]-span
            while ang < self.lattice[1]+span:
                ang += ang_inc
                self.grid.set_cells(centerx+lattice_radius*math.cos(ang), centery+lattice_radius*math.sin(ang), spot_radius, 2)

            # one last time for ending
            self.grid.set_cells(centerx+lattice_radius*math.cos(ang), centery+lattice_radius*math.sin(ang), spot_radius, 2)
        
        self.maze.draw_maze(centerx, centery, self.stack)
        self.grid.draw_cells()
        
        if self.finished:
            return

        ### Maze ###
        if self.ur_control.isSteady():
            self.request_calibration(10)
            if self.exploring:
                # path finished no wall
                new_cell = (self.direction[0]+self.stack[-1][0], self.direction[1]+self.stack[-1][1])
                if self.backup:
                    self.stack.pop()
                    self.backup = False
                else:
                    self.maze.set_wall(self.stack[-1], self.direction, 1)
                    self.stack.append(new_cell)
            elif self.direction:
                # path interrupted cause wall
                self.maze.set_wall(self.stack[-1], self.direction, 2)

            self.exploring = True

            # calculate next move
            directions = [(1, 0), (0, 1), (-1, 0), (0, -1)]
            valid_dirs = []
            for dir in directions:
                new_cell = (self.stack[-1][0] + dir[0], self.stack[-1][1] + dir[1])
                if new_cell[0] < -3 or new_cell[0] > 3 or new_cell[1] < -3 or new_cell[1] > 3:
                    continue
                if self.maze.get_wall(self.stack[-1], dir) == 2 or self.maze.get_wall(self.stack[-1], dir) == 1:
                    continue
                if new_cell in self.stack:
                    continue
                valid_dirs.append(dir)
            if len(valid_dirs) > 0:
                self.direction = valid_dirs[random.randint(0, len(valid_dirs)-1)]
            elif len(self.stack) > 1:
                self.direction = (self.stack[-2][0]-self.stack[-1][0], self.stack[-2][1]-self.stack[-1][1])
                self.backup = True
            else:
                self.get_logger().info("Mapping finished")
                self.finished = True

            # next space
            if not self.finished:
                self.move_cell(self.stack[-1][0] + self.direction[0], self.stack[-1][1] + self.direction[1])
        elif self.exploring and self.lattice[0] > disp_thresh:
            self.exploring = False
            self.ur_control.stopL(1.8)
            self.move_cell(self.stack[-1][0], self.stack[-1][1])

    def move_cell(self, cx, cy):
        self.move_robot(cx*self.maze.grid_scale, cy*self.maze.grid_scale, 0, asyn=True)


def main(args=None):
    rclpy.init(args=args)
    node = MapNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# grid = Grid(0.6, 0.6, 0.001)
# maze = Maze(8, 7, 0.08, 100)
# # maze.direction = (0, 1)
# maze.set_wall((0, 0), (0, 1), 2)
# grid.set_cells(0, 0, 0.3, 1)
# while True:
#     maze.draw_maze(0, 0)
#     grid.draw_cells()

#     time.sleep(0.1)