#!/usr/bin/env python3
import math

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from rclpy.node import Node
from rtde_control import RTDEControlInterface as RTDEControl
from std_msgs.msg import Float32MultiArray


def dist(x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    return math.sqrt(dx * dx + dy * dy)

line = None
def init_plotter():
    plt.ion()

    fig, ax = plt.subplots()#subplot_kw={'projection': 'polar'})

    global line
    line, = ax.plot([], [], color='b', marker = 'o', linestyle='', markersize=5)

    ax.set_xlim(0, 0.05) # r
    ax.set_ylim(0, 0.01) # dist

def push_plot(x, y):
    line.set_data(np.append(line.get_xdata(), x), np.append(line.get_ydata(), y))

def update_plot():
    plt.gcf().canvas.draw_idle()
    plt.gcf().canvas.flush_events()

class CharacterizeNode(Node):
    def __init__(self):
        # This is the actual name of the node that will be in ros
        super().__init__("characterize")

        # Create subscriptions
        self.interpret_sub = self.create_subscription(Float32MultiArray, "/interpret", self.interpret_callback, 10)
        self.tcp_sub = self.create_subscription(Float32MultiArray, "/tcp", self.tcp_callback, 10)

        self.center_x = 0
        self.center_y = 0

        self.r = 0
        self.dist = 0

        init_plotter()

        # home_clear = [-0.3, 0.3, 0.3, -math.pi*math.sqrt(2)/2, math.pi*math.sqrt(2)/2, 0]
        # self.ur_control.moveL(home_clear, 0.2, 0.1)
        # self.get_logger().info("Moved to clear position")

        # home_clear = [-0.3, 0.3, 0.095, -math.pi*math.sqrt(2)/2, math.pi*math.sqrt(2)/2, 0]
        # self.ur_control.moveL(home_clear, 0.2, 0.1)
        # self.get_logger().info("Moved to ready position")


    def interpret_callback(self, msg: Float32MultiArray):
        idata = msg.data
        self.r = idata[0]
        push_plot(self.r, self.dist)
        update_plot()
        print(self.r, self.dist)
    
    def tcp_callback(self, msg: Float32MultiArray):
        tcp_data = msg.data
        self.dist = dist(tcp_data[0], tcp_data[1], self.center_x, self.center_y)

def main(args=None):
    rclpy.init(args=args)
    node = CharacterizeNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
