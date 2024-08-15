#!/usr/bin/env python3
import math

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from rclpy.node import Node
from rtde_control import RTDEControlInterface as RTDEControl
from std_msgs.msg import Float32MultiArray


class ControlNode(Node):
    def __init__(self):
        # This is the actual name of the node that will be in ros
        super().__init__("control")
        self.running = True

        # Create pressure subscription
        self.pressure_sub = self.create_subscription(Float32MultiArray, "/pressures", self.pressure_callback, 10)
        self.joint_sub = self.create_subscription(Float32MultiArray, "/joint", self.joint_callback, 10)
        self.ur_control = RTDEControl("192.168.1.101")

        self.rotation = 0
        self.pressures = [0 for i in range(6)]

        home_clear = [-0.3, 0.3, 0.3, -math.pi*math.sqrt(2)/2, math.pi*math.sqrt(2)/2, 0]
        self.ur_control.moveL(home_clear, 0.2, 0.1)
        self.get_logger().info("Moved to clear position")

        home_clear = [-0.3, 0.3, 0.095, -math.pi*math.sqrt(2)/2, math.pi*math.sqrt(2)/2, 0]
        self.ur_control.moveL(home_clear, 0.2, 0.1)
        self.get_logger().info("Moved to ready position")

        self.init_plotter()
    
    def init_plotter(self):
        plt.ion()

        fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
        self.lines = []

        colors = ['b', 'g', 'r', 'c', 'm', 'y']
        for i in range(6):
            ln, = ax.plot([], [], color=colors[i], marker = 'o', linestyle='', markersize=5)
            self.lines.append(ln)

        # theta limits
        ax.set_xlim(0, 2 * np.pi)
        # r limits
        ax.set_ylim(10180, 10200)

        self.draw_timer = self.create_timer(0.1, self.update_plot)
        self.get_logger().info("Started draw timer")
    
    def push_plot(self, line, theta, r):
        self.lines[line].set_data(np.append(self.lines[line].get_xdata(), theta), np.append(self.lines[line].get_ydata(), r))
    
    def update_plot(self):
        plt.gcf().canvas.draw_idle()
        plt.gcf().canvas.flush_events()

    def pressure_callback(self, msg: Float32MultiArray):
        pressures = msg.data
        
        filter = 10
        for i in range(6):
            self.pressures[i] *= (filter-1)/(filter)
            self.pressures[i] += pressures[i]/filter
    
    def joint_callback(self, msg: Float32MultiArray):
        joints = msg.data

        self.rotation = joints[5]

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
