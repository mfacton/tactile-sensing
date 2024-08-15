#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rtde_control import RTDEControlInterface as RTDEControl
from std_msgs.msg import Float32MultiArray


class ControlNode(Node):
    def __init__(self):
        # This is the actual name of the node that will be in ros
        super().__init__("control")

        self.xdiff = 0
        self.ydiff = 0
        self.zdiff = 0

        # Create pressure subscription
        self.pressure_sub = self.create_subscription(Float32MultiArray, "/pressure", self.pressure_callback, 10)
        self.ur_control = RTDEControl("192.168.1.101")

        # home = [-0.5, 0, 0.35, 0, 0, 0]
        home = [-0.5, 0.2, 0.45, 0, -math.pi/2, 0]
        self.ur_control.moveL(home, 0.2, 0.1)
    
    def pressure_callback(self, msg: Float32MultiArray):
        # Called on every pressure update
        pressures = msg.data

        diffx = 0
        diffy = 0

        avg = 0
        for i in range(6):
            avg += pressures[i]
            angle = i*math.pi/3 + math.pi/2
            diffx += math.cos(angle)*pressures[i]
            diffy += math.sin(angle)*pressures[i]

            self.xdiff = diffx / 1500
            self.ydiff = diffy / 1500
        
        avg /= 6
        self.zdiff = (pressures[6]-avg) / 750
        
        
    def update(self):
        # position = [-0.5+self.xdiff, -self.ydiff, 0.35-self.zdiff, 0, 0, 0]
        position = [-0.5+self.zdiff, 0.2-self.ydiff, 0.45+self.xdiff, 0, -math.pi/2, 0]
        
        self.ur_control.servoL(position, 0, 0, 0.008, 0.15, 100)

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    while True:
        node.update()
        rclpy.spin_once(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
