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

        self.radius = 0.2
        self.angle = 0

        self.xdiff = self.radius*math.cos(self.angle)
        self.ydiff = self.radius*math.sin(self.angle)
        self.zdiff = 0

        # Create pressure subscription
        self.pressure_sub = self.create_subscription(Float32MultiArray, "/pressure", self.pressure_callback, 10)
        self.ur_control = RTDEControl("192.168.1.101")

        # home = [-0.5+self.xdiff, -self.ydiff, 0.35, 0, 0, 0]
        home = [-0.5, 0.2-self.ydiff, 0.45+self.xdiff, 0, -math.pi/2, 0]
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
        
        avg /= 6
        # self.zdiff = (pressures[6]-avg) / 750

        #orthogonal tangent vector
        thetax = -math.sin(self.angle)
        thetay = math.cos(self.angle)

        #increase angle based on dot product
        dotp = thetax*diffx+thetay*diffy
        angle_vel = dotp/80000

        if angle_vel > 0.012:
            angle_vel = 0.012
        self.angle += angle_vel

        
    def update(self):
        #increment angle based on velocity at 125hz
        self.xdiff = self.radius*math.cos(self.angle)
        self.ydiff = self.radius*math.sin(self.angle)

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
