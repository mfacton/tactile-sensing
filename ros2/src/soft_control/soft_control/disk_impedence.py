#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
from std_msgs.msg import Float32MultiArray


class ControlNode(Node):
    def __init__(self):
        # This is the actual name of the node that will be in ros
        super().__init__("control")

        self.radius = 0.2

        self.px = 0
        self.py = 0
        self.pz = 0

        self.begin = False

        # Create pressure subscription
        self.pressure_sub = self.create_subscription(Float32MultiArray, "/pressures", self.pressure_callback, 10)
        self.ur_control = RTDEControl("192.168.1.101")

        # home = [-0.5, 0, 0.35, 0, 0, 0]
        home = [-0.5, 0.2, 0.45, 0, -math.pi/2, 0]
        self.ur_control.moveL(home, 0.2, 0.1)
        self.begin = True
    
    def pressure_callback(self, msg: Float32MultiArray):
        # Called on every pressure update
        if not self.begin:
            return
        
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

        px = self.px + diffx/70000
        py = self.py + diffy/70000
        
        # constrain to circle if necessary
        if math.sqrt(px*px + py*py) > self.radius:
            angle = math.atan2(py, px)
            px = self.radius*math.cos(angle)
            py = self.radius*math.sin(angle)

        self.px = px
        self.py = py

        
    def update(self):
        # position = [-0.5+self.px, -self.py, 0.35-self.pz, 0, 0, 0]
        position = [-0.5+self.pz, 0.2-self.py, 0.45+self.px, 0, -math.pi/2, 0]
        
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
