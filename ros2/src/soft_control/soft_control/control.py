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

        self.xdiff = 0
        self.ydiff = 0
        self.zdiff = 0

        self.fdiffs = [0, 0, 0]

        # Create pressure subscription
        self.pressure_sub = self.create_subscription(Float32MultiArray, "/pressures", self.pressure_callback, 10)
        self.ur_control = RTDEControl("192.168.1.101")
        self.ur_receive = RTDEReceive("192.168.1.101")

        home = [-0.5, 0, 0.35, 0, 0, 0]
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
        filter_order = 2
        factor = 5000
        forces = self.ur_receive.getActualTCPForce()[0:3]
        scaled_forces = [f/factor for f in forces]

        fdifftemp = [(filter_order-1)*f/filter_order for f in self.fdiffs]
        for i in range(3):
            fdifftemp[i] += scaled_forces[i]/filter_order

        # self.fdiffs = fdifftemp
        # self.fdiffs = [0, 0, fdifftemp[2]]

        # print(f"{fdifftemp[0]:7.4f} {fdifftemp[1]:7.4f} {fdifftemp[2]:7.4f}")

        position = [-0.5+self.xdiff+self.fdiffs[0], -self.ydiff+self.fdiffs[1], 0.35-self.zdiff+self.fdiffs[2], 0, 0, 0]
        
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