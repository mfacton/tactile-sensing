#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class InterpretNode(Node):
    def __init__(self):
        # This is the actual name of the node that will be in ros
        super().__init__("interpret")

        self.xdiff = 0
        self.ydiff = 0
        self.zdiff = 0

        self.pressure_sub = self.create_subscription(Float32MultiArray, "/pressure", self.pressure_callback, 10)
        self.interpret_pub = self.create_publisher(Float32MultiArray, "/interpret", 10)
    
    def pressure_callback(self, msg: Float32MultiArray):
        # Called on every pressure update
        pressures = msg.data

        scale_plane = 1/50000
        scale_z = 1/1000000

        distx = 0
        disty = 0

        avg = 0
        for i in range(6):
            avg += pressures[i]
            angle = i*math.pi/3 + math.pi/2
            distx += math.cos(angle)*pressures[i]
            disty += math.sin(angle)*pressures[i]

        avg /= 6

        distx *= scale_plane
        disty *= scale_plane
        distz = (avg-pressures[6]) * scale_z

        r = math.sqrt(distx * distx + disty * disty)
        theta = math.atan2(disty, distx)

        idata = Float32MultiArray()
        idata.data = [r, theta, distz]

        self.interpret_pub.publish(idata)

def main(args=None):
    rclpy.init(args=args)
    node = InterpretNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
