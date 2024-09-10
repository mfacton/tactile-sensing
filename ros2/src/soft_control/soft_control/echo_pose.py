#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from rtde_receive import RTDEReceiveInterface as RTDEReceive
from std_msgs.msg import Float32MultiArray


class ControlNode(Node):
    def __init__(self):
        # This is the actual name of the node that will be in ros
        super().__init__("pose")

        # Create pose publisher
        self.tcp_pub = self.create_publisher(Float32MultiArray, "/tcp", 10)
        self.joint_pub = self.create_publisher(Float32MultiArray, "/joint", 10)

        self.ur_receive = RTDEReceive("192.168.1.101")

        self.get_logger().info("Started publishing pose data")

        
    def run(self):
        while True:
            tcp_data = Float32MultiArray()
            joint_data = Float32MultiArray()

            tcp_data.data = self.ur_receive.getActualTCPPose()
            joint_data.data = self.ur_receive.getActualQ()

            self.tcp_pub.publish(tcp_data)
            self.joint_pub.publish(joint_data)

            time.sleep(0.001)

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
