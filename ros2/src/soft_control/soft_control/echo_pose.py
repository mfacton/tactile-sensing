#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rtde_receive import RTDEReceiveInterface as RTDEReceive
from std_msgs.msg import Float32MultiArray


class ControlNode(Node):
    def __init__(self):
        # This is the actual name of the node that will be in ros
        super().__init__("pose")

        # Create pose publisher
        self.pose_pub = self.create_publisher(Float32MultiArray, "/pose", 10)
        self.ur_receive = RTDEReceive("192.168.1.101")

        self.get_logger().info("Started publishing TCP poses")

        
    def run(self):
        while True:
            pose_data = Float32MultiArray()

            pose_data.data = self.ur_receive.getActualTCPPose()

            self.pose_pub.publish(pose_data)

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
