import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import time

class CommandPublisher(Node):
    def __init__(self):
        super().__init__('command_publisher')
        self.publisher = self.create_publisher(PoseStamped, '/forward_position_controller/commands', 10)

    def publish_command(self):
        msg = PoseStamped()
        msg.header.frame_id = ''  # Frame of reference for the pose
        # Set the desired position and orientation
        # [INFO] [1714849390.937519967] [tf_subscriber]: Current Position: [-0.49088613244258184, 0.10880145197764154, 0.37482623390297765]
        # [INFO] [1714849390.938116668] [tf_subscriber]: Current Orientation: [-0.04526481894039408, -0.04281448841406358, -0.4978362267918881, 0.8650301191529409]

        msg.pose.position.x = -0.4909
        msg.pose.position.y = 0.1088
        msg.pose.position.z = 0.3048
        msg.pose.orientation.x = -0.04526481894039408
        msg.pose.orientation.y = -0.04526481894039408
        msg.pose.orientation.z = -0.4978362267918881
        msg.pose.orientation.w = 0.8650301191529409  # Identity quaternion for orientation if 1.0
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    command_publisher = CommandPublisher()
    command_publisher.publish_command()  # Publish a single command
    time.sleep(1)  # Delay for 1 second to allow publishing
    rclpy.spin_once(command_publisher)  # Process any pending callbacks
    command_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()