import rclpy
from rclpy.node import Node
import tf2_ros
import geometry_msgs.msg

class TFSubscriber(Node):
    def __init__(self):
        super().__init__('tf_subscriber')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.process_tf_data)

    def process_tf_data(self):
        try:
            # Lookup the transform from the base frame to the end effector frame
            # can use world or baselink
            transform = self.tf_buffer.lookup_transform(
                'base_link', 'wrist_3_link', rclpy.time.Time())
            # Extract the position from the transform
            current_position = transform.transform.translation
            current_orientation = transform.transform.rotation
            self.get_logger().info(
                f'Current Position: [{current_position.x}, {current_position.y}, {current_position.z}]'
            )
            self.get_logger().info(
                f'Current Orientation: [{current_orientation.x}, {current_orientation.y}, {current_orientation.z}, {current_orientation.w}]'
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'Failed to lookup transform: {e}')

def main(args=None):
    rclpy.init(args=args)
    tf_subscriber = TFSubscriber()
    rclpy.spin(tf_subscriber)
    tf_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
