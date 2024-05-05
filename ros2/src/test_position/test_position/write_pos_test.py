import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit import MoveGroupInterface


class MoveItNode(Node):
    def __init__(self):
        super().__init__('ur5_moveit_move')
        self.move_group = MoveGroupInterface("ur_manipulator", "base_link")

    def move_to_position(self, target):
        pose_target = Pose()
        pose_target.orientation.w = 1.0
        pose_target.position.x = target[0]
        pose_target.position.y = target[1]
        pose_target.position.z = target[2]

        self.move_group.moveToPose(pose_target, "tool0", 0.01)


def main(args=None):
    rclpy.init(args=args)
    moveit_node = MoveItNode()
    try:
        target_position = [0, 0, 0]  # XYZ position
        moveit_node.move_to_position(target_position)
    finally:
        moveit_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
