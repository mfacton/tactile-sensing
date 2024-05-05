import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg


def move_to_position(target):
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("ur5_moveit_move", anonymous=True)

    group = moveit_commander.MoveGroupCommander("manipulator")
    group.set_pose_reference_frame("base_link")

    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.w = 1.0
    pose_target.position.x = target[0]
    pose_target.position.y = target[1]
    pose_target.position.z = target[2]
    group.set_pose_target(pose_target)

    plan = group.go(wait=True)
    group.stop()

    group.clear_pose_targets()

    current_pose = group.get_current_pose().pose
    print("Current Pose:", current_pose)
    if plan:
        print("Movement successful")
    else:
        print("Movement failed")


if __name__ == "__main__":
    target_position = [0, 0, 0, 0, 0, 0]
    move_to_position(target_position)
