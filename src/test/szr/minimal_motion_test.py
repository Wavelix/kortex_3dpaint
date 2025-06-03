#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
from math import pi
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("minimal_motion_test", anonymous=True)

    robot = moveit_commander.RobotCommander()
    arm_group = moveit_commander.MoveGroupCommander("arm")

    rospy.loginfo("Moving to home position...")
    arm_group.set_named_target("home")
    arm_group.go(wait=True)

    rospy.sleep(1.0)

    # 获取当前位置
    current_pose = arm_group.get_current_pose().pose

    # 向下移动 5cm
    target_pose = Pose()
    target_pose.position.x = current_pose.position.x
    target_pose.position.y = current_pose.position.y
    target_pose.position.z = current_pose.position.z - 0.05  # 向下

    # 姿态保持不变
    target_pose.orientation = current_pose.orientation

    arm_group.set_pose_target(target_pose)

    rospy.loginfo("Moving downward 5cm...")
    success = arm_group.go(wait=True)

    if success:
        rospy.loginfo("Downward motion success.")
    else:
        rospy.logwarn("Downward motion failed.")

    rospy.sleep(1.0)

    rospy.loginfo("Returning to home position...")
    arm_group.set_named_target("home")
    arm_group.go(wait=True)

    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    main()
