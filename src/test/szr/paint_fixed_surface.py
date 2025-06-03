#!/usr/bin/env python3
# import rospy
# import sys
# import numpy as np
# import math
# from geometry_msgs.msg import Pose
# from tf.transformations import quaternion_from_euler
# from moveit_commander import MoveGroupCommander, RobotCommander, roscpp_initialize, roscpp_shutdown

# class PaintRobot:
#     def __init__(self):
#         roscpp_initialize(sys.argv)
#         rospy.init_node("paint_node", anonymous=True)

#         self.robot = RobotCommander()
#         self.arm_group = MoveGroupCommander("arm")
#         self.gripper_group = MoveGroupCommander("gripper")

#         rospy.sleep(1.0)

#     def open_gripper(self):
#         joint_values = self.gripper_group.get_current_joint_values()
#         joint_values[0] = 0.9
#         self.gripper_group.set_joint_value_target(joint_values)
#         self.gripper_group.go(wait=True)

#     def close_gripper(self):
#         joint_values = self.gripper_group.get_current_joint_values()
#         joint_values[0] = 0.0
#         self.gripper_group.set_joint_value_target(joint_values)
#         self.gripper_group.go(wait=True)

#     def draw_on_fixed_surface(self, T_base_marker, shape='grid', size=0.045, step=0.01):
#         def generate_surface_points(T_base_marker, size=0.1, step=0.01):
#             points = []
#             for x in np.arange(-size/2, size/2, step):
#                 for y in np.arange(-size/2, size/2, step):
#                     pt_local = np.array([x, y, 0, 1.0])
#                     pt_base = T_base_marker @ pt_local
#                     points.append(pt_base[:3])
#             return points

#         def points_to_cartesian_poses(points_base, z_offset=0.005):
#             poses = []
#             for pt in points_base:
#                 pose = Pose()
#                 pose.position.x = pt[0]
#                 pose.position.y = pt[1]
#                 pose.position.z = pt[2] + z_offset
#                 q = quaternion_from_euler(0, math.pi, 0)  # 工具竖直向下
#                 pose.orientation.x = q[0]
#                 pose.orientation.y = q[1]
#                 pose.orientation.z = q[2]
#                 pose.orientation.w = q[3]
#                 poses.append(pose)
#             return poses

#         rospy.loginfo("Generating drawing trajectory on fixed surface...")
#         points = generate_surface_points(T_base_marker, size=size, step=step)
#         poses = points_to_cartesian_poses(points, z_offset=0.003)

#         rospy.loginfo("Executing trajectory, total points: %d" % len(poses))
#         (plan, fraction) = self.arm_group.compute_cartesian_path(
#             poses,
#             eef_step=0.005,
#         )

#         if fraction > 0.9:
#             self.arm_group.execute(plan, wait=True)
#             rospy.loginfo("Drawing complete.")
#             return True
#         else:
#             rospy.logwarn("Trajectory incomplete, only %.2f%% achieved" % (fraction * 100))
#             return False

# if __name__ == "__main__":
#     example = PaintRobot()

#     rospy.loginfo("Moving to home position...")
#     example.arm_group.set_named_target("home")
#     example.arm_group.go(wait=True)

#     # 固定立方体顶面中心位置
#     T_base_marker = np.eye(4)
#     T_base_marker[0, 3] = 0.344
#     T_base_marker[1, 3] = 0.0
#     T_base_marker[2, 3] = 0.025 + 0.049 / 2

#     example.draw_on_fixed_surface(T_base_marker, shape='grid', size=0.045, step=0.01)

#     roscpp_shutdown()

import rospy
import sys
import numpy as np
import math
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
from moveit_commander import MoveGroupCommander, RobotCommander, roscpp_initialize, roscpp_shutdown
class PaintRobot:
    def __init__(self):
        roscpp_initialize(sys.argv)
        rospy.init_node("paint_node", anonymous=True)

        self.robot = RobotCommander()
        self.arm_group = MoveGroupCommander("arm")
        self.gripper_group = MoveGroupCommander("gripper")

        # 设置规划时间与重试次数
        self.arm_group.set_planning_time(10.0)
        self.arm_group.set_num_planning_attempts(5)

        rospy.sleep(1.0)

    def open_gripper(self):
        joint_values = self.gripper_group.get_current_joint_values()
        joint_values[0] = 0.9
        self.gripper_group.set_joint_value_target(joint_values)
        self.gripper_group.go(wait=True)

    def close_gripper(self):
        joint_values = self.gripper_group.get_current_joint_values()
        joint_values[0] = 0.0
        self.gripper_group.set_joint_value_target(joint_values)
        self.gripper_group.go(wait=True)

    def draw_circle_on_surface(self, T_base_marker, radius=0.02, num_points=50):
        def generate_circle_points(center, radius, num_points):
            points = []
            for i in range(num_points):
                theta = 2 * np.pi * i / num_points
                x = center[0] + radius * np.cos(theta)
                y = center[1] + radius * np.sin(theta)
                z = center[2]
                points.append(np.array([x, y, z]))
            return points

        def points_to_cartesian_poses(points_base, z_offset=0.015):  # 抬高一点
            poses = []
            for pt in points_base:
                pose = Pose()
                pose.position.x = pt[0]
                pose.position.y = pt[1]
                pose.position.z = pt[2] + z_offset
                q = quaternion_from_euler(0, math.pi, 0)
                pose.orientation.x = q[0]
                pose.orientation.y = q[1]
                pose.orientation.z = q[2]
                pose.orientation.w = q[3]
                poses.append(pose)
            return poses

        rospy.loginfo("Generating circular drawing trajectory...")
        surface_center = T_base_marker[:3, 3]
        points = generate_circle_points(surface_center, radius, num_points)
        poses = points_to_cartesian_poses(points)

        rospy.loginfo("Executing circular trajectory...")
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()

        (plan, fraction) = self.arm_group.compute_cartesian_path(
            poses,
            eef_step=0.01,  # 不要太小
            jump_threshold=0.0
        )

        rospy.loginfo("Trajectory planned with %.2f%% success" % (fraction * 100))

        if fraction > 0.8:
            self.arm_group.execute(plan, wait=True)
            rospy.loginfo("Circular drawing complete.")
            return True
        else:
            rospy.logwarn("Trajectory incomplete, only %.2f%% achieved" % (fraction * 100))
            return False

if __name__ == "__main__":
    example = PaintRobot()

    rospy.loginfo("Moving to home position...")
    example.arm_group.set_named_target("home")
    example.arm_group.go(wait=True)

    # 固定立方体顶面中心位置
    T_base_marker = np.eye(4)
    T_base_marker[0, 3] = 0.344
    T_base_marker[1, 3] = 0.0
    T_base_marker[2, 3] = 0.025 + 0.049 / 2  # 顶面中心高

    success = example.draw_circle_on_surface(T_base_marker, radius=0.02, num_points=50)

    if not success:
        rospy.logerr("Failed to complete circular drawing.")

    roscpp_shutdown()
