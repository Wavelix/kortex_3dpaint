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

import sys
import time
import math
import rospy
import moveit_commander
import numpy as np
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import RobotTrajectory

class MoveItArm(object):
    def __init__(self):
        super(MoveItArm, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('ik_circle_paint_node')

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.arm_group = moveit_commander.MoveGroupCommander("arm")
        self.gripper_group = moveit_commander.MoveGroupCommander("gripper")

        self.arm_group.set_planning_time(10.0)
        self.arm_group.set_num_planning_attempts(5)
        self.arm_group.set_max_velocity_scaling_factor(0.3)
        self.arm_group.set_max_acceleration_scaling_factor(0.3)

        rospy.sleep(1.0)

    def reach_named_position(self, target):
        self.arm_group.set_named_target(target)
        return self.arm_group.go(wait=True)

    def compute_ik(self, pose):
        self.arm_group.set_pose_target(pose)
        plan = self.arm_group.plan()
        if plan and hasattr(plan[1], 'joint_trajectory') and plan[1].joint_trajectory.points:
            return plan[1].joint_trajectory.points[-1].positions
        else:
            rospy.logwarn("Inverse kinematics failed for pose: %s", pose)
            return None

    def generate_joint_trajectory_on_circle(self, center, radius=0.02, num_points=50, z_offset=0.015):
        joint_trajectory = []
        for i in range(num_points):
            theta = 2 * np.pi * i / num_points
            x = center[0] + radius * np.cos(theta)
            y = center[1] + radius * np.sin(theta)
            z = center[2] + z_offset

            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = z
            q = quaternion_from_euler(0, math.pi, 0)
            pose.orientation.x = q[0]
            pose.orientation.y = q[1]
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]

            joint_angles = self.compute_ik(pose)
            if joint_angles:
                joint_trajectory.append(joint_angles)
                rospy.loginfo("Point %d/%d completed", len(joint_trajectory), num_points)
            else:
                rospy.logwarn("Skipping point %d due to IK failure", i + 1)
        return joint_trajectory
    
    def execute_joint_trajectory_batch(self, joint_traj):
        joint_names = self.arm_group.get_active_joints()
    
        trajectory = RobotTrajectory()
        trajectory.joint_trajectory.joint_names = joint_names
    
        points = []
        for i, joints in enumerate(joint_traj):
            point = JointTrajectoryPoint()
            point.positions = joints
            point.time_from_start = rospy.Duration(0)  # 先不设时间
            points.append(point)
    
        trajectory.joint_trajectory.points = points
    
        # 时间参数化：让轨迹有速度/加速度信息
        iptp = IterativeParabolicTimeParameterization()
        success = iptp.compute_time_stamps(trajectory)

    if not success:
        rospy.logerr("Failed to parameterize time for trajectory.")
        return False

    rospy.loginfo("Executing trajectory with %d points", len(points))
    return self.arm_group.execute(trajectory, wait=True)

    def draw_circle_with_ik(self, T_base_marker, radius=0.02, num_points=50):
        center = T_base_marker[:3, 3]
        rospy.loginfo("Generating joint-space trajectory for circle...")
        joint_traj = self.generate_joint_trajectory_on_circle(center, radius, num_points)
        rospy.loginfo("Executing joint-space circular motion...")
        return self.execute_joint_trajectory_batch(joint_traj)

def main():
    robot = MoveItArm()

    rospy.loginfo("Moving to home position...")
    success = robot.reach_named_position("home")
    if not success:
        rospy.logerr("Failed to reach home position.")
        return

    T_base_marker = np.eye(4)
    T_base_marker[0, 3] = 0.344
    T_base_marker[1, 3] = 0.0
    T_base_marker[2, 3] = 0.025 + 0.049 / 2

    rospy.loginfo("Starting circular drawing...")
    success = robot.draw_circle_with_ik(T_base_marker, radius=0.02, num_points=50)
    if not success:
        rospy.logerr("Failed to draw circle with IK.")

if __name__ == '__main__':
    main()
