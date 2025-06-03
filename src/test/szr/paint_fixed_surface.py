#!/usr/bin/env python3
# import sys
# import time
# import math
# import rospy
# import moveit_commander
# import numpy as np
# from geometry_msgs.msg import Pose
# from tf.transformations import quaternion_from_euler


# class MoveItArm(object):
#     def __init__(self):
#         super(MoveItArm, self).__init__()
#         moveit_commander.roscpp_initialize(sys.argv)
#         rospy.init_node('ik_circle_paint_node')

#         self.robot = moveit_commander.RobotCommander()
#         self.scene = moveit_commander.PlanningSceneInterface()
#         self.arm_group = moveit_commander.MoveGroupCommander("arm")
#         self.gripper_group = moveit_commander.MoveGroupCommander("gripper")

#         self.arm_group.set_planning_time(10.0)
#         self.arm_group.set_num_planning_attempts(5)
#         self.arm_group.set_max_velocity_scaling_factor(0.3)
#         self.arm_group.set_max_acceleration_scaling_factor(0.3)

#         rospy.sleep(1.0)

#     def reach_named_position(self, target):
#         self.arm_group.set_named_target(target)
#         return self.arm_group.go(wait=True)
    

#     def compute_ik(self, pose):
#         self.arm_group.set_pose_target(pose)
#         plan = self.arm_group.plan()
#         if plan and hasattr(plan[1], 'joint_trajectory') and plan[1].joint_trajectory.points:
#             return plan[1].joint_trajectory.points[-1].positions
#         else:
#             rospy.logwarn("Inverse kinematics failed for pose: %s", pose)
#             return None

#     def generate_joint_trajectory_on_circle(self, center, radius=0.02, num_points=50, z_offset=0.000):
#         joint_trajectory = []
#         for i in range(num_points):
#             theta = 2 * np.pi * i / num_points
#             x = center[0] + radius * np.cos(theta)
#             y = center[1] + radius * np.sin(theta)
#             z = center[2] + z_offset

#             pose = Pose()
#             pose.position.x = x
#             pose.position.y = y
#             pose.position.z = z
#             q = quaternion_from_euler(0, math.pi, 0)
#             pose.orientation.x = q[0]
#             pose.orientation.y = q[1]
#             pose.orientation.z = q[2]
#             pose.orientation.w = q[3]

#             joint_angles = self.compute_ik(pose)
#             if joint_angles:
#                 joint_trajectory.append(joint_angles)
#         return joint_trajectory

#     def execute_joint_trajectory(self, joint_traj, tolerance=0.01, delay=0.002):
#         for i, joints in enumerate(joint_traj):
#             self.arm_group.set_goal_joint_tolerance(tolerance)
#             self.arm_group.set_joint_value_target(joints)
#             success = self.arm_group.go(wait=True)
#             rospy.sleep(delay)
#             rospy.loginfo("Point %d/%d completed", i+1 , len(joint_traj))
#             if not success:
#                 rospy.logwarn("Failed to reach joint configuration: %s", str(joints))
#                 return False
#         return True

#     def draw_circle_with_ik(self, T_base_marker, radius=0.02, num_points=50):
#         center = T_base_marker[:3, 3]
#         rospy.loginfo("Generating joint-space trajectory for circle...")
#         joint_traj = self.generate_joint_trajectory_on_circle(center, radius, num_points)
#         rospy.loginfo("Executing joint-space circular motion...")
#         return self.execute_joint_trajectory(joint_traj)

# def main():
#     robot = MoveItArm()
    
#     rospy.loginfo("Moving to home position...")
#     success = robot.reach_named_position("home")
#     if not success:
#         rospy.logerr("Failed to reach home position.")
#         return

#     T_base_marker = np.eye(4)
#     T_base_marker[0, 3] = 0.344
#     T_base_marker[1, 3] = 0.0
#     T_base_marker[2, 3] = 0.025 + 0.049 / 2

#     rospy.loginfo("Starting circular drawing...")
#     success = robot.draw_circle_with_ik(T_base_marker, radius=0.02, num_points=50)
#     if not success:
#         rospy.logerr("Failed to draw circle with IK.")

# if __name__ == '__main__':
#     main()

import sys
import time
import math
import rospy
import moveit_commander
import numpy as np
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler


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

    def generate_joint_trajectory_on_circle(self, center, radius=0.02, num_points=50, z_offset=0.000):
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
        return joint_trajectory

    def send_trajectory_via_action(self, joint_traj, dt=0.04):
        """
        通过 actionlib 一次性发送整条 JointTrajectory
        :param joint_traj: [[q1,q2,...], ...]  关节角序列
        :param dt:         相邻点时间间隔 (s)；调大更慢，调小更快但加速度更高
        """
    
        # 1) 构造 JointTrajectory 消息
        traj = JointTrajectory()
        traj.joint_names = self.arm_group.get_active_joints()  # 6 轴名
    
        for i, q in enumerate(joint_traj):
            p = JointTrajectoryPoint()
            p.positions = q
            p.time_from_start = rospy.Duration((i+1) * dt)
            traj.points.append(p)
    
        # 2) 构造 Action Goal
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = traj
        goal.goal_time_tolerance = rospy.Duration(0.2)  # 允许误差
    
        # 3) 发送到控制器 Action 服务器
        action_ns = "/gen3_lite_joint_trajectory_controller/follow_joint_trajectory"
        client = actionlib.SimpleActionClient(action_ns, FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for %s action server...", action_ns)
        client.wait_for_server()
    
        rospy.loginfo("Sending trajectory (%d points, dt=%.3fs)...", len(joint_traj), dt)
        client.send_goal(goal)
        client.wait_for_result()             # 阻塞直到完成
    
        result = client.get_result()
        if result.error_code == 0:
            rospy.loginfo("Trajectory execution SUCCESS")
            return True
        else:
            rospy.logwarn("Trajectory execution ABORTED, error_code=%d", result.error_code)
            return False


    def draw_circle_with_ik(self, T_base_marker, radius=0.02, num_points=50):
        center = T_base_marker[:3, 3]
        rospy.loginfo("Generating joint-space trajectory for circle...")
        joint_traj = self.generate_joint_trajectory_on_circle(center, radius, num_points)
        rospy.loginfo("Executing joint-space circular motion...")
        return self.send_trajectory_via_action(joint_traj, dt=0.05)


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

