#!/usr/bin/env python3

import sys
import time
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import cv2
import numpy as np
from math import pi
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import quaternion_from_euler, quaternion_matrix, quaternion_from_matrix, euler_from_quaternion

class MoveItArm(object):
    """ExampleMoveItTrajectories with Hand-Eye Calibration"""
    def __init__(self):
        super(MoveItArm, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('eyehand')

        # 手眼标定数据结构
        self.base_effector_poses = []  # 存储机械臂末端位姿 (base->effector)
        self.camera_marker_poses = []  # 存储标记板位姿 (camera->marker)
        self.handeye_matrix = None     # 存储标定结果 (effector->camera)
        self.latest_marker_pose = None # 最新检测到的标记板位姿
        
        try:
            self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", False)
            if self.is_gripper_present:
                gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
                self.gripper_joint_name = gripper_joint_names[0]
            else:
                self.gripper_joint_name = ""
            self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 7)

            # Create the MoveItInterface necessary objects
            arm_group_name = "arm"
            self.robot = moveit_commander.RobotCommander("robot_description")
            self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
            self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
            self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
                                                        moveit_msgs.msg.DisplayTrajectory,
                                                        queue_size=20)

            if self.is_gripper_present:
                gripper_group_name = "gripper"
                self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())

            rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())
            
            # 订阅Aruco检测位姿
            rospy.Subscriber("/aruco_pose", PoseStamped, self.marker_pose_callback)
            
        except Exception as e:
            rospy.logerr("Initialization failed: %s", str(e))
            self.is_init_success = False
        else:
            self.is_init_success = True

    def marker_pose_callback(self, msg):
        """Aruco标记位姿回调函数"""
        self.latest_marker_pose = msg.pose
        # rospy.loginfo("Received ArUco marker pose.") # 频繁记录，方便调试

    def pose_to_matrix(self, pose):
        """将geometry_msgs/Pose转换为4x4齐次变换矩阵"""
        T = np.eye(4)
        
        # 位置
        T[0, 3] = pose.position.x
        T[1, 3] = pose.position.y
        T[2, 3] = pose.position.z
        
        # 姿态（四元数转旋转矩阵）
        q = [pose.orientation.x, pose.orientation.y, 
             pose.orientation.z, pose.orientation.w]
        R = quaternion_matrix(q)[:3, :3]
        T[:3, :3] = R
        
        return T

    def draw_on_surface(self, shape='grid', size=0.1, step=0.02):
        rospy.loginfo("等待 ArUco marker 位姿...")
        self.latest_marker_pose = None
        start_time = time.time()
        while self.latest_marker_pose is None:
            if time.time() - start_time > 5.0:
                rospy.logwarn("未检测到 ArUco marker，取消画画任务")
                return False
            rospy.sleep(0.05)

        # 计算 marker 在 base 坐标系下的位姿
        T_base_effector = self.pose_to_matrix(self.get_cartesian_pose())
        T_camera_marker = self.pose_to_matrix(self.latest_marker_pose)
        T_base_marker = T_base_effector @ self.handeye_matrix @ T_camera_marker

        # 构造局部表面网格点
        def generate_surface_points(T_base_marker, size=0.1, step=0.01):
            points = []
            for x in np.arange(-size/2, size/2, step):
                for y in np.arange(-size/2, size/2, step):
                    pt_local = np.array([x, y, 0, 1.0])
                    pt_base = T_base_marker @ pt_local
                    points.append(pt_base[:3])
            return points

        # 构造 ROS Pose 列表
        def points_to_cartesian_poses(points_base, z_offset=0.005):
            poses = []
            for pt in points_base:
                pose = Pose()
                pose.position.x = pt[0]
                pose.position.y = pt[1]
                pose.position.z = pt[2] + z_offset  # 加一点高度，避免扎进去

                q = quaternion_from_euler(0, math.pi, 0)
                pose.orientation.x = q[0]
                pose.orientation.y = q[1]
                pose.orientation.z = q[2]
                pose.orientation.w = q[3]
                poses.append(pose)
            return poses

        rospy.loginfo("正在生成 %s 表面轨迹..." % shape)
        if shape == 'grid':
            points = generate_surface_points(T_base_marker, size=size, step=step)
        else:
            rospy.logerr("暂不支持的图形类型: %s" % shape)
            return False

        poses = points_to_cartesian_poses(points, z_offset=0.003)

        # 规划并执行轨迹
        rospy.loginfo("开始执行轨迹，总点数：%d" % len(poses))
        (plan, fraction) = self.arm_group.compute_cartesian_path(
            poses,
            eef_step=0.005,
            jump_threshold=0.0
        )

        if fraction > 0.9:
            self.arm_group.execute(plan, wait=True)
            rospy.loginfo("画图完成")
            return True
        else:
            rospy.logwarn("轨迹规划未完成，仅完成率 %.2f%%" % (fraction * 100))
            return False

    def matrix_to_pose(self, matrix):
        """将4x4齐次变换矩阵转换为geometry_msgs/Pose"""
        pose = Pose()
        
        # 位置
        pose.position.x = matrix[0, 3]
        pose.position.y = matrix[1, 3]
        pose.position.z = matrix[2, 3]
        
        # 姿态（旋转矩阵转四元数）
        R = matrix[:3, :3]
        # quaternion_from_matrix 期望一个4x4的齐次矩阵作为输入
        temp_matrix = np.eye(4)
        temp_matrix[:3, :3] = R
        q = quaternion_from_matrix(temp_matrix)
        
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        
        return pose

    def add_calibration_data(self):
        """添加当前位姿下的标定数据，并强化等待机制"""
        self.latest_marker_pose = None # 重置为None，确保等待新的Aruco数据
        rospy.loginfo("Waiting for ArUco marker pose for data point %d...", len(self.base_effector_poses) + 1)
        
        # 等待Aruco位姿，设置一个合理的超时时间
        start_time = time.time()
        timeout_duration = 5.0 # 5秒超时
        while self.latest_marker_pose is None:
            if time.time() - start_time > timeout_duration:
                rospy.logwarn("Timeout: No ArUco marker pose received after %f seconds for data point %d.", 
                              timeout_duration, len(self.base_effector_poses) + 1)
                return False
            rospy.sleep(0.05) # 短暂休眠，避免占用过多CPU
        
        rospy.loginfo("ArUco marker pose received for data point %d.", len(self.base_effector_poses) + 1)
        
        # 获取当前机械臂末端位姿
        effector_pose = self.get_cartesian_pose()
        
        # 转换为矩阵
        T_base_effector = self.pose_to_matrix(effector_pose)
        T_camera_marker = self.pose_to_matrix(self.latest_marker_pose)
        
        # 存储数据 (T_base_effector 和 T_camera_marker)
        # 在 Eye-on-Hand 标定中，cv2.calibrateHandEye 期望输入 T_effector_base 和 T_marker_camera
        # 我们在这里存储原始数据，在执行标定函数时再进行逆运算，保持数据原始性
        self.base_effector_poses.append(T_base_effector)
        self.camera_marker_poses.append(T_camera_marker)
        
        rospy.loginfo("Successfully added calibration data point %d", len(self.base_effector_poses))
        return True

    def perform_handeye_calibration(self):
        """执行手眼标定计算"""
        # 建议增加数据点数量以提高鲁棒性，至少15个点
        if len(self.base_effector_poses) < 15: 
            rospy.logerr("Insufficient data points (%d), need at least 15 for reliable calibration.", len(self.base_effector_poses))
            return False
        
        R_gripper2base = []
        t_gripper2base = []
        R_target2cam = []
        t_target2cam = []

        for i in range(len(self.base_effector_poses)):
            T_base_effector = self.base_effector_poses[i]
            T_camera_marker = self.camera_marker_poses[i]
            
            # Eye-on-Hand 标定，需要 T_effector_base 和 T_marker_camera
            # T_effector_base = (T_base_effector)^-1
            T_effector_base = np.linalg.inv(T_base_effector)
            R_gripper2base.append(T_effector_base[:3, :3])
            t_gripper2base.append(T_effector_base[:3, 3])

            # T_marker_camera = (T_camera_marker)^-1
            T_marker_camera = np.linalg.inv(T_camera_marker)
            R_target2cam.append(T_marker_camera[:3, :3])
            t_target2cam.append(T_marker_camera[:3, 3])
        
        rospy.loginfo("Performing hand-eye calibration with OpenCV's CALIB_HAND_EYE_TSAI method...")
        try:
            R_effector_camera, t_effector_camera = cv2.calibrateHandEye(
                R_gripper2base, t_gripper2base,
                R_target2cam, t_target2cam,
                method=cv2.CALIB_HAND_EYE_TSAI # 推荐使用Tsai方法，通常效果较好
            )
        except cv2.error as e:
            rospy.logerr("OpenCV hand-eye calibration failed: %s", str(e))
            return False
        
        # 组合成变换矩阵
        self.handeye_matrix = np.eye(4)
        self.handeye_matrix[:3, :3] = R_effector_camera
        self.handeye_matrix[:3, 3] = t_effector_camera.flatten()
        
        rospy.loginfo("Hand-eye calibration completed!")
        rospy.loginfo("Effector to camera transformation (T_effector_camera):\n%s", str(self.handeye_matrix))
        
        # 保存标定结果
        try:
            np.save('handeye_matrix.npy', self.handeye_matrix)
            rospy.loginfo("Saved hand-eye matrix to handeye_matrix.npy")
        except Exception as e:
            rospy.logerr("Failed to save hand-eye matrix: %s", str(e))
            
        return True

    def move_to_calibration_poses(self):
        """移动到标定位姿序列"""
        # 定义一组安全的标定位置（根据实际工作空间调整）
        # 用户要求不修改此部分，但请注意姿态多样性对标定结果的重要性
        calibration_poses = [
            [0.2, 0.1, 0.45, 0, pi, 0],
            [0.2, -0.1, 0.45, 0, pi, 0],
            [0.2, 0.05, 0.45, 0, pi, pi/6],
            [0.2, -0.05, 0.45, 0, pi, pi/6],
            [0.3, 0.15, 0.4, 0, pi, 0],
            [0.3, -0.15, 0.4, 0, pi, 0],
            [0.35, 0.1, 0.4, 0, pi, 0],
            [0.35, -0.1, 0.4, 0, pi, 0],
            [0.35, 0.05, 0.3, 0, pi, 0], 
            [0.35, -0.05, 0.3, 0, pi, 0],
            [0.35, 0.05, 0.25, 0, pi, 0], 
            [0.35, -0.05, 0.2, 0, pi, pi/6],
            [0.3, 0.1, 0.4, 0, pi, 0],
            [0.3, -0.1, 0.4, 0, pi, 0],
            [0.25, 0.0, 0.4, 0, pi, pi/6],
            [0.25, 0.1, 0.3, 0, pi, pi/6],
            [0.25, -0.1, 0.3, 0, pi, pi/6],
            [0.35, 0.15, 0.4, 0, pi, pi/6],
            [0.35, -0.15, 0.4, 0, pi, 0],
            [0.3, 0.1, 0.3, 0, pi, pi/6],
            [0.3, -0.1, 0.3, 0, pi, 0],
            [0.35, -0.0, 0.4, 0, pi, pi/6],
            [0.35, -0.0, 0.4, 0, pi, 0],
            [0.2, -0.05, 0.45, 0, pi, 0],
        ]
        
        for i, pose_params in enumerate(calibration_poses):
            rospy.loginfo("Moving to calibration pose %d/%d", i+1, len(calibration_poses))
            
            target_pose = Pose() # 创建新的 Pose 对象
            target_pose.position.x = pose_params[0]
            target_pose.position.y = pose_params[1]
            target_pose.position.z = pose_params[2]
            target_pose.orientation = geometry_msgs.msg.Quaternion(*quaternion_from_euler(
                pose_params[3], pose_params[4], pose_params[5]))
            
            if not self.reach_cartesian_pose(pose=target_pose, tolerance=0.02, constraints=None):
                rospy.logwarn("Failed to reach pose %d. Skipping data collection for this pose.", i+1)
                continue
            
            rospy.sleep(1.0) # 等待机械臂稳定
            
            if not self.add_calibration_data():
                rospy.logwarn("Failed to add data for pose %d. ArUco marker not detected or timeout.", i+1)
        
        return True

    def reach_named_position(self, target):
        arm_group = self.arm_group
        rospy.loginfo("Going to named target " + target)
        
        arm_group.set_named_target(target)
        
        plan_success, trajectory_message, planning_time, error_code = arm_group.plan()
        
        if plan_success:
            return arm_group.execute(trajectory_message, wait=True)
        else:
            rospy.logwarn(f"Failed to plan to named target {target}. Error code: {error_code.val}")
            return False

    def reach_joint_angles(self, tolerance, J=[0,0,pi/2,pi/4,0,pi/2]):
        arm_group = self.arm_group
        success = True

        rospy.loginfo("Setting joint angles...")
        joint_positions = arm_group.get_current_joint_values()
        rospy.loginfo("Current joint positions before movement : %s", [f"{p:.3f}" for p in joint_positions])

        self.arm_group.set_goal_joint_tolerance(tolerance)
        for i in range(min(len(J), len(joint_positions))): # 避免索引越界
            joint_positions[i] = J[i]
        arm_group.set_joint_value_target(joint_positions)
        
        success &= arm_group.go(wait=True)

        new_joint_positions = arm_group.get_current_joint_values()
        rospy.loginfo("Current joint positions after movement : %s", [f"{p:.3f}" for p in new_joint_positions])
        return success

    def get_cartesian_pose(self):
        arm_group = self.arm_group

        pose = arm_group.get_current_pose()
        rospy.loginfo("Actual cartesian pose is : ")
        rospy.loginfo("Position: x=%.4f, y=%.4f, z=%.4f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
        rospy.loginfo("Orientation: x=%.4f, y=%.4f, z=%.4f, w=%.4f", pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w)

        return pose.pose

    def reach_cartesian_pose(self, pose, tolerance, constraints):
        arm_group = self.arm_group
        
        arm_group.set_goal_position_tolerance(tolerance)

        if constraints is not None:
            arm_group.set_path_constraints(constraints)

        arm_group.set_pose_target(pose)

        rospy.loginfo("Planning and going to the Cartesian Pose...")
        plan_success, trajectory_message, planning_time, error_code = arm_group.plan()

        if plan_success:
            return arm_group.execute(trajectory_message, wait=True)
        else:
            rospy.logwarn(f"Failed to plan to Cartesian Pose. Error code: {error_code.val}")
            return False

    def reach_gripper_position(self, relative_position):
        gripper_group = self.gripper_group
        
        gripper_joint = self.robot.get_joint(self.gripper_joint_name)
        if not gripper_joint:
            rospy.logerr("Gripper joint not found: %s", self.gripper_joint_name)
            return False

        gripper_max_absolute_pos = gripper_joint.max_bound()
        gripper_min_absolute_pos = gripper_joint.min_bound()
        
        rospy.loginfo(f"Gripper joint name: {self.gripper_joint_name}")
        rospy.loginfo(f"Gripper max bound: {gripper_max_absolute_pos}")
        rospy.loginfo(f"Gripper min bound: {gripper_min_absolute_pos}")

        try:
            target_position = relative_position * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos
            val = gripper_joint.move(target_position, True)
            return val
        except Exception as e:
            rospy.logerr("Failed to move gripper: %s", str(e))
            return False 

def main():
    example = MoveItArm()

    success = example.is_init_success
    try:
        rospy.delete_param("/kortex_examples_test_results/moveit_general_python")
    except:
        pass
    
    # 首先移动到home位置
    if success:
        rospy.loginfo("Reaching Named Target Home...")
        success &= example.reach_named_position("home")
    
    # 执行手眼标定流程
    if success:
        rospy.loginfo("Starting hand-eye calibration process...")
        
        # 移动到各个标定位姿并采集数据
        example.move_to_calibration_poses()
        
        # 执行标定计算
        if example.perform_handeye_calibration():
            rospy.loginfo("Hand-eye calibration SUCCESS!")
        else:
            rospy.logerr("Hand-eye calibration FAILED! Check logs for details (e.g., insufficient data points, OpenCV error).")
            success = False
    
    # 验证标定结果（可选）
    if success and example.handeye_matrix is not None:
        rospy.loginfo("Verifying calibration...")
        
        # 获取当前机械臂末端位姿
        effector_pose = example.get_cartesian_pose()
        T_base_effector = example.pose_to_matrix(effector_pose)
        
        # 等待最新的Aruco位姿进行验证
        example.latest_marker_pose = None
        rospy.loginfo("Waiting for ArUco marker pose for verification...")
        start_time = time.time()
        timeout_duration = 5.0 
        while example.latest_marker_pose is None:
            if time.time() - start_time > timeout_duration:
                rospy.logwarn("Timeout: No ArUco marker pose received for verification.")
                break
            rospy.sleep(0.05)

        if example.latest_marker_pose:
            T_camera_marker = example.pose_to_matrix(example.latest_marker_pose)
            
            # 计算标记板在base坐标系中的理论位置 (T_base_marker = T_base_effector @ T_effector_camera @ T_camera_marker)
            T_base_marker_calculated = T_base_effector @ example.handeye_matrix @ T_camera_marker
            
            rospy.loginfo("Marker position in base frame (calculated based on calibration):")
            rospy.loginfo("x=%.4f, y=%.4f, z=%.4f", 
                          T_base_marker_calculated[0,3], T_base_marker_calculated[1,3], T_base_marker_calculated[2,3])
            
            # 姿态（四元数）
            q_calculated = quaternion_from_matrix(T_base_marker_calculated)
            rospy.loginfo("Orientation (calculated): x=%.4f, y=%.4f, z=%.4f, w=%.4f", 
                          q_calculated[0], q_calculated[1], q_calculated[2], q_calculated[3])

        else:
            rospy.logwarn("Verification skipped as no ArUco marker pose was available.")

    if success:
        example.draw_on_surface(shape='grid', size=0.08, step=0.015)
        rospy.set_param("/kortex_examples_test_results/moveit_general_python", success)

    if not success:
        rospy.logerr("The hand-eye calibration process encountered errors.")
    else:
        rospy.loginfo("Hand-eye calibration process finished successfully.")

if __name__ == '__main__':
    main()
