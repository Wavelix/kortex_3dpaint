#!/usr/bin/env python3
import rospy
import cv2
import cv2.aruco as aruco
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError

import numpy as np

def matrix_to_euler_xyz(R):
    r11, r12, r13 = R[0, :]
    r21, r22, r23 = R[1, :]
    r31, r32, r33 = R[2, :]

    if np.abs(r31) != 1:
        beta = -np.arcsin(r31)
        alpha = np.arctan2(r32 / np.cos(beta), r33 / np.cos(beta))
        gamma = np.arctan2(r21 / np.cos(beta), r11 / np.cos(beta))
    else:
        gamma = 0
        if r31 == -1:
            beta = np.pi / 2
            alpha = gamma + np.arctan2(r12, r13)
        else:       
            beta = -np.pi / 2
            alpha = -gamma + np.arctan2(-r12, -r13)

    return np.rad2deg([alpha, beta, gamma])

class CameraPositionCalculator:
    def __init__(self):
        # Set numpy print options for better readability
        np.set_printoptions(suppress=True, precision=4)
        # Example DH parameters for Kinova Gen3 Lite (need to replace with actual values!)
        # Format: [theta, d, a, alpha]
        self.dh_params = [
            [0, 243.3,   0,   0],     # Joint 1
            [0,    30,   0,  90],     # Joint 2
            [0,    20, 280, 180],     # Joint 3
            [0,   245,   0,  90],     # Joint 4
            [0,    109,   0,  90],     # Joint 5
            [0,   149,   -34, -90],     # Joint 6
        ]

    def deg2rad(self,degrees):
        return np.deg2rad(degrees)

    def dh_transform(self,theta, d, a, alpha):
        theta = np.deg2rad(theta)
        alpha = np.deg2rad(alpha)
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)
        # return np.array([
        #     [ct, -st * ca,  st * sa, a * ct],
        #     [st,  ct * ca, -ct * sa, a * st],
        #     [0,       sa,       ca,      d],
        #     [0,        0,        0,      1]
        # ])


        # return np.array([
        #     [ct, st * ca,  st * sa, -a * ct],
        #     [-st,  ct * ca, ct * sa, a * st],
        #     [0,       -sa,       ca,     -d],
        #     [0,        0,        0,      1]
        # ])


        return np.array([
            [ct     , -st     ,   0,  a     ],
            [st * ca,  ct * ca, -sa, -sa * d],
            [st * sa,  ct * sa,  ca,  ca * d],
            [0      ,  0      ,  0 ,  1     ]
        ])


    def forward_kinematics(self,joint_angles):
        T = np.eye(4)
        for i in range(6):
            theta = joint_angles[i]
            d, a, alpha = self.dh_params[i][1:]
            T_i = self.dh_transform(theta, d, a, alpha)
            T =  T @ T_i
        return T


    def print_output(self,name, T):
        px, py, pz = T[:3, 3]
        alpha, beta, gamma = matrix_to_euler_xyz(T[:3,:3])
        print(f"{name}:")
        # print(f"Position: px={px:.1f} mm, py={py:.1f} mm, pz={pz:.1f} mm")
        # print(f"Euler angles: α={alpha:.1f}°, β={beta:.1f}°, γ={gamma:.1f}°\n")
        print("Homogeneous transformation matrix:")
        print(np.round(T, 4))
        print("\n")
    
    def test(self,angles):
        # angles[1] += 90
        # angles[2] += 90
        # angles[3] += 90
        # angles[5] -= 90
        T = self.forward_kinematics(angles)
        self.print_output("T for base to camera:", T)
        return T


class ArucoDetector:
    def __init__(self, camera_calc, joint_angles):
        rospy.init_node('aruco_detector', anonymous=True)
        
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.marker_length = 0.05  # 根据Aruco标记实际大小调整（单位：米）
        self.camera_calc = camera_calc
        self.joint_angles = joint_angles
        
        # 订阅相机图像和相机信息
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.camera_info_callback)
        
        # 发布检测到的标记位姿
        self.pose_pub = rospy.Publisher("/aruco_pose", PoseStamped, queue_size=10)
        
        # 定义Aruco字典
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters_create()
        
        rospy.loginfo("Aruco detector node initialized")

    def build_homogeneous_matrix(self,rvec, tvec):
        R, _ = cv2.Rodrigues(rvec)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = tvec * 1000
        return T

    def camera_info_callback(self, msg):
        """获取相机内参和畸变系数"""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.K).reshape(3, 3)
            self.dist_coeffs = np.array(msg.D)
            rospy.loginfo("Camera parameters received")

    def image_callback(self, msg):
        """处理图像并检测Aruco标记"""
        if self.camera_matrix is None:
            rospy.logwarn_once("Camera parameters not yet received")
            return
            
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
            
        # 转换为灰度图像
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # 检测标记
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        
        if ids is not None:
            # 估计标记位姿
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_length, self.camera_matrix, self.dist_coeffs)
            
            T_base_cam = self.camera_calc.test(self.joint_angles)

            for i in range(len(ids)):
                # 打印检测到的标记ID和位姿
                rospy.loginfo("Detected Aruco ID: %d", ids[i][0])

                T_cam_obj = self.build_homogeneous_matrix(rvecs[i][0], tvecs[i][0])
                T_base_obj = T_base_cam @ T_cam_obj

                pos = T_base_obj[:3, 3]
                R = T_base_obj[:3, :3]
                euler = matrix_to_euler_xyz(R)

                # rospy.loginfo("Position: %s", str(tvecs[i][0]))
                # rospy.loginfo("Rotation: %s", str(rvecs[i][0]))
                rospy.loginfo(f"物块在 base 坐标系下的位置：{pos}")
                rospy.loginfo(f"姿态（欧拉角）：Roll={euler[0]:.2f}°, Pitch={euler[1]:.2f}°, Yaw={euler[2]:.2f}°")
                
                # # 创建位姿消息
                # pose_msg = PoseStamped()
                # pose_msg.header.stamp = rospy.Time.now()
                # pose_msg.header.frame_id = "camera_color_optical_frame"  # 根据你的相机帧ID调整
                
                # # 设置位置
                # pose_msg.pose.position.x = tvecs[i][0][0]
                # pose_msg.pose.position.y = tvecs[i][0][1]
                # pose_msg.pose.position.z = tvecs[i][0][2]
                
                # # 设置姿态（将旋转向量转换为四元数）
                # rotation_matrix = np.eye(4)
                # rotation_matrix[0:3, 0:3] = cv2.Rodrigues(rvecs[i][0])[0]
                # quaternion = self.rotation_matrix_to_quaternion(rotation_matrix[0:3, 0:3])
                
                # pose_msg.pose.orientation.x = quaternion[0]
                # pose_msg.pose.orientation.y = quaternion[1]
                # pose_msg.pose.orientation.z = quaternion[2]
                # pose_msg.pose.orientation.w = quaternion[3]

                # 从 T_base_obj 中提取
                pose_msg = PoseStamped()
                pose_msg.header.stamp = rospy.Time.now()
                pose_msg.header.frame_id = "base_link"  # 修改为 base 坐标系

                # 位置
                pose_msg.pose.position.x = pos[0]
                pose_msg.pose.position.y = pos[1]
                pose_msg.pose.position.z = pos[2]

                # 四元数
                quaternion = self.rotation_matrix_to_quaternion(R)
                pose_msg.pose.orientation.x = quaternion[0]
                pose_msg.pose.orientation.y = quaternion[1]
                pose_msg.pose.orientation.z = quaternion[2]
                pose_msg.pose.orientation.w = quaternion[3]
                
                # 发布位姿
                self.pose_pub.publish(pose_msg)
                
            # 在图像上绘制检测结果
            aruco.drawDetectedMarkers(cv_image, corners, ids)
            for i in range(len(ids)):
                aruco.drawAxis(cv_image, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.05)
        
        # 显示结果
        cv2.imshow("Aruco Detection", cv_image)
        cv2.waitKey(1)
    
    def rotation_matrix_to_quaternion(self, R):
        """将旋转矩阵转换为四元数"""
        q = np.zeros(4)
        q[3] = np.sqrt(1.0 + R[0, 0] + R[1, 1] + R[2, 2]) / 2.0
        q[0] = (R[2, 1] - R[1, 2]) / (4.0 * q[3])
        q[1] = (R[0, 2] - R[2, 0]) / (4.0 * q[3])
        q[2] = (R[1, 0] - R[0, 1]) / (4.0 * q[3])
        return q

if __name__ == '__main__':
    try:
        camera_calc = CameraPositionCalculator()
        joint_angles = [-36, -11, 120, -90, -50, -81]  # 示例，替换为实时读取的角度

        joint_angles[1] += 90
        joint_angles[2] += 90
        joint_angles[3] += 90
        joint_angles[5] -= 90

        detector = ArucoDetector(camera_calc, joint_angles)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
