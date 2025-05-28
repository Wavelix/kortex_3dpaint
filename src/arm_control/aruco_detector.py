#!/usr/bin/env python3
import rospy
import cv2
import cv2.aruco as aruco
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError

class ArucoDetector:
    def __init__(self):
        rospy.init_node('aruco_detector', anonymous=True)
        
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.marker_length = 0.05  # 根据Aruco标记实际大小调整（单位：米）
        
        # 订阅相机图像和相机信息
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.camera_info_callback)
        
        # 发布检测到的标记位姿
        self.pose_pub = rospy.Publisher("/aruco_pose", PoseStamped, queue_size=10)
        
        # 定义Aruco字典
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters_create()
        
        rospy.loginfo("Aruco detector node initialized")

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
            
            for i in range(len(ids)):
                # 打印检测到的标记ID和位姿
                rospy.loginfo("Detected Aruco ID: %d", ids[i][0])
                rospy.loginfo("Position: %s", str(tvecs[i][0]))
                rospy.loginfo("Rotation: %s", str(rvecs[i][0]))
                
                # 创建位姿消息
                pose_msg = PoseStamped()
                pose_msg.header.stamp = rospy.Time.now()
                pose_msg.header.frame_id = "camera_color_optical_frame"  # 根据你的相机帧ID调整
                
                # 设置位置
                pose_msg.pose.position.x = tvecs[i][0][0]
                pose_msg.pose.position.y = tvecs[i][0][1]
                pose_msg.pose.position.z = tvecs[i][0][2]
                
                # 设置姿态（将旋转向量转换为四元数）
                rotation_matrix = np.eye(4)
                rotation_matrix[0:3, 0:3] = cv2.Rodrigues(rvecs[i][0])[0]
                quaternion = self.rotation_matrix_to_quaternion(rotation_matrix[0:3, 0:3])
                
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
        detector = ArucoDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()