#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import numpy as np
import tf.transformations as tf

# ==================== 固定参数 ====================
L = 0.05    # 立方体边长（可选，实际无用）
R = 0.02    # 圆轨迹半径
U = 0.4     # 圆心 x
V = 0.0     # 圆心 y
W = 0.2     # 圆心 z

# ==================== 工具函数 ====================
def build_uv(n):
    tmp = (0, 0, 1) if abs(n[2]) < 0.9 else (1, 0, 0)
    u = tf.unit_vector(np.cross(n, tmp))
    v = tf.unit_vector(np.cross(n, u))
    return u, v

def quaternion_align_z_to(n_target):
    """
    生成一个四元数，使工具坐标系的 -Z 轴朝向 n_target。
    """
    z_tool = np.array([0, 0, -1])
    n_target = np.array(n_target)
    v = np.cross(z_tool, n_target)
    s = math.sqrt(np.linalg.norm(z_tool)**2 * np.linalg.norm(n_target)**2) + np.dot(z_tool, n_target)
    q = tf.unit_vector([s, *v])  # (w, x, y, z)
    return q[1], q[2], q[3], q[0]  # 转为 ROS 顺序 (x, y, z, w)

# ==================== 主逻辑 ====================
def publish_fixed_path():
    rospy.init_node("fixed_path_publisher")

    pub = rospy.Publisher("/desired_path", Path, queue_size=1, latch=True)
    rospy.sleep(1.0)  # 等待连接

    path = Path()
    path.header.frame_id = "base_link"

    center = np.array([U, V, W])
    normal = np.array([0, 0, 1])  # 默认朝上的平面
    u, v = build_uv(normal)
    quat = quaternion_align_z_to(normal)

    for deg in range(0, 360, 5):
        theta = math.radians(deg)
        p_offset = R * (math.cos(theta) * u + math.sin(theta) * v)
        pos = center + p_offset

        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = pos
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = quat

        path.poses.append(pose)

    pub.publish(path)
    rospy.loginfo(f"[traj] Published fixed circular path with {len(path.poses)} points.")
    rospy.spin()

if __name__ == "__main__":
    publish_fixed_path()
