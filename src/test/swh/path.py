#!/usr/bin/env python3
import rospy, math
from geometry_msgs.msg import PoseStamped, Quaternion, Vector3
from nav_msgs.msg import Path
from std_srvs.srv import SetBool, SetBoolResponse
import tf.transformations as tf

# === 全局可调参数 ===
L = rospy.get_param("/cube_edge", 0.10)   # 立方体边长 10 cm
R = 0.03                                   # 默认圆半径
IDX = 0                                    # 默认投影在顶点 0

# 1️⃣ 服务：动态修改半径 & 位置 ----------------------------
def set_param_cb(req):
    global R, IDX
    # 解析 req.message 格式: "radius,idx" 例 "0.04,5"
    try:
        r, idx = req.message.split(',')
        R = float(r); IDX = int(idx)
        rospy.loginfo(f"[traj] 参数更新: R={R:.3f}, idx={IDX}")
        return SetBoolResponse(True, "OK")
    except Exception as e:
        return SetBoolResponse(False, str(e))

# 2️⃣ 根据 idx 得局部圆心 & 法向 ---------------------------
def get_local_center_normal(idx):
    half = L/2.0
    tbl = {
        0: (( half, half, half), (0,0,1)),      # 顶点 0, 法向 +Z
        1: (( half, half,-half), (0,0,1)),      # 自行拓展 7 顶点 + 12 边
    }
    return tbl[idx]

# 3️⃣ 接收立方体 Pose 并生成 Path -------------------------
def pose_cb(msg):
    global pub
    C = msg.pose.position
    Q = msg.pose.orientation
    # 转四元数 tuple
    q = [Q.w, Q.x, Q.y, Q.z]
    P_local, n_local = get_local_center_normal(IDX)
    # 把局部点/向量转世界
    p_world = tf.quaternion_matrix(q).dot(list(P_local)+[1])[:3] + [C.x, C.y, C.z]
    n_world = tf.quaternion_matrix(q).dot(list(n_local)+[0])[:3]

    # 生成圆轨迹
    path = Path()
    path.header = msg.header
    for deg in range(0, 360, 5):
        rad = math.radians(deg)
        # 构造圆面正交基 u,v
        u = tf.unit_vector(tf.cross(n_world, (0,0,1)) or (1,0,0))
        v = tf.unit_vector(tf.cross(n_world, u))
        p = [
            p_world[i] + R*(math.cos(rad)*u[i] + math.sin(rad)*v[i])
            for i in range(3)
        ]
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = p
        pose.pose.orientation = Q        # 末端朝向保持立方体
        path.poses.append(pose)
    pub.publish(path)

# 4️⃣ main -------------------------------------------------
rospy.init_node("interactive_traj_node")
pub = rospy.Publisher("/desired_path", Path, queue_size=1)
rospy.Service("/set_traj_param", SetBool, set_param_cb)

rospy.Subscriber("/aruco/cube_pose", PoseStamped, pose_cb)
rospy.loginfo("[traj] ready. call rosservice call /set_traj_param '\"0.05,3\"'")
rospy.spin()
