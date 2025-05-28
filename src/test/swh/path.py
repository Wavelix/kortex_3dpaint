#!/usr/bin/env python3
import rospy, math
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_srvs.srv import SetBool, SetBoolResponse
import tf.transformations as tf

# ==================== 参数 ====================
L   = rospy.get_param("/cube_edge", 0.05)   # 立方体边长 5 cm
R   = 0.02                                  # 默认圆半径
IDX = 0                                     # 圆心所在实体 (0-7 顶点, 8-19 边)

# ---------- service：动态改半径 & 位置 ----------
def set_param_cb(req):
    global R, IDX
    try:
        r, idx = req.message.split(',')
        R, IDX = float(r), int(idx)
        rospy.loginfo(f"[traj] R={R:.3f}  idx={IDX}")
        return SetBoolResponse(True, "ok")
    except Exception as e:
        return SetBoolResponse(False, str(e))

# ---------- 依据 idx 给局部圆心 & 平面法向 ----------
def get_local_center_normal(idx):
    h = L/2.0
    # 顶点 0-7
    verts = [
        (+h,+h,+h),(+h,+h,-h),(+h,-h,+h),(+h,-h,-h),
        (-h,+h,+h),(-h,+h,-h),(-h,-h,+h),(-h,-h,-h)
    ]
    vert_norm = [(1,1,1),(1,1,-1),(1,-1,1),(1,-1,-1),
                 (-1,1,1),(-1,1,-1),(-1,-1,1),(-1,-1,-1)]
    # 12 条边 8-19（示例只列 4 条，其余可自行补）
    edges = {
        8 : ((+h,+h, 0), (0,0,1)),   # +X+Y 边（沿 Z）
        9 : ((+h,-h, 0), (0,0,1)),   # +X-Y
        10: ((-h,+h, 0), (0,0,1)),   # -X+Y
        11: ((-h,-h, 0), (0,0,1)),   # -X-Y
        # …… 12-19 依次补全
    }

    if idx < 8:
        p = verts[idx]
        n = tf.unit_vector(vert_norm[idx])
    else:
        p, n = edges[idx]
    return list(p), list(n)

# ---------- 工具：选平面基 u,v ----------
def build_uv(n):
    # 取一个不平行的临时向量
    tmp = (0,0,1) if abs(n[2]) < 0.9 else (1,0,0)
    u = tf.unit_vector(tf.cross(n, tmp))
    v = tf.unit_vector(tf.cross(n, u))
    return u, v                                # u ⟂ v ⟂ n

# ---------- 工具：把点压到最近面 ----------
def snap_to_face(p):
    half = L/2.0
    d = [abs(p[i]) - half for i in range(3)]   # 超界量
    axis = d.index(max(d))                     # 最大超界轴
    if d[axis] > 1e-6:                         # 真正越界才修
        p[axis] = math.copysign(half, p[axis])
    return p

# ---------- 主回调：收到立方体 Pose → 发布 Path ----------
def pose_cb(msg):
    Cw = msg.pose.position                    # 中心 (world)
    Qw = msg.pose.orientation
    q = [Qw.w, Qw.x, Qw.y, Qw.z]

    center_l, n_l = get_local_center_normal(IDX)
    u_l, v_l = build_uv(n_l)

    path = Path();  path.header = msg.header
    for deg in range(0, 360, 5):              # 5° 采样
        rad = math.radians(deg)

        # 1⃣  圆上原始点 (局部)
        p_l = [ center_l[i] +
                R*( math.cos(rad)*u_l[i] + math.sin(rad)*v_l[i] )
                for i in range(3) ]

        # 2⃣  压到最近面
        p_l = snap_to_face(p_l)

        # 3⃣  转到 world
        p_w = tf.quaternion_matrix(q).dot(p_l + [1])[:3] + [Cw.x, Cw.y, Cw.z+ L/2.0]

        # 4⃣  填 PoseStamped
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = p_w
        pose.pose.orientation = Qw            # 保持立方体姿态
        path.poses.append(pose)

    pub.publish(path)

# ==================== ROS 节点启动 ====================
rospy.init_node("wrapped_circle_traj_node")
pub = rospy.Publisher("/desired_path", Path, queue_size=1)
rospy.Service("/set_traj_param", SetBool, set_param_cb)
rospy.Subscriber("/aruco/cube_pose", PoseStamped, pose_cb)
rospy.loginfo("[traj] ready. 例: rosservice call /set_traj_param '\"0.03,8\"'")
rospy.spin()
