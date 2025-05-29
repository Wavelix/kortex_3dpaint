#!/usr/bin/env python3
import sys, os, time, math, rospy, numpy as np, actionlib
from kortex_driver.srv import *
from kortex_driver.msg import *
from nav_msgs.msg import Path
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation

# 若有自定义运动学模型仍可保留
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from Arm import Gen3LiteArm


class ArmControl(object):
    def __init__(self):
        try:
            rospy.init_node('arm_control')
            self.robot_name = 'my_gen3_lite'
            rospy.loginfo("Using robot_name " + self.robot_name)

            # ---------- 状态缓存 ----------
            self.my_path_   = None
            self.curr_state_= None
            self.plot       = False
            self.arm_model  = Gen3LiteArm()

            # ---------- 话题 ----------
            self.path_subscriber = rospy.Subscriber(
                "/desired_path", Path, self.get_path, queue_size=1)
            self.curr_state_sub  = rospy.Subscriber(
                "/my_gen3_lite/joint_states", JointState, self.get_curr_state, queue_size=1)

            # ----------  Kinova 服务 / Action ----------
            clear_faults = f"/{self.robot_name}/base/clear_faults"
            rospy.wait_for_service(clear_faults)
            self.clear_faults = rospy.ServiceProxy(clear_faults, Base_ClearFaults)

            act_topic = f"/{self.robot_name}/action_topic"
            self.action_topic_sub = rospy.Subscriber(act_topic, ActionNotification,
                                                     self.cb_action_topic)
            self.last_action_notif_type = None

            self.client = actionlib.SimpleActionClient(
                f"/{self.robot_name}/cartesian_trajectory_controller/follow_cartesian_trajectory",
                kortex_driver.msg.FollowCartesianTrajectoryAction)
            self.client.wait_for_server()

        except Exception as e:
            rospy.logerr("Init failed: %s", e)
            self.is_init_success = False
        else:
            self.is_init_success = True

    # ----------------------------------------------------
    #  回调 & 工具函数
    # ----------------------------------------------------
    def cb_action_topic(self, notif):
        self.last_action_notif_type = notif.action_event

    def get_path(self, msg):
        self.my_path_ = msg

    def get_curr_state(self, msg):
        self.curr_state_ = np.array(msg.position[0:6])

    # ---------- Waypoint 生成：位置 + 四元数 ----------
    def FillCartesianWaypoint(self, pos, quat, blend=0.01):
        """
        pos : (x,y,z)  或 geometry_msgs/Point
        quat: (x,y,z,w)或 geometry_msgs/Quaternion
        """
        wp = CartesianWaypoint()

        # —位置—
        if hasattr(pos, 'x'):
            wp.pose.x, wp.pose.y, wp.pose.z = pos.x, pos.y, pos.z
        else:
            wp.pose.x, wp.pose.y, wp.pose.z = pos

        # —姿态—  四元数 → Euler(rad)
        if hasattr(quat, 'x'):
            q_list = [quat.x, quat.y, quat.z, quat.w]
        else:
            q_list = quat
        roll, pitch, yaw = Rotation.from_quat(q_list).as_euler('xyz')
        wp.pose.theta_x = roll
        wp.pose.theta_y = pitch
        wp.pose.theta_z = yaw

        # 其他
        wp.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_BASE
        wp.blending_radius = blend     # 圆角过渡
        return wp

    # ----------------------------------------------------
    #  主要执行：把 Path 发送给动作服务器
    # ----------------------------------------------------
    def follow_path(self):
        if self.my_path_ is None:
            rospy.logerr("No path received")
            return False

        goal = FollowCartesianTrajectoryGoal()
        for pose_stamped in self.my_path_.poses:
            p = pose_stamped.pose.position
            q = pose_stamped.pose.orientation
            goal.trajectory.append(self.FillCartesianWaypoint(p, q, 0.01))

        rospy.loginfo("Sending trajectory with %d waypoints", len(goal.trajectory))
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration(1000.0))
        return True

    # ----------------------------------------------------
    #  main 流程（保留清故障 / home 等）
    # ----------------------------------------------------
    def main(self):
        if not self.is_init_success:
            rospy.logerr("Initialization failed")
            return False

        try:
            self.clear_faults()
        except rospy.ServiceException:
            rospy.logerr("Failed ClearFaults")
            return False

        rospy.loginfo("Waiting for Path ...")
        while not rospy.is_shutdown() and self.my_path_ is None:
            rospy.sleep(0.1)

        if not self.follow_path():
            return False

        rospy.loginfo("Complete painting!")
        return True


if __name__ == '__main__':
    ArmControl().main()
