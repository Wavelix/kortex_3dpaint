#!/usr/bin/env python3

#roslaunch kortex_3dpaint arm_control.launch __ns:=my_gen3_lite

import sys
import time
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import quaternion_from_euler

class MoveItArm(object):
  """ExampleMoveItTrajectories"""
  def __init__(self):

    super(MoveItArm, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('arm_control')

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
    except Exception as e:
      print (e)
      self.is_init_success = False
    else:
      self.is_init_success = True


  def reach_named_position(self, target):
    arm_group = self.arm_group
    
    rospy.loginfo("Going to named target " + target)
    arm_group.set_named_target(target)
    (success_flag, trajectory_message, planning_time, error_code) = arm_group.plan()
    return arm_group.execute(trajectory_message, wait=True)

  def reach_joint_angles(self, tolerance, J=[0,0,pi/2,pi/4,0,pi/2]):
    arm_group = self.arm_group
    success = True

    # Get the current joint positions
    joint_positions = arm_group.get_current_joint_values()
    rospy.loginfo("Printing current joint positions before movement :")
    for p in joint_positions: rospy.loginfo(p)

    # Set the goal joint tolerance
    self.arm_group.set_goal_joint_tolerance(tolerance)
    for i in range(6):
      joint_positions[i] = J[i]
    arm_group.set_joint_value_target(joint_positions)
    
    # Plan and execute in one command
    success &= arm_group.go(wait=True)

    # Show joint positions after movement
    new_joint_positions = arm_group.get_current_joint_values()
    rospy.loginfo("Printing current joint positions after movement :")
    for p in new_joint_positions: rospy.loginfo(p)
    return success

  def get_cartesian_pose(self):
    arm_group = self.arm_group

    # Get the current pose and display it
    pose = arm_group.get_current_pose()
    rospy.loginfo("Actual cartesian pose is : ")
    rospy.loginfo(pose.pose)

    return pose.pose

  def reach_cartesian_pose(self, pose, tolerance, constraints):
    arm_group = self.arm_group
    
    # Set the tolerance
    arm_group.set_goal_position_tolerance(tolerance)

    # Set the trajectory constraint if one is specified
    if constraints is not None:
      arm_group.set_path_constraints(constraints)

    # Get the current Cartesian Position
    arm_group.set_pose_target(pose)

    # Plan and execute
    rospy.loginfo("Planning and going to the Cartesian Pose")
    return arm_group.go(wait=True)

  def reach_gripper_position(self, relative_position):
    gripper_group = self.gripper_group
    
    # We only have to move this joint because all others are mimic!
    gripper_joint = self.robot.get_joint(self.gripper_joint_name)
    gripper_max_absolute_pos = gripper_joint.max_bound()
    gripper_min_absolute_pos = gripper_joint.min_bound()
    # 0.96 -0.09
    print("Gripper joint name: " + self.gripper_joint_name)
    print(gripper_joint.max_bound())
    print(gripper_joint.min_bound())
    try:
      val = gripper_joint.move(relative_position * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos, True)
      return val
    except:
      return False 

def main():
  example = MoveItArm()

  success = example.is_init_success
  try:
      rospy.delete_param("/kortex_examples_test_results/moveit_general_python")
  except:
      pass
  if success:
    example.get_cartesian_pose()

  # Reaching Home
  if success:
    rospy.loginfo("Reaching Named Target Home...")
    success &= example.reach_named_position("home")
    print (success)

  """
  Picking up a pen
    Open the gripper
    Reach the pen
    Move downward
    Close the gripper
    Reach the home position
  """
  if example.is_gripper_present and success:
    rospy.loginfo("Opening the gripper...")
    success &= example.reach_gripper_position(0.6)
    print (success)
  if success:
    rospy.loginfo("Reaching the pen...")
    pose = example.get_cartesian_pose()
    pose.position.x = 0.15
    pose.position.y = -0.2
    pose.position.z = 0.125
    pose.orientation = geometry_msgs.msg.Quaternion(*quaternion_from_euler(0, pi, pi/4))
    success &= example.reach_cartesian_pose(pose=pose, tolerance=0.01, constraints=None)
    print (success)
  if success: 
    rospy.loginfo("Downward...")
    actual_pose = example.get_cartesian_pose()
    actual_pose.position.z -= 0.1
    success &= example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=None)
    print (success)
  if example.is_gripper_present and success:
    rospy.loginfo("Closing the gripper ...")
    success &= example.reach_gripper_position(0.0)
    print (success)
  if success:
    rospy.loginfo("Picking up the pen...")
    success &= example.reach_named_position("home")
    print (success)
    
  rospy.set_param("/kortex_examples_test_results/moveit_general_python", success)

  if not success:
      rospy.logerr("The example encountered an error.")

if __name__ == '__main__':
  main()