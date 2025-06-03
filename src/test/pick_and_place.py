#!/usr/bin/env python3

import sys
import rospy
import time

from kortex_driver.srv import *
from kortex_driver.msg import *

class ArmMovement:
    def __init__(self):
        try:
            rospy.init_node('pick_and_place_node')

            self.HOME_ACTION_IDENTIFIER = 2

            # Get node params
            self.robot_name = rospy.get_param('~robot_name', "my_gen3")
            self.degrees_of_freedom = rospy.get_param("/" + self.robot_name + "/degrees_of_freedom", 7)
            self.is_gripper_present = rospy.get_param("/" + self.robot_name + "/is_gripper_present", False)

            rospy.loginfo("Using robot_name " + self.robot_name + " , robot has " + str(self.degrees_of_freedom) + " degrees of freedom and is_gripper_present is " + str(self.is_gripper_present))

            # Init the action topic subscriber
            self.action_topic_sub = rospy.Subscriber("/" + self.robot_name + "/action_topic", ActionNotification, self.cb_action_topic)
            self.last_action_notif_type = None

            # Init the services
            clear_faults_full_name = '/' + self.robot_name + '/base/clear_faults'
            rospy.wait_for_service(clear_faults_full_name)
            self.clear_faults = rospy.ServiceProxy(clear_faults_full_name, Base_ClearFaults)

            read_action_full_name = '/' + self.robot_name + '/base/read_action'
            rospy.wait_for_service(read_action_full_name)
            self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)

            execute_action_full_name = '/' + self.robot_name + '/base/execute_action'
            rospy.wait_for_service(execute_action_full_name)
            self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

            set_cartesian_reference_frame_full_name = '/' + self.robot_name + '/control_config/set_cartesian_reference_frame'
            rospy.wait_for_service(set_cartesian_reference_frame_full_name)
            self.set_cartesian_reference_frame = rospy.ServiceProxy(set_cartesian_reference_frame_full_name, SetCartesianReferenceFrame)

            send_gripper_command_full_name = '/' + self.robot_name + '/base/send_gripper_command'
            rospy.wait_for_service(send_gripper_command_full_name)
            self.send_gripper_command = rospy.ServiceProxy(send_gripper_command_full_name, SendGripperCommand)

            activate_publishing_of_action_notification_full_name = '/' + self.robot_name + '/base/activate_publishing_of_action_topic'
            rospy.wait_for_service(activate_publishing_of_action_notification_full_name)
            self.activate_publishing_of_action_notification = rospy.ServiceProxy(activate_publishing_of_action_notification_full_name, OnNotificationActionTopic)
        
            get_product_configuration_full_name = '/' + self.robot_name + '/base/get_product_configuration'
            rospy.wait_for_service(get_product_configuration_full_name)
            self.get_product_configuration = rospy.ServiceProxy(get_product_configuration_full_name, GetProductConfiguration)

            validate_waypoint_list_full_name = '/' + self.robot_name + '/base/validate_waypoint_list'
            rospy.wait_for_service(validate_waypoint_list_full_name)
            self.validate_waypoint_list = rospy.ServiceProxy(validate_waypoint_list_full_name, ValidateWaypointList)
        except:
            self.is_init_success = False
        else:
            self.is_init_success = True

    def cb_action_topic(self, notif):
        self.last_action_notif_type = notif.action_event
    
    def FillCartesianWaypoint(self, new_x, new_y, new_z, new_theta_x, new_theta_y, new_theta_z, blending_radius):
        waypoint = Waypoint()
        cartesianWaypoint = CartesianWaypoint()

        cartesianWaypoint.pose.x = new_x
        cartesianWaypoint.pose.y = new_y
        cartesianWaypoint.pose.z = new_z
        cartesianWaypoint.pose.theta_x = new_theta_x
        cartesianWaypoint.pose.theta_y = new_theta_y
        cartesianWaypoint.pose.theta_z = new_theta_z
        cartesianWaypoint.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_BASE
        cartesianWaypoint.blending_radius = blending_radius
        waypoint.oneof_type_of_waypoint.cartesian_waypoint.append(cartesianWaypoint)

        return waypoint

    def wait_for_action_end_or_abort(self):
        while not rospy.is_shutdown():
            if (self.last_action_notif_type == ActionEvent.ACTION_END):
                rospy.loginfo("Received ACTION_END notification")
                return True
            elif (self.last_action_notif_type == ActionEvent.ACTION_ABORT):
                rospy.loginfo("Received ACTION_ABORT notification")
                return False
            else:
                time.sleep(0.01)

    def example_subscribe_to_a_robot_notification(self):
        # Activate the publishing of the ActionNotification
        req = OnNotificationActionTopicRequest()
        rospy.loginfo("Activating the action notifications...")
        try:
            self.activate_publishing_of_action_notification(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call OnNotificationActionTopic")
            return False
        else:
            rospy.loginfo("Successfully activated the Action Notifications!")

        rospy.sleep(1.0)
        return True

    def example_clear_faults(self):
        try:
            self.clear_faults()
        except rospy.ServiceException:
            rospy.logerr("Failed to call ClearFaults")
            return False
        else:
            rospy.loginfo("Cleared the faults successfully")
            rospy.sleep(2.5)
            return True

    def example_home_the_robot(self):
        # The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
        self.last_action_notif_type = None
        req = ReadActionRequest()
        req.input.identifier = self.HOME_ACTION_IDENTIFIER
        try:
            res = self.read_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ReadAction")
            return False
        # Execute the HOME action if we could read it
        else:
            # What we just read is the input of the ExecuteAction service
            req = ExecuteActionRequest()
            req.input = res.output
            rospy.loginfo("Sending the robot home...")
            try:
                self.execute_action(req)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ExecuteAction")
                return False
            else:
                return self.wait_for_action_end_or_abort()

    def example_set_cartesian_reference_frame(self):
        self.last_action_notif_type = None
        # Prepare the request with the frame we want to set
        req = SetCartesianReferenceFrameRequest()
        req.input.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_MIXED

        # Call the service
        try:
            self.set_cartesian_reference_frame(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call SetCartesianReferenceFrame")
            return False
        else:
            rospy.loginfo("Set the cartesian reference frame successfully")

        # Wait a bit
        rospy.sleep(0.25)
        return True

    def example_send_cartesian_pose(self, x, y, z, theta_x, theta_y, theta_z):
        self.last_action_notif_type = None
        req = ExecuteActionRequest()
        trajectory = WaypointList()

        trajectory.waypoints.append(self.FillCartesianWaypoint(x, y, z, theta_x, theta_y, theta_z, 0))

        req.input.oneof_action_parameters.execute_waypoint_list.append(trajectory)

        # Call the service
        rospy.loginfo("Sending the robot to the cartesian pose...")
        try:
            self.execute_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ExecuteWaypointTrajectory")
            return False
        else:
            return self.wait_for_action_end_or_abort()

    def example_send_gripper_command(self, value):
        # Initialize the request
        req = SendGripperCommandRequest()
        finger = Finger()
        finger.finger_identifier = 0
        finger.value = value
        req.input.gripper.finger.append(finger)
        req.input.mode = GripperMode.GRIPPER_POSITION

        rospy.loginfo("Sending the gripper command...")

        # Call the service 
        try:
            self.send_gripper_command(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call SendGripperCommand")
            return False
        else:
            time.sleep(2.0)
            return True

    def main(self):
        success = self.is_init_success
        try:
            rospy.delete_param("/kortex_examples_test_results/arm_movement_python")
        except:
            pass

        if success:
            # Clear faults
            success &= self.example_clear_faults()
            # Activate notifications
            success &= self.example_subscribe_to_a_robot_notification()
            # Home the robot
            # success &= self.example_home_the_robot()

            # Set Cartesian reference frame
            success &= self.example_set_cartesian_reference_frame()

            # Move to position A
            # success &= self.example_send_cartesian_pose(0.5, 0.2, 0.1, 90, 0, 90) 
            
            # Open gripper
            # if self.is_gripper_present:
            #     success &= self.example_send_gripper_command(0.0)
            
            # Move down to pick object
            # success &= self.example_send_cartesian_pose(0.5, 0.2, -0.02, 90, 0, 90)  
            
            # Close gripper to grasp object
            if self.is_gripper_present:
                success &= self.example_send_gripper_command(1.0)
            if self.is_gripper_present:
                success &= self.example_send_gripper_command(0.0)
            
            # Move up with object
            # success &= self.example_send_cartesian_pose(0.5, 0.2, 0.2, 90, 0, 90)  

            # Move to position B
            # success &= self.example_send_cartesian_pose(0.3, -0.2, 0.2, 90, 0, 90)  
            
            # Move down to place object
            # success &= self.example_send_cartesian_pose(0.3, -0.2, 0.02, 90, 0, 90)  
            
            # Open gripper to release object
            # if self.is_gripper_present:
            #     success &= self.example_send_gripper_command(0.0)
            
            # Move up
            # success &= self.example_send_cartesian_pose(0.3, -0.2, 0.2, 90, 0, 90)

            # Return to home position
            # success &= self.example_home_the_robot()

        # For testing purposes
        rospy.set_param("/kortex_examples_test_results/arm_movement_python", success)

        if not success:
            rospy.logerr("The example encountered an error.")

if __name__ == "__main__":
    ex = ArmMovement()
    ex.main()