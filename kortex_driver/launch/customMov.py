#!/usr/bin/env python3

import sys
import rospy
import time
from kortex_driver.srv import *
from kortex_driver.msg import *

class movTest:
    def __init__(self):
        try:
            rospy.init_node('kinova_rishik')

            self.HOME_ACTION_IDENTIFIER = 2

            self.action_topic_sub = None
            self.all_notifs_succeeded = True

            self.all_notifs_succeeded = True

            # Get node params
            self.robot_name = rospy.get_param('~robot_name', "my_gen3")
            self.degrees_of_freedom = rospy.get_param("/" + self.robot_name + "/degrees_of_freedom", 7)
            self.is_gripper_present = rospy.get_param("/" + self.robot_name + "/is_gripper_present", False)

            rospy.loginfo("Using robot_name " + self.robot_name)

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
        except:
            self.is_init_success = False
        else:
            self.is_init_success = True

    def cb_action_topic(self, notif):
        self.last_action_notif_type = notif.action_event

    def wait_for_action_end_or_abort(self):
        while not rospy.is_shutdown():
            if (self.last_action_notif_type == ActionEvent.ACTION_END):
                rospy.loginfo("Received ACTION_END notification")
                return True
            elif (self.last_action_notif_type == ActionEvent.ACTION_ABORT):
                rospy.loginfo("Received ACTION_ABORT notification")
                self.all_notifs_succeeded = False
                return False
            else:
                time.sleep(0.01)

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
        req = ReadActionRequest()
        req.input.identifier = self.HOME_ACTION_IDENTIFIER
        self.last_action_notif_type = None
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
        # Prepare the request with the frame we want to set
        req = SetCartesianReferenceFrameRequest()
        req.input.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_MIXED

        # Call the service
        try:
            self.set_cartesian_reference_frame()
        except rospy.ServiceException:
            rospy.logerr("Failed to call SetCartesianReferenceFrame")
            return False
        else:
            rospy.loginfo("Set the cartesian reference frame successfully")
            return True

        # Wait a bit
        rospy.sleep(0.25)

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
    
    '''
    def send_gripper_command(self, value):
        # Initialize the request
        # Close the gripper

        if(not self.is_gripper_present):
           print("Gripper not detected!")
           return False

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
            return True
    '''

    def main(self):
        # For testing purposes
        success = self.is_init_success
        result_topic = "/kinova_rishik_results/movtest"

        try:
            rospy.delete_param(result_topic)
        except:
            pass

        if success:
            success &= self.example_clear_faults()
            # success &= self.example_home_the_robot()
            success &= self.example_set_cartesian_reference_frame()
            success &= self.example_subscribe_to_a_robot_notification()

            # POSITION DEFINITION
            my_cartesian_speed = CartesianSpeed()
            my_cartesian_speed.translation = 0.2 # m/s
            my_cartesian_speed.orientation = 15  # deg/s

            my_constrained_pose = ConstrainedPose()
            my_constrained_pose.constraint.oneof_type.speed.append(my_cartesian_speed)

            poseId = 1
            try:
                while True:
                    shouldQuit = input("Next[enter] / type 'quit' to end: ")
                    if(shouldQuit.lower() == 'quit'):
                        break

                    px = float(input("\npose X: "))
                    py = float(input("pose Y: "))
                    pz = float(input("pose Z: "))
                    # gripperVal = input("\nGripper Position: ")

                    if(((px**2) + (py**2) + (pz**2))**0.5 >= 0.9):
                        print("Infeasible pose, try again")
                        continue

                    # if(gripperVal != ''):
                    #     gripperVal = float(gripperVal)
                    #     self.send_gripper_command(gripperVal)

                    my_constrained_pose.target_pose.x = px
                    my_constrained_pose.target_pose.y = py
                    my_constrained_pose.target_pose.z = pz
                    my_constrained_pose.target_pose.theta_x = 0     # 0deg - points down
                    my_constrained_pose.target_pose.theta_y = 180   # 180deg - camera is outward and straight
                    my_constrained_pose.target_pose.theta_z = 0     # 0deg - points toward +ve y-axis

                    req = ExecuteActionRequest()
                    req.input.oneof_action_parameters.reach_pose.append(my_constrained_pose)
                    req.input.name = "pose" + str(poseId)
                    req.input.handle.action_type = ActionType.REACH_POSE
                    req.input.handle.identifier = 1000 + poseId

                    rospy.loginfo("Sending pose " + str(poseId) + "...")
                    self.last_action_notif_type = None
                    try:
                        self.execute_action(req)
                    except rospy.ServiceException:
                        rospy.logerr("Failed to send pose "+str(poseId))
                        success = False
                    else:
                        rospy.loginfo("Waiting for pose to finish...")

                    self.wait_for_action_end_or_abort()

                    success &= self.all_notifs_succeeded
                    success &= self.all_notifs_succeeded
                    poseId += 1

            except KeyboardInterrupt:
                print('Terminating...')
                pass

        # For testing purposes
        # rospy.set_param(result_topic, success)

        if not success:
            rospy.logerr("The example encountered an error.")

if __name__ == "__main__":
    ex = movTest()
    ex.main()
