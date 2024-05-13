#!/usr/bin/env python

import rospy
import time
import json
import threading
import numpy as np
from Queue import Queue

from std_msgs.msg import String
from kortex_driver.srv import *
from kortex_driver.msg import *

class KinovaControls:
    def __init__(self):
        self.ENCODING_FORMAT = 'utf-8'
        self.cmdQueue = Queue()
        self.isRunning = True
        self.lastGripperVal = -1.0
        self.listenerThread = None
        self.sendCommandFeedback = None

        try:
            rospy.init_node('kinova_rishik')

            self.HOME_ACTION_IDENTIFIER = 2

            self.action_topic_sub = None
            self.all_notifs_succeeded = True
            self.poseId = 1

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
        
        success = self.is_init_success
        success &= self.clear_errs()
        # success &= self.home_the_robot()
        success &= self.setReferenceToCartesian()
        success &= self.subscribeRobotNotification()

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

    def clear_errs(self):
        try:
            self.clear_faults()
        except rospy.ServiceException:
            rospy.logerr("Failed to call ClearFaults")
            return False
        else:
            rospy.loginfo("Cleared the faults successfully")
            # rospy.sleep(2.5)
            return True

    def home_the_robot(self):
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

    def setReferenceToCartesian(self):
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


    def subscribeRobotNotification(self):
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
    

    def actuate_gripper(self, value):
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

    def move_cartesian(self, px, py, pz, gripperVal = 0.0, sendFeedback = False):
        # For testing purposes
        success = self.is_init_success

        if success:
            # success &= self.clear_errs()
            # success &= self.home_the_robot()
            # success &= self.setReferenceToCartesian()
            # success &= self.subscribeRobotNotification()

            # POSITION DEFINITION
            my_cartesian_speed = CartesianSpeed()
            my_cartesian_speed.translation = 0.2 # m/s
            my_cartesian_speed.orientation = 15  # deg/s

            try:
                if(((px**2) + (py**2) + (pz**2))**0.5 >= 0.9):
                    print("Infeasible pose!")
                    return False
                
                # actuate gripper
                if(gripperVal != self.lastGripperVal):
                    print('actuating gripper')
                    self.actuate_gripper(gripperVal)
                    self.lastGripperVal = gripperVal
                    time.sleep(1) # required for action to complete
                
                print('preparing pose')
                my_constrained_pose = ConstrainedPose()
                my_constrained_pose.target_pose.x = px
                my_constrained_pose.target_pose.y = py
                my_constrained_pose.target_pose.z = pz
                my_constrained_pose.target_pose.theta_x = 0     # 0deg - points down
                my_constrained_pose.target_pose.theta_y = 180   # 180deg - camera is outward and straight
                my_constrained_pose.target_pose.theta_z = 0     # 0deg - points toward +ve y-axis
                my_constrained_pose.constraint.oneof_type.speed.append(my_cartesian_speed)

                req = ExecuteActionRequest()
                req.input.oneof_action_parameters.reach_pose.append(my_constrained_pose)
                req.input.name = "pose" + str(self.poseId)
                req.input.handle.action_type = ActionType.REACH_POSE
                req.input.handle.identifier = 1000 + self.poseId

                rospy.loginfo("Sending pose " + str(self.poseId) + "...")
                self.last_action_notif_type = None
                try:
                    self.execute_action(req)
                except rospy.ServiceException:
                    rospy.logerr("Failed to send pose "+str(self.poseId))
                    success = False

                self.wait_for_action_end_or_abort()
                if sendFeedback == True:
                    # self.sendCommandFeedback('done')
                    poseStr = "%.2f %.2f %.2f %.2f" % (px, py, pz, gripperVal)
                    self.sendCommandFeedback(poseStr.encode(self.ENCODING_FORMAT))
                self.poseId += 1

            except KeyboardInterrupt:
                print('Terminating...')
                self.isRunning = False
                pass

        if not success:
            rospy.logerr("The program encountered an error.")
            self.isRunning = False
        

    def _startMoveThread(self):
        try:
            while self.isRunning:
                if self.cmdQueue.empty():
                    continue
                
                commandJson = self.cmdQueue.get()
                cartCoords = commandJson["pose"].split(' ')
                cartCoords = np.asarray(cartCoords, dtype=float)
                self.move_cartesian(*cartCoords, sendFeedback=commandJson["feedback"])
        except Exception or KeyboardInterrupt as e:
            print('exception, terminating movement controller\n|>\t', e)
        finally:
            self.closeKinova()
        return None


    def addPoseToQueue(self, pose):
        self.cmdQueue.put(pose)
        return None

    
    def getQueueSize(self):
        return self.cmdQueue.qsize()


    def initContinousListener(self):
        self.listenerThread = threading.Thread(target=self._startMoveThread, args=())
        self.listenerThread.start()
        return True


    def closeKinova(self):
        if self.isRunning == True: # to prevent recursive calls
            print('Closing kinova controller')
            self.isRunning = False
            with self.cmdQueue.mutex:
                self.cmdQueue.queue.clear()
