#! /usr/bin/env python3

import time
import threading
import numpy as np
from queue import Queue

import kinovaUtils
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient

from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2

class KinovaControls:
    TIMEOUT_DURATION = 60

    def __init__(self):
        self.ENCODING_FORMAT = 'utf-8'
        self.CONNECTION_CREDS = {
            'ip': '192.168.1.10',
            'username': 'admin',
            'password': 'admin'
        }

        self.base = None
        self.base_cyclic = None

        self.cmdQueue = Queue()
        self.isRunning = True
        self.lastGripperVal = None
        self.listenerThread = None
        self.sendCommandFeedback = None

        return None


    
    def check_for_end_or_abort(self, e):
        """Return a closure checking for END or ABORT notifications

        Arguments:
        e -- event to signal when the action is completed
            (will be set when an END or ABORT occurs)
        """
        def check(notification, e = e):
            print("EVENT : " + \
                Base_pb2.ActionEvent.Name(notification.action_event))
            if notification.action_event == Base_pb2.ACTION_END \
            or notification.action_event == Base_pb2.ACTION_ABORT:
                e.set()
        return check
    


    def move_to_home_position(self):
        if self.base == None or self.base_cyclic == None:
            print('base or base_cyclic undefined')
            return None

        # Make sure the arm is in Single Level Servoing mode
        base_servo_mode = Base_pb2.ServoingModeInformation()
        base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
        self.base.SetServoingMode(base_servo_mode)
        
        # Move arm to ready position
        print("Moving the arm to a safe position")
        action_type = Base_pb2.RequestedActionType()
        action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
        action_list = self.base.ReadAllActions(action_type)
        action_handle = None
        for action in action_list.action_list:
            if action.name == "Home":
                action_handle = action.handle

        if action_handle == None:
            print("Can't reach safe position. Exiting")
            return False

        e = threading.Event()
        notification_handle = self.base.OnNotificationActionTopic(
            self.check_for_end_or_abort(e),
            Base_pb2.NotificationOptions()
        )

        self.base.ExecuteActionFromReference(action_handle)
        finished = e.wait(self.TIMEOUT_DURATION)
        self.base.Unsubscribe(notification_handle)

        if finished:
            print("Safe position reached")
        else:
            print("Timeout on action notification wait")
        return finished



    def actuate_gripper(self, value):
        print('actuating gripper')
        # Create the GripperCommand we will send
        gripper_command = Base_pb2.GripperCommand()
        finger = gripper_command.gripper.finger.add()

        # Set speed to open gripper
        gripper_command.mode = Base_pb2.GRIPPER_POSITION
        finger.value = value
        self.base.SendGripperCommand(gripper_command)
        time.sleep(0.5)
        print('gripper actuation done')
        return None



    def move_cartesian(self, px, py, pz, gripperVal = 0.0):
        print("Starting Cartesian action movement ...")

        if(((px**2) + (py**2) + (pz**2))**0.5 >= 0.9):
            print("Infeasible pose!")
            return False

        # feedback = self.base_cyclic.RefreshFeedback()

        # actuate gripper, not working
        if(gripperVal != self.lastGripperVal):
            self.lastGripperVal = gripperVal
            self.actuate_gripper(gripperVal)
        
        action = Base_pb2.Action()
        action.name = "custom_cartesian_move"
        action.application_data = ""
        
        action.reach_pose.constraint.speed.translation = 0.5 # m/s
        action.reach_pose.constraint.speed.orientation = 30  # deg/s

        cartesian_pose = action.reach_pose.target_pose
        cartesian_pose.x = px
        cartesian_pose.y = py
        cartesian_pose.z = pz
        cartesian_pose.theta_x = 0.0 # (degrees)
        cartesian_pose.theta_y = 180.0 # (degrees)
        cartesian_pose.theta_z = 0.0 # (degrees)

        e = threading.Event()
        notification_handle = self.base.OnNotificationActionTopic(
            self.check_for_end_or_abort(e),
            Base_pb2.NotificationOptions()
        )

        print("Executing action")
        self.base.ExecuteAction(action)

        print("Waiting for movement to finish ...")
        finished = e.wait(self.TIMEOUT_DURATION)
        self.base.Unsubscribe(notification_handle)

        if finished:
            print("Cartesian movement completed")
            poseStr = f"{px} {py} {pz} {gripperVal}"
            self.sendCommandFeedback(poseStr.encode(self.ENCODING_FORMAT)) # important, always send feedback on action complete
        else:
            print("Timeout on action notification wait")
            self.sendCommandFeedback('tout'.encode(self.ENCODING_FORMAT))
        return finished



    def _startMoveThread(self):
        # Create connection to the device and get the router
        with kinovaUtils.DeviceConnection.createTcpConnection(self.CONNECTION_CREDS) as router:
            # Create required services
            self.base = BaseClient(router)
            self.base_cyclic = BaseCyclicClient(router)
            self.success = True
            # self.success &= self.move_to_home_position()
            self.move_cartesian(0.3, 0.3, 0.4, 0.0)

            while self.isRunning:
                if self.cmdQueue.empty():
                    continue
                    
                try:
                    poseStr = self.cmdQueue.get()
                    # print('got coords', poseStr)
                    cartCoords = poseStr.split(' ')
                    cartCoords = np.asarray(cartCoords, dtype=float)
                    self.move_cartesian(*cartCoords)
                except Exception as e:
                    print(e)
                    print('exception, skipping pose: %s' % poseStr)

        self.closeKinova()
        return None



    def addPoseToQueue(self, pose):
        self.cmdQueue.put(pose)
        print('added pose', pose)
        return None
    


    def getQueueSize(self):
        return self.cmdQueue.qsize()



    def initContinousListener(self):
        self.listenerThread = threading.Thread(target=self._startMoveThread, args=())
        self.listenerThread.start()
        return True



    def closeKinova(self):
        print('Closing kinova controller')
        self.isRunning = False
        if self.listenerThread:
            self.listenerThread.join(3.0)
        with self.cmdQueue.mutex:
            self.cmdQueue.queue.clear()



if __name__ == '__main__':
    print('external cmd mode')
    k = KinovaControls()
    k.initContinousListener()
    # time.sleep(3.0)
    # k.addPoseToQueue('0.1 0.3 0.3 0.7')
    while True and k.listenerThread.isAlive():
        pose = input('pose [x,y,z,g]: ')
        if pose == 'quit':
            k.closeKinova()
            break
        k.addPoseToQueue(pose)
        time.sleep(3.0)