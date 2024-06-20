#! /usr/bin/env python3

import time
from datetime import datetime
import threading

import kinovaUtils
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient

from kortex_api.autogen.messages import Base_pb2

class FeedbackTest:
    TIMEOUT_DURATION = 60
    LOG_DIR_PATH = "/home/rishik/catkin_workspace/src/ros_kortex/kortex_driver/launch/python_scripts/recordings/"
    
    log_file = None
    log_file_name = ''
    script_start_time = None

    def __init__(self):
        self.ENCODING_FORMAT = 'utf-8'
        self.CONNECTION_CREDS = {
            'ip': '192.168.1.10',
            'username': 'admin',
            'password': 'admin'
        }

        self.base = None
        self.base_cyclic = None
        self.isRunning = True
        self.listenerThread = None
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



    def startAcquisition(self):
        # Create connection to the device and get the router
        try:
            with kinovaUtils.DeviceConnection.createTcpConnection(self.CONNECTION_CREDS) as router:
                # Create required services
                self.base = BaseClient(router)
                self.base_cyclic = BaseCyclicClient(router)
                self.success = True
                # self.success &= self.move_to_home_position()
                self.script_start_time = time.perf_counter()

                tempCounter = 0
                rowBuf = []

                while True:
                    base_feedback = self.base_cyclic.RefreshFeedback()
                    
                    # timeSec
                    rowBuf.append(str(round(time.perf_counter() - self.script_start_time, 3))) 

                    # add joint values
                    for actuator in base_feedback.actuators:
                        rowBuf.append(str(actuator.torque))
                        rowBuf.append(str(actuator.current_motor))

                    # add gripper values
                    rowBuf.append(str(base_feedback.interconnect.gripper_feedback.motor[0].position))
                    rowBuf.append(str(base_feedback.interconnect.gripper_feedback.motor[0].current_motor))

                    # write to file
                    self.log_file.write(','.join(rowBuf) + '\n')
                    tempCounter += 1
                    rowBuf = []

                    if (tempCounter == 20): # flush every second
                        self.log_file.flush()
                        tempCounter = 0

                    # print(base_feedback.actuators[1].torque) # shoulder joint
                    # print(base_feedback.actuators[2].torque) # elbow joint
                    # print(base_feedback.actuators[4].torque) # wrist hinge joint
                    # print(base_feedback.interconnect.voltage)
                    # print(base_feedback.interconnect.gripper_feedback.motor[0].position)
                    # print(base_feedback.interconnect.gripper_feedback.motor[0].voltage)
                    # print(base_feedback.interconnect.gripper_feedback.motor[0].current_motor)
                    time.sleep(0.05)
                        
        except Exception as e:
            print("Exception", e)      
        finally:
            self.closeKinova()

        return None



    def initContinousListener(self):
        dt = datetime.now().strftime("%d-%b-%Y__%H-%M-%S")
        self.log_file_name = self.LOG_DIR_PATH + "feedbackLog__" + dt + ".csv"

        cols = [
            'timeSec',
            'j0_T', 'j0_I',
            'j1_T', 'j1_I',
            'j2_T', 'j2_I',
            'j3_T', 'j3_I',
            'j4_T', 'j4_I',
            'j5_T', 'j5_I',
            'gripper_pos', 'gripper_I'
        ]
        colsRow = ','.join(cols) + '\n'

        try:
            self.log_file = open(self.log_file_name, "a")
            print("--- writing to file: %s\n" % self.log_file_name)
            self.log_file.write(colsRow)
            print("Recording...\n")
            self.startAcquisition()

        except Exception or KeyboardInterrupt as e:
            print('Interrupted', e)
            self.closeKinova()

        return True



    def closeKinova(self):
        if self.isRunning == True: # to prevent recursive calls
            print('Closing kinova controller')
            self.isRunning = False
            if (self.log_file != None):
                self.log_file.close()
                print("\n\n--- closed file: %s\n\n" % self.log_file_name)
        print("\nexit!\n")
        exit(1)



if __name__ == '__main__':
    print('external mode')
    k = FeedbackTest()
    k.initContinousListener()
