#!/usr/bin/env python

import socket
import threading
# import rospy
# import numpy as np
# from kinova import KinovaControls           # USES ROS-KORTEX DRIVER
from kinovaViaAPI import KinovaControls   # USES KORTEX API

class CommandServer:
    def __init__(self):
        self.HOST_ADDR = ('0.0.0.0', 9999)
        self.ENCODING_FORMAT = 'utf-8'
        self.HEADER_SIZE = 32
        
        self.isRunning = True
        self.rcvmsg_socket = None
        self.client_socket = None
        self.client_addr = None

        self.kinovactrl = KinovaControls()
        # self.subscriberNode = 'rishik_commander'
        # self.topicName = '/my_gen3/joint_states'
        # rospy.init_node(subscriberNode, anonymous=True)
        # rospy.Subscriber(topicName, String, sendStateData) # TODO: create publisher
        # rospy.spin()


    def initServer(self):
        print('Starting command link...')
        recvMsgThread = None
        try:
            self.rcvmsg_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM) #TCP
            self.rcvmsg_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.rcvmsg_socket.settimeout(600.0)
            self.rcvmsg_socket.bind(self.HOST_ADDR)
            self.rcvmsg_socket.listen(5)
            print('Command link listening at', self.HOST_ADDR)

            (self.client_socket, self.client_addr) = self.rcvmsg_socket.accept()
            self.client_socket.settimeout(600.0)
            if self.client_socket:
                print('Client connected', self.client_addr)
                recvMsgThread = threading.Thread(target=self._recvMsgs)
                recvMsgThread.start()
                self.kinovactrl.initContinousListener()
                self.kinovactrl.sendCommandFeedback = self.client_socket.sendall
        
        except KeyboardInterrupt:
            print('KbdInrpt, closing socket')
            recvMsgThread.join(5.0)
            self.closeSocket()


    def _recvMsgs(self):
        while self.isRunning:
            try:
                msg = self.client_socket.recv(self.HEADER_SIZE).decode(encoding=self.ENCODING_FORMAT)
                if (msg == ''):
                    continue
                elif(msg == 'quit'):
                    print('> QUIT RCVD <')
                    break
                print('RCVD CMD: \n>\t%s' % msg)
                self.kinovactrl.addPoseToQueue(msg)
                print('In queue: %d' % self.kinovactrl.getQueueSize())

            except Exception or KeyboardInterrupt as e:
                print(e)
                print('Exception occured, closing socket')
                break

        self.closeSocket()


    def closeSocket(self):
        print('Quitting...')
        self.isRunning = False
        self.kinovactrl.closeKinova()
        self.client_socket.shutdown(socket.SHUT_RDWR)
        self.rcvmsg_socket.shutdown(socket.SHUT_RDWR)
        self.client_socket.close()
        self.rcvmsg_socket.close()
        # rospy.signal_shutdown('')
        print('Command link closed')



if __name__ == '__main__':
    cmdSrv = CommandServer()
    cmdSrv.initServer()
