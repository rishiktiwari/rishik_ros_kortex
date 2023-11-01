#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import socket

subscriberNode = 'rishik_tcpdataserver'
topicName = '/my_gen3/joint_states'
# rospy.init_node(subscriberNode, anonymous=True)
# rospy.Subscriber(topicName, String, sendStateData) #need publisher
# rospy.spin()

print('Starting Command link')
addr = ('0.0.0.0', 9997)
rcvmsg_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM) #TCP
rcvmsg_socket.bind(addr)
rcvmsg_socket.listen(5)
print('Command link listening at', addr)

(conn, addr) = rcvmsg_socket.accept()

if conn:
    print('Client connected', addr)
    while True:
        try:
            data = conn.recv(1024).decode("ascii")
            if(data == ''):
                print('Empty stream, closing Data sockets')
                break
            print('RCV CMD: ' + data)

        except:
            print('Reception failed, closing Data sockets')
            break

conn.close()
rcvmsg_socket.close()
print('Command link closed')
