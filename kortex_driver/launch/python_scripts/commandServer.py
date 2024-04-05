#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import socket

HOST_ADDR = ('0.0.0.0', 9999)
ENCODING_FORMAT = 'utf-8'
HEADER_SIZE = 1024

subscriberNode = 'rishik_commander'
topicName = '/my_gen3/joint_states'
# rospy.init_node(subscriberNode, anonymous=True)
# rospy.Subscriber(topicName, String, sendStateData) # TODO: create publisher
# rospy.spin()

print('Starting command link...')

rcvmsg_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM) #TCP
rcvmsg_socket.bind(HOST_ADDR)
rcvmsg_socket.listen(5)

print('Command link listening at', HOST_ADDR)

(client_socket, client_addr) = rcvmsg_socket.accept()

if client_socket:
    print('Client connected', client_addr)
    while True:
        try:
            msg = client_socket.recv(HEADER_SIZE).decode(encoding=ENCODING_FORMAT)
            if (msg == ''):
                continue
            elif(msg == 'quit'):
                print('> QUIT RCVD <')
                break
            
            print('RCVD CMD: \n>\t%s' % msg)

        except Exception or KeyboardInterrupt:
            print('Exception occured, closing socket')
            break


print('Quitting...')
client_socket.shutdown(socket.SHUT_RDWR)
rcvmsg_socket.shutdown(socket.SHUT_RDWR)
client_socket.close()
rcvmsg_socket.close()
# rospy.signal_shutdown('')
print('Command link closed')
