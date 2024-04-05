#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
import pickle, socket

subscriberNode = 'rishik_tcpdataserver'
topicName = '/my_gen3/joint_states'

print('Starting data link')
addr = ('0.0.0.0', 9998)
rcvmsg_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM) #TCP
rcvmsg_socket.bind(addr)
rcvmsg_socket.listen(5)
print('Data link listening at', addr)

(conn, addr) = rcvmsg_socket.accept()

def sendStateData(msg):
    global conn
    data = {
        "name": msg.name,
        "pos": msg.position,
        "vel": msg.velocity
    }
    # print(data)
    try:
        check = conn.sendall(pickle.dumps(data))
        if(check != None):
            print("Data send incomplete/failed")

    except Exception as e:
        print(e)
        print('Client unavailable, terminating')
        rospy.signal_shutdown('')


if conn:
    print('Client connected ' + addr[0])
    rospy.init_node(subscriberNode, anonymous=True)
    rospy.Subscriber(topicName, JointState, sendStateData)
    rospy.spin()


conn.close()
rcvmsg_socket.close()
print('Data link closed')
