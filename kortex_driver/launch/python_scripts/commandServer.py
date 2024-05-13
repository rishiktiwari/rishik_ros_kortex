#!/usr/bin/env python

import socket
import threading
import json
from kinova import KinovaControls           # USES ROS-KORTEX DRIVER
# from kinovaViaAPI import KinovaControls   # USES KORTEX API

class CommandServer:
    def __init__(self):
        self.HOST_ADDR = ('0.0.0.0', 9999)
        self.ENCODING_FORMAT = 'utf-8'
        self.HEADER_SIZE = 64
        self.DELIMITER = '[CMD]'
        
        self.isRunning = True
        self.rcvmsg_socket = None
        self.client_socket = None
        self.client_addr = None

        self.kinovactrl = KinovaControls()



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



    def _getWordIndex(self, word, text, skip = 0):
        i = -1
        try:
            i = str(text).index(word, skip)
        except ValueError:
            pass
        return i



    def _recvMsgs(self):
        buffer = ''
        pack_start = -1
        pack_end = -1
        pack_delimiter_len = len(self.DELIMITER)

        empty_buf_count = 0
        while self.isRunning:
            try:
                buffer += self.client_socket.recv(self.HEADER_SIZE).decode(encoding=self.ENCODING_FORMAT)
                if(buffer == ''):
                    empty_buf_count += 1

                if(empty_buf_count >= 10):
                    print('Too many empty packets, closing socket')
                    break
                
                empty_buf_count = 0 # reset, buf has some data
                pack_start = self._getWordIndex(self.DELIMITER, buffer)
                pack_end = pack_start + self.HEADER_SIZE if pack_start >= 0 else -1

                if(pack_start != -1 and len(buffer) >= pack_end): # extract data
                    data_start = pack_start+8 # skips first 8 bytes because has data length
                    
                    # extract length of data, always in first 8 bytes, strip filler (.) bytes
                    data_len = int(buffer[pack_start+pack_delimiter_len : data_start].strip('.'))

                    msg = buffer[data_start : data_start+data_len] # pack_end may be greater than pack_start+data_len due to filler bytes
                    buffer = buffer[pack_end:] # remove this package from buffer
                    print('RCVD CMD: \n>\t%s' % msg)

                    if (msg.strip('.') == ''):
                        continue

                    msg = json.loads(msg)
                    if(msg['pose'] == 'quit'):
                        print('> QUIT RCVD <')
                        break
                    
                    self.kinovactrl.addPoseToQueue(msg)
                    print('In queue: %d' % self.kinovactrl.getQueueSize())

            except Exception or KeyboardInterrupt as e:
                print(e)
                print('Exception occured, closing socket')
                self.isRunning = False
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
    print("""\n
	+--------------------------------------------------------+
	|                                                        |
	|   ..::Victoria University, Melbourne, Australia::..    |
	|                     -- May 2024 --                     |
	|                                                        |
	| LLM-CV powered robotic arm manipulator for Kinova Gen3 |
	|                                                        |
	| Developed by Rishik R. Tiwari                          |
	|              techyrishik[at]gmail[dot]com              |
	|                                                        |
	| LLM: Microsoft Phi-3-mini-q4-GGUF                      |
	| VLM: OpenAI CLIP                                       |
	|                                                        |
	| Tested on: Apple Macbook Pro (M1, 16GB)                |
	| Remark: Stable 1Hz inference                           |
	|                                                        |
	+--------------------------------------------------------+
	\n\n\n""")
    cmdSrv = CommandServer()
    cmdSrv.initServer()
