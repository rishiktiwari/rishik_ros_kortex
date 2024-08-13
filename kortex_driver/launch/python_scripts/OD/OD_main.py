#!/usr/bin/env python3

# import threading
from time import sleep
import rospy

from OD_cupPnP_gen3Lite import Gen3Lite_cupPnP
from OD_dispenserPnP_gen3 import Gen3_dispenserPnP

class ODMain:
    def __init__(self):
        print('Launching main arm controller...')
        print('Developed at Victoria University, Melbourne by:\n\tRishik Tiwari\n\tDevin Johnson\n\n')
        
        rospy.init_node('od_pnp_demo')
        
        self.gen3lite_controller = Gen3Lite_cupPnP()
        self.gen3_controller = Gen3_dispenserPnP()

        self.gen3_controller.init()
        print('--- gen3 ctrl loaded ---')
        sleep(2.0)
        self.gen3lite_controller.init()
        print('--- gen3lite ctrl loaded ---')
        sleep(2.0)

        print('--- All arm controllers loaded ---')



    def executeRequest(self, kitkatQty, CookieQty) -> bool:
        '''
        To invoke action request programmatically.
        '''
        try:
            if not self.gen3lite_controller.startTask():
                print('---\n\tGEN 3 LITE CONTROLLER ERR!\n---')
                return False
            
            if not self.gen3_controller.startTask(kitkatQty, CookieQty):
                print('---\n\tGEN 3 CONTROLLER ERR!\n---')
                return False
    
            print('---\n\tALL ACTIONS COMPLETED!\n---')

        except Exception as e:
            print('An error occured, terminating...\nE: ', e)
            return False 
        return True



    def start(self) -> None:
        '''
        A forever while loop for command line input
        '''

        while True:
            uinp = input("\nStart action[enter] / type 'quit' to end: ").lower().strip()
            if(uinp == 'quit'):
                print("Quitting...")
                break
            elif (uinp != ''):
                continue
            
            numOfKitkat = 0
            numOfCookie = 0
            try:
                numOfKitkat = int(input("KitKat qantity: ").strip())
                numOfCookie = int(input("Cookie quantity: ").strip())
            except ValueError:
                print('Invalid input, pls try again!\n')
                continue

            if not self.executeRequest(numOfKitkat, numOfCookie):
                break
        return None



if __name__ == "__main__":
    mainProgram = ODMain()
    mainProgram.start()



