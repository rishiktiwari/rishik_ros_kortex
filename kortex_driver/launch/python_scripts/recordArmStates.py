#!/usr/bin/env python3

import rospy
# import pandas as pd
import json
import time
from datetime import datetime
from sensor_msgs.msg import JointState

LOG_DIR_PATH = "/home/rishik/catkin_workspace/src/ros_kortex/kortex_driver/launch/python_scripts/recordings/"
SUBSAMPLE_SKIP_FRAMES = 5
samples_skipped = 0
log_file = None
flag_first_entry = True
script_start_time = None



def callback(jointState):
    global samples_skipped, flag_first_entry

    if (samples_skipped == SUBSAMPLE_SKIP_FRAMES):
        samples_skipped = 0

        entry_prefix = ",\n\t"
        if flag_first_entry:
            entry_prefix = "\t"
            flag_first_entry = False

        data = {
            "time_s": round(time.perf_counter() - script_start_time, 3)
        }        
        for i in range(0, len(jointState.name), 1):
            data[jointState.name[i] + "--pos"] = jointState.position[i]
            data[jointState.name[i] + "--vel"] = jointState.velocity[i]
            data[jointState.name[i] + "--effort"] = jointState.effort[i]

        log_file.write(entry_prefix + json.dumps(data))
        return None
    
    samples_skipped += 1
    return None



def init():
    global log_file, script_start_time
    dt = datetime.now().strftime("%d-%b-%Y__%H-%M-%S")
    log_file_name = LOG_DIR_PATH + "log__" + dt + ".json"

    try:
        log_file = open(log_file_name, "a")
        print("--- writing to file: %s\n" % log_file_name)
        log_file.write('[\n')
        print("Recording...\n")

        rospy.init_node('py_data_recorder', anonymous=True)
        rospy.Subscriber('/my_gen3/joint_states', JointState, callback)
        script_start_time = time.perf_counter()
        rospy.spin()
    except Exception or KeyboardInterrupt as e:
        print('Interrupted', e)
    finally:
        if (log_file != None):
            log_file.write("\n]\n")
            log_file.close()
            print("\n\n--- closed file: %s\n\n" % log_file_name)
        print("\nexit!\n")

if __name__ == '__main__':
    init()
