#!/usr/bin/env python
# Recorder node receives commands from external nodes and launches rosbagrecord
# only on specific -bark- topics.
# This node is useful to have multiple hosts in the network without consolidating all the data
# in a single computer (reduce latency).
#
# How to use?
#
# Run the rosrecorder node (rosrun recorder recorder.py name:=<somename> )
#
# This node will subscribe to /recorder listening to commands (custom_msgs String)
#                             /recorder_std listening to commands (std_msgs String)
#           COMMANDS
#           START : start recording
#           STOP  : stop recording
#
#
# By default the recorder stores with incremental filenames 1.bag,2.bag etc.
#

import rospy
import numpy as np
import signal
import sys

import threading

import os
from EpicToolbox import FileManager,mkdirfile
from custom_msgs.msg import String
from std_msgs.msg import Header
from std_msgs.msg import String as stdString
from roshandlers.rosbag import Rosbag
from roshandlers.params import Rosparam


from datetime import date

##################### ROS MESSAGES AND PUBLISHERS ##############################
stringmsg=String()

################################################################################
print('Inicio')
states=['idle','recording']
state='idle'
# Setup a Rosbag
path=os.path.join(os.environ['HOME'],date.today().strftime('%m_%d_%y'))
mkdirfile(path)
f=FileManager(path)
allbags=f.fileList({'File':'*.bag'})
print(allbags)
nextbag=f.genList({'File':'{:01d}.bag'.format(len(allbags)+1)})
rosbag=Rosbag(path=nextbag[0])
rosparam=Rosparam('/')

############################ ROS CALLBACKS #####################################
def command_callback(msg):
    global state, rosbag, SESSION_DURATION_S,nextbag
    if msg.data=='START' and state=='idle':
        session_duration_seconds=rosparam.get('/session_duration_seconds')
        SESSION_DURATION_S= (30 if session_duration_seconds==[] else session_duration_seconds)
        allbags=f.fileList({'File':'*.bag'})
        nextbag=f.genList({'File':'{:01d}.bag'.format(len(allbags)+1)})
        rosbag=Rosbag(path=nextbag[0])
        rosbag.record()
        state='recording'
    elif msg.data=='STOP' :
        rosbag.stop()
        state='idle'

######################## HELPER FUNCTIONS ######################################
def signal_handler(sig,frame):
    ''' Terminate the connection to eris and close the node'''
    print('Ctrl+c')
    rosbag.stop()
    sys.exit(0)
signal.signal(signal.SIGINT,signal_handler)

################################################################################
''' Main loop'''
rospy.init_node('genericprotocol', anonymous=True)
cmdsub = rospy.Subscriber('/recorder',String,command_callback)
cmdsub_std = rospy.Subscriber('/recorder_std',stdString,command_callback)

ROSRATE= 1 #Hz
session_duration_seconds=rosparam.get('/session_duration_seconds')
SESSION_DURATION_S= (30 if session_duration_seconds==[] else session_duration_seconds)
rate = rospy.Rate(ROSRATE)
rate.sleep()


elapsed=0
lasttime=rospy.Time.now()

while True:

    time=rospy.Time.now()
    elapsed=time.to_sec()-lasttime.to_sec()
    print('State={}'.format(state))
    if state=='idle':
        elapsed=0
        lasttime=rospy.Time.now()
    elif state=='recording':
        #Chill until is time to switch to next ref
        print('\t{} ({}s remaining)'.format(nextbag,SESSION_DURATION_S-elapsed))
        if elapsed>SESSION_DURATION_S:
            state='idle'
            lasttime=rospy.Time.now()
            rosbag.stop()
    else:
        print('ERROR CRITICAL')

    #Wait a second
    rate.sleep()
