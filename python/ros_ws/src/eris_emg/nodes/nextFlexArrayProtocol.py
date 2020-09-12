#!/usr/bin/env python
# READ EMG FEATURE DATA FROM ERIS AND SEND TO ROS TOPICS
# read EMG from ETI Arrays and offer the option to modify the reference selection

import rospy
import numpy as np
import signal
import sys

import threading

import os
from EpicToolbox import FileManager
from std_msgs.msg import String
from roshandlers.rosbag import Rosbag

##################### ROS MESSAGES AND PUBLISHERS ##############################
stringmsg=String()
commandpub = rospy.Publisher('/eris/command', String, queue_size=50)
################################################################################
print('Inicio')
states=['idle','recording','ref0','ref1','ref2','ref3','ref4','ref5','ref6','ref7']
state='idle'
# Setup a Rosbag
path=os.path.join(os.environ['HOME'],'09_12_2020')
f=FileManager(path)
allbags=f.fileList({'File':'*.bag'})
print(allbags)
nextbag=f.genList({'File':'{:01d}.bag'.format(len(allbags)+1)})
rosbag=Rosbag(path=nextbag[0])

############################ ROS CALLBACKS #####################################
def command_callback(msg):
    global state, rosbag
    if msg.data=='START' and state=='idle':
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

def SetPin(reference):
    #Set the pin reference by sending a command to eris
    stringmsg.data='NEG {:01d}'.format(reference)

################################################################################
''' Main loop'''
rospy.init_node('nextflexArrayProtocol', anonymous=True)
cmdsub = rospy.Subscriber('/arrayprotocol',String,command_callback)
ROSRATE= 1 #Hz
SESSION_DURATION_S=1
rate = rospy.Rate(ROSRATE)
rate.sleep()


elapsed=rospy.Duration.from_sec(0)
lasttime=rospy.Time.now()
reference=0

while True:

    time=rospy.Time.now()
    elapsed=time-lasttime

    print('State={}'.format(state))
    if state=='idle':
        pass
    elif state=='recording':
        #Setup the recording
        #Set pin reference to 0
        reference=0
        SetPin(reference)
        state='ref0'
        lasttime=rospy.Time.now()
    elif state=='ref0':
        #Chill until is time to switch to next ref
        if elapsed.to_sec()>SESSION_DURATION_S:
            reference=1
            SetPin(reference)
            state='ref{:01d}'.format(reference)
            lasttime=rospy.Time.now()
    elif state=='ref1':
        #Chill until is time to switch to next ref
        if elapsed.to_sec()>SESSION_DURATION_S:
            reference=2
            SetPin(reference)
            state='ref{:01d}'.format(reference)
            lasttime=rospy.Time.now()
    elif state=='ref2':
        #Chill until is time to switch to next ref
        if elapsed.to_sec()>SESSION_DURATION_S:
            reference=3
            SetPin(reference)
            state='ref{:01d}'.format(reference)
            lasttime=rospy.Time.now()
    elif state=='ref3':
        #Chill until is time to switch to next ref
        if elapsed.to_sec()>SESSION_DURATION_S:
            reference=4
            SetPin(reference)
            state='ref{:01d}'.format(reference)
            lasttime=rospy.Time.now()
    elif state=='ref4':
        #Chill until is time to switch to next ref
        if elapsed.to_sec()>SESSION_DURATION_S:
            reference=5
            SetPin(reference)
            state='ref{:01d}'.format(reference)
            lasttime=rospy.Time.now()
    elif state=='ref5':
        #Chill until is time to switch to next ref
        if elapsed.to_sec()>SESSION_DURATION_S:
            reference=6
            SetPin(reference)
            state='ref{:01d}'.format(reference)
            lasttime=rospy.Time.now()
    elif state=='ref6':
        #Chill until is time to switch to next ref
        if elapsed.to_sec()>SESSION_DURATION_S:
            state='idle'
            lasttime=rospy.Time.now()
            rosbag.stop()
    else:
        print('ERROR CRITICAL')

    #Wait a second
    rate.sleep()