#!/usr/bin/env python
# READ EMG FEATURE DATA FROM ERIS AND SEND TO ROS TOPICS
# read EMG from ETI Arrays and offer the option to modify the reference selection

import rospy
import numpy as np
import signal
import sys

import threading

import os
from EpicToolbox import FileManager,mkdirfile
from custom_msgs.msg import String
from std_msgs.msg import Header
from roshandlers.rosbag import Rosbag
from roshandlers.params import Rosparam

from datetime import date

##################### ROS MESSAGES AND PUBLISHERS ##############################
stringmsg=String()
commandpub = rospy.Publisher('/eris/command', String, queue_size=50)
################################################################################
print('Inicio')
states=['idle','recording','ref0','ref1','ref2','ref3','ref4','ref5','ref6','ref7']
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
    global state, rosbag, SESSION_DURATION_S
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

def SetPin(reference):
    #Set the pin reference by sending a command to eris
    stringmsg.header=Header()
    stringmsg.header.stamp=rospy.Time.now()
    stringmsg.data='NEG {:01d}'.format(reference)

################################################################################
''' Main loop'''
rospy.init_node('nextflexArrayProtocol', anonymous=True)
cmdsub = rospy.Subscriber('/arrayprotocol',String,command_callback)
ROSRATE= 1 #Hz
session_duration_seconds=rosparam.get('/session_duration_seconds')
SESSION_DURATION_S= (30 if session_duration_seconds==[] else session_duration_seconds)
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
        #Chill until it's time to switch to next ref
        if elapsed.to_sec()>SESSION_DURATION_S:
           reference=7
           SetPin(reference)
           state='ref{:01d}'.format(reference)
           lasttime=rospy.Time.now()
    elif state=='ref7':
        #Chill until is time to switch to next ref
        if elapsed.to_sec()>SESSION_DURATION_S:
            state='idle'
            lasttime=rospy.Time.now()
            rosbag.stop()
    else:
        print('ERROR CRITICAL')

    #Wait a second
    rate.sleep()
