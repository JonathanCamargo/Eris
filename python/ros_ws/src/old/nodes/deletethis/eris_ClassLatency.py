#!/usr/bin/env python
# TEST LATENCY OF CLASSIFICATION FEATURE QUERYING

import rospy
from custom_msgs.msg import featmsg, classmsg
from std_msgs.msg import Header

from eris.eris import Eris
from threading import Thread,Lock

import signal
import sys

import copy

#import Exception

dataMutex=Lock()

#ROS message for Features
classquerypub = rospy.Publisher('eris/ClassQuery', classmsg, queue_size=1)

#Gait-locations to for classification
lastClassT = 0 

def signal_handler(sig,frame):
    print('Ctrl+c')
    e.sendCommand('S_OFF')
    sys.exit(0)
signal.signal(signal.SIGINT,signal_handler)
############Callback functions############
def class_callback(msg):
    global lastClassT
    dataMutex.acquire(1)
    localT = lastClassT
    dataMutex.release()
    print("feats recieved!")
    print(rospy.Time.now().to_sec() - t0 - localT)

############Program Main body#############
rospy.init_node('talker', anonymous=True)
#Setting up callbacks
classsub = rospy.Subscriber('record/ClassFeats', featmsg, class_callback)

ROSRATE=100 #Hz
rate = rospy.Rate(ROSRATE)
t0=rospy.Time.now().to_sec()
count = 0

querymsg = classmsg()
querymsg.nextWin = 400
querymsg.maskIndex = 0

while True:
    count = count + 1 
    if (count == 100):
        count = 0
        print("queried!")
        lastClassT = rospy.Time.now().to_sec() - t0
        classquerypub.publish(querymsg)
    rate.sleep()
