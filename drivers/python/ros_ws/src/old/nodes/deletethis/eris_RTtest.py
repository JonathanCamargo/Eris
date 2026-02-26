#!/usr/bin/env python
# READ EMG FEATURE DATA FROM ERIS AND SEND TO ROS TOPICS
# Need to: Finalize mask file format, come up with way of handling multiple teensy's

import rospy

from custom_msgs.msg import Float32, classmsg, regmsg, maskmsg
from std_msgs.msg import Header, Bool

from eris.eris import Eris
from threading import Thread,Lock

#from time import sleep
import numpy as np
import struct
import signal
import sys
import os

import copy

#import Exception

startFlag = False
exitFlag = False 

#What to read from Eris
#Set up Eris driver to retrieve only features
NUMFEATS=9 #How many features 
NUMCHANNELS=8

#ROS messages
maskMsg = maskmsg() 
classMsg = classmsg()
regMsg = regmsg() 
startmsg = Bool() 
startmsg.data = True

#Publishers
classquerypub = rospy.Publisher('eris/ClassQuery', classmsg, queue_size=3)
regpub = rospy.Publisher('eris/RegParams', regmsg, queue_size=3)
maskpub = rospy.Publisher('eris/mask', maskmsg, queue_size=50)
startpub = rospy.Publisher('eris/start', Bool, queue_size=3)

#Mask file locations - each device has a file for regression and a file for classification
maskFile = os.getenv("C") + "/Eris/test_file.txt"
print(maskFile)

#Gait-locations to for classification
gaitLocs = np.array([5, 21.67, 38.33, 55, 71.67, 88.33])
classifWins = np.array([500, 500, 500, 400, 400, 400])
lastGait = 0

def signal_handler(sig,frame):
    print('Ctrl+c')
    #Send Stop Signal to other nodes
    startmsg.data = False
    startpub.publish(startmsg)
    #Shutdown this node
    rospy.signal_shutdown("Master shutdown!")
    sys.exit(0)

signal.signal(signal.SIGINT,signal_handler)

def cleanNaN(listcontainer):
    ''' Remove nans from a listcontainer'''
    data = np.array(listcontainer)
    n=np.isnan(data)
    data=data[~n];
    return data
#########Initialization Helpers###########
def maskHelper(filename, publisher):
    file_object = open(filename, 'r')
    lines = file_object.readlines() 
    for line in lines:
        numbers = line.split(',') 
        maskMsg.isClassifier = int(numbers[0])
        maskMsg.chan = int(numbers[1])
        maskMsg.index = int(numbers[2])
        maskMsg.mask = numbers[3]
        publisher.publish(maskMsg)
    file_object.close()

###############CallBacks##################
def gait_callback(msg):
    global lastGait
    currGait = msg.data
    # check to see if we need to classify
    locInd = np.argmin(abs(gaitLocs - currGait))
    loc = gaitLocs[locInd]

############Program Main body#############
rospy.init_node('talker', anonymous=True, disable_signals=True)
ROSRATE=100 #Hz
rate = rospy.Rate(ROSRATE)

#Setting up callbacks
gaitsub = rospy.Subscriber('record/Gait', Float32, gait_callback)

#Setup
maskHelper(maskFile, maskpub)

rospy.sleep(1) #wait for other nodes to get started up
print('Real time master has started')
t0=rospy.Time.now().to_sec()
count = 0

startpub.publish(startmsg)
print(startmsg)

while True:
    #Get data from teensy
    rate.sleep()
