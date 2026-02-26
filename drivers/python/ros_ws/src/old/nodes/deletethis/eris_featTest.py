#!/usr/bin/env python
# READ EMG FEATURE DATA FROM ERIS AND SEND TO ROS TOPICS

import rospy
import threading

from custom_msgs.msg import Float32, String, featmsg, classmsg, regmsg, maskmsg, Float32MultiArray
from std_msgs.msg import Header, Bool, MultiArrayDimension

from eris.eris import Eris
from threading import Thread,Lock

#from time import sleep
import numpy as np
import struct
from construct import Array,Struct,Float32l,Int8ub,this
import signal
import sys
import copy

#import Exception

dataMutex=Lock()
startFlag = False
runningFlag = False

#What to read from Eris
#Set up Eris driver to retrieve only features
features=['Gait']

featuresformat=Struct(
    "timestamp" / Float32l,
    "len" / Int8ub,
    "features" / Array(this.len,Float32l)
)

gaitFormat = Struct("Gait" / Float32l)

#if rospy.has_param('eris/port'):
#    port=rospy.get_param('eris/port')
#else:
    #port='/dev/ttyACM0'
port='/dev/ttyACM0'

##################### ROS MESSAGES AND PUBLISHERS ##############################
textmsg = String()
featuresmsg=Float32MultiArray()

regfeatpub = rospy.Publisher('record/RegFeats', Float32MultiArray, queue_size=3)
clasfeatpub = rospy.Publisher('record/ClassFeats', Float32MultiArray, queue_size=3)
classquerypub = rospy.Publisher('eris/ClassQuery', classmsg, queue_size=1)
gaitpub = rospy.Publisher('record/Gait', Float32, queue_size=3)
textpub = rospy.Publisher('eris/text', String, queue_size=1)

#Gait-locations to for classification
gaitLocs = np.array([5, 21.67, 38.33, 55, 71.67, 88.33])
lastGait = 0

################################################################################
#Create an eris object
streams=['Gait']
streamsformat=[gaitFormat]
e=Eris(streams,streamsformat,port)

######################## HELPER FUNCTIONS ######################################
def signal_handler(sig,frame):
    print('Ctrl+c')
    e.sendCommand('F_OFF')
    e.sendCommand('S_OFF')
    e.stop()
    sys.exit(0)
signal.signal(signal.SIGINT,signal_handler)

def cleanNaN(listcontainer):
    ''' Remove nans from a listcontainer'''
    data = np.array(listcontainer)
    n=np.isnan(data)
    data=data[~n];
    return data

def publishFeaturesHelper(sample, publisher):
    '''Publish features data'''
    timestamp=sample['timestamp']
    featuresmsg.data=sample['features']
    featuresmsg.header=Header(stamp=t0+rospy.Duration(timestamp))
    featuresmsg.layout.dim=[MultiArrayDimension()]
    featuresmsg.layout.dim[0].size=sample['len']
    featuresmsg.layout.dim[0].stride=1
    featuresmsg.layout.dim[0].label='index'
    publisher.publish(featuresmsg)

######################## THREAD FUNCTIONS ######################################
def publishFeatures(data, publisher):
    dataMutex.acquire(1)
    local=copy.copy(data)
    dataMutex.release()
    
    lastpacket=local[-1]
    try:
        packetdata=featuresformat.parse(lastpacket[0])
    except Exception as ex:
        print('mierda')
        print(ex)
        print(lastpacket)
        return
        #TODO publish a missing data message under eris/errors
    publishFeaturesHelper(packetdata, publisher)

def publishGait(data):
    global lastGait

    dataMutex.acquire(1)
    local=copy.copy(data)
    dataMutex.release()    

    currGait = struct.unpack('f', local[0])[0]

    gaitmsg = Float32()
    gaitmsg.data = currGait
    gaitmsg.header = Header(stamp = rospy.Time.now())

    #check to see if we need to classify
    locInd = np.argmin(abs(gaitLocs - currGait))
    loc = gaitLocs[locInd]

    if((lastGait < loc) & (currGait > loc) | (lastGait > gaitLocs[-1]) & (currGait > loc) & (locInd == 0)):
        #print([lastGait, loc, currGait])
        # if going from last to current crosses loc, or if last was near end of gc (larger than
        # the latest classif location) and the current gait is greater than the nearest location
        querymsg = classmsg()
        querymsg.time = rospy.Time.now().to_sec() - t0.to_sec()
        querymsg.maskIndex = locInd.item()
        classquerypub.publish(querymsg)
        
    gaitpub.publish(gaitmsg)

    lastGait = currGait

def publishText(data):
    for packet in enumerate(data):
        textmsg.data = packet
        textmsg.header=Header(stamp=rospy.Time.now())
        textpub.publish(textmsg)

######################## CALLBACK FUNCTIONS ##############################
def command_callback(msg):
    e.sendCommand(msg.data)

def reg_callback(msg):
    command = "REG " + str(msg.win) + " " + str(msg.inc) + " " + str(msg.maskIndex)
    e.sendCommand(command)

def class_callback(msg):
    command = "F_CLASS " + str(msg.maskIndex)
    print(command)
    e.sendCommand(command)

def mask_callback(msg):
    command = "F_MASK " + str(msg.isClassifier) + " " + str(msg.chan) + " " + str(msg.index) + " " + msg.mask
    e.sendCommand(command)

def start_callback(msg): 
    global startFlag 
    startFlag = msg.data

########################### Program Main body ###########################
rospy.init_node('biomnode', anonymous=True)
#Setting up callbacks
cmdsub = rospy.Subscriber('eris/command',String,command_callback)
regsub = rospy.Subscriber('eris/RegParams', regmsg, reg_callback)
classsub = rospy.Subscriber('eris/ClassQuery', classmsg, class_callback)
masksub = rospy.Subscriber('eris/mask', maskmsg, mask_callback)
startsub = rospy.Subscriber('eris/start', Bool, start_callback)

ROSRATE=200 #Hz
rate = rospy.Rate(ROSRATE)
e.sendCommand('TPAC 1')
e.sendCommand('S_OFF')
e.sendCommand('S_TIME')

while True:
    if startFlag:
        #Runs setup only if start command is issued and we aren't already running
        if runningFlag == False:
            e.sendCommand('TPAC 1')     #Text commands
            e.sendCommand('F_ON')       #Feature extraction enabled
            e.sendCommand('S_ON')       #Streaming enabled
            print('Inicio')
            t0=rospy.Time.now()
            count = 0
            runningFlag = True
   
        try:
            dataMutex.acquire(1)
            d=e.read()
            dataMutex.release()
        except Exception as ex:
            dataMutex.release()
            print('Problem')
            print(ex)
            e.sendCommand('S_OFF')
            rate.sleep()
            e.sendCommand('S_ON')
            rate.sleep()
            continue

        #Get data from teensy
        if type(d) is dict:
            if len(d['R'])>0:
                tr = Thread(target = publishFeatures, args = ((d['R'],), regfeatpub))             
                tr.start()
            if len(d['C'])>0:
                tc = Thread(target = publishFeatures, args = ((d['C'],),clasfeatpub))
                tc.start()
            if len(d['D'])>0:
                td = Thread(target = publishGait, args = (d['D'],))
                td.start()
            if len(d['T'])>0:
                tt = Thread(target = publishText, args = (d['T'],))
                tt.start()
    
    #If we are running but we're not supposed to be, stop
    elif runningFlag == True:
        e.sendCommand('F_OFF')
        e.sendCommand('S_OFF')
        runningFlag = False

    rate.sleep()
    
