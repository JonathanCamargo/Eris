#!/usr/bin/env python
# READ SINEWAVE DATA FROM ERIS AND SEND TO ROS TOPICS

import rospy
import threading

from custom_msgs.msg import String, Float32, Bool 
from std_msgs.msg import Header

from eris.eris import Eris
from eris.customtypes import EMGSample_t, floatSample_t, FSRSample_t, uint8_tSample_t

from threading import Thread,Lock

from construct import Array,Struct,Float32l,Int8ub,this
import numpy as np
import signal
import sys
import copy

dataMutex=Lock()

#What to read from Eris?

#Upcoming data is length and samples

sineformat=Struct(
    "len" / Int8ub,
    "sinedata" / Array(this.len,floatSample_t)
)

fsrformat=Struct(
    "len" / Int8ub,
    "fsrdata" / Array(this.len,floatSample_t)
)


if rospy.has_param('eris/port'):
    port=rospy.get_param('eris/port')
else:
    port='/dev/ttyACM0'

##################### ROS MESSAGES AND PUBLISHERS ##############################
sinemsg=Float32()
fsrmsg=Float32()

textpub = rospy.Publisher('/record/eris/print', String, queue_size=50)
sinepub = rospy.Publisher('/record/eris/sine', Float32, queue_size=50)
fsrpub = rospy.Publisher('/record/eris/fsr', Float32, queue_size=50)

t0=0 #global variable to store time reference to linux time
def publishFSR(sample):
    '''Publish data for FSR'''
    timestamp=sample['timestamp']/1000.0
    fsrmsg.header=Header(stamp=t0+rospy.Duration(timestamp))
    fsrmsg.data=sample['value']
    fsrpub.publish(fsrmsg)

def publishSine(sample):
    '''Publish data for Sine'''
    timestamp=sample['timestamp']/1000.0
    sinemsg.header=Header(stamp=t0+rospy.Duration(timestamp))
    sinemsg.data=sample['value']
    sinepub.publish(sinemsg)

def publishText(data):
    textmsg.data = data[0];
    textmsg.header=Header(stamp=rospy.Time.now())
    textpub.publish(textmsg)

def command_callback(msg):
    ''' A callback to transmit a command to eris'''
    e.sendCommand(msg.data)

################################################################################
#Create an eris object
streams=['EMG']
streamsformat=[sineformat]
e=Eris(streams,streamsformat,port)

######################## HELPER FUNCTIONS ######################################
def signal_handler(sig,frame):
    ''' Terminate the connection to eris and close the node'''
    print('Ctrl+c')
    e.sendCommand('S_OFF')
    e.sendCommand('F_OFF')
    sys.exit(0)
signal.signal(signal.SIGINT,signal_handler)

def publishData(data):
    ''' Get all the data in binary packetform, parse it and publish
    to the topics'''
    #Make a local copy of the data
    dataMutex.acquire(1)
    local=copy.copy(data)
    dataMutex.release()
    for packetIdx,packet in enumerate(local):
        try:
            packetData=e.parse(packet)
        except Exception as ex:
            print('mierda')
            print(ex)
            print(packet)
            #TODO publish a missing data message under eris/errors
            continue

        sinedata=packetData['SINE']['sinedata']
        n=packetData['SINE']['len']
        for sample in sinedata: #all channels should have same lenght
            publishSine(sample)

        #fsrdata=packetData['FSR']['fsrdata']
        #n=packetData['FSR']['len']
        #for sample in fsrdata: #all channels should have same lenght
        #    publishFSR(sample)


################################################################################

''' Main loop'''
dataMutex=Lock();
rospy.init_node('readerisnode', anonymous=True)
cmdsub = rospy.Subscriber('eris/command',String,command_callback)

ROSRATE=50 #Hz
rate = rospy.Rate(ROSRATE)
e.sendCommand('S_OFF')
e.sendCommand('S_T0') #Reset time to 0
t0=rospy.Time.now()
rate.sleep()

print('Inicio')

# Transmit a time stamp to eris to sync with pi time ??
e.start()

while True:
    #Get data from teensy
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

    if type(d) is dict:
        if len(d['T'])>0:
            t=Thread(target=publishText, args=(d['T'],))
            t.start()
        if len(d['D'])>0:
            #Threaded option (not necessary if just using ros)
            #t2=Thread(target=publishData,args=(d['D'],))
            #t2.start()
            publishData(d['D'])
        if len(d['R'])>0:
            #Regression features
            regression(d['R'])

    #rospy.loginfo_once("This message will print only once")
    rate.sleep()
e.sendCommand('S_OFF')
e.stop()


