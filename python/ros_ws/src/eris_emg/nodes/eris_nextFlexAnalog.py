#!/usr/bin/env python
# READ EMG FEATURE DATA FROM ERIS AND SEND TO ROS TOPICS
# read EMG and ETI only

import rospy
import threading

from custom_msgs.msg import EMG2CH, String, Float32, Bool, Float32MultiArray, ETI
from std_msgs.msg import Header, MultiArrayDimension

from eris.eris import Eris
from eris.customtypes import EMG2CHSample_t, floatSample_t, uint8_tSample_t

from threading import Thread,Lock

from construct import Array,Struct,Float32l,Int8ub,this
import numpy as np
import signal
import sys
import copy

dataMutex=Lock()

#What to read from Eris?

#Upcoming data is length and samples
emgformat=Struct(
    "len" / Int8ub,
    "emgdata" / Array(this.len,EMG2CHSample_t)
)

sineformat=Struct(
    "len" / Int8ub,
    "sinedata" / Array(this.len,floatSample_t)
)


tiSample_t = Struct(
    "timestamp" / Float32l,
    "temperature" / Float32l[1],
    "impedance" / Float32l[1]
    )

etiformat=Struct(
    "len" / Int8ub,
    "etidata" / Array(this.len,tiSample_t)
)


if rospy.has_param('eris/port'):
    port=rospy.get_param('eris/port')
else:
    port='/dev/ttyACM0'

##################### ROS MESSAGES AND PUBLISHERS ##############################
emgmsg=EMG2CH()
sinemsg=Float32()
etimsg=ETI()

featuresmsg=Float32MultiArray()

textpub = rospy.Publisher('/eris/print', String, queue_size=50)
sinepub = rospy.Publisher('/eris/sine', Float32, queue_size=50)
etipub = rospy.Publisher('/eris/ti', ETI, queue_size=50)
emgpub = rospy.Publisher('/eris/emg', EMG2CH, queue_size=100)
featurespub = rospy.Publisher('/record/eris/features', Float32MultiArray, queue_size=1)

t0=0 #global variable to store time reference to linux time
def publishEMG(sample):
    '''Publish data for EMG'''
    timestamp=sample['timestamp']/1000.0
    #print(timestamp)
    emgmsg.header=Header(stamp=t0+rospy.Duration(timestamp))
    emgmsg.ch0=sample['ch'][0]
    emgmsg.ch1=sample['ch'][1]
    #emgmsg.ch2=sample['ch'][2]
    #emgmsg.ch3=sample['ch'][3]
    #emgmsg.ch4=sample['ch'][4]
    #emgmsg.ch5=sample['ch'][5]
    #emgmsg.ch6=sample['ch'][6]
    #emgmsg.ch7=sample['ch'][7]
    emgpub.publish(emgmsg)
    #print(emgmsg)

def publishSine(sample):
    '''Publish data for Sine'''
    timestamp=sample['timestamp']/1000.0
    sinemsg.header=Header(stamp=t0+rospy.Duration(timestamp))
    sinemsg.data=sample['value']
    sinepub.publish(sinemsg)

def publishETI(sample):
    '''Publish data for ETI'''
    timestamp=sample['timestamp']/1000.0
    etimsg.header=Header(stamp=t0+rospy.Duration(timestamp))
    etimsg.temperature=sample['temperature'][0]
    etimsg.impedance=sample['impedance'][0]
    etipub.publish(etimsg)

def publishText(data):
    textmsg.data = data[0];
    textmsg.header=Header(stamp=rospy.Time.now())
    textpub.publish(textmsg)

def publishFeatures(sample):
    '''Publish features data'''
    timestamp=sample['timestamp']/1000.0
    featuresmsg.data=sample['features']
    featuresmsg.header=Header(stamp=t0+rospy.Duration(timestamp))
    featuresmsg.layout.dim=[MultiArrayDimension()]
    featuresmsg.layout.dim[0].size=sample['len']
    featuresmsg.layout.dim[0].stride=1
    featuresmsg.layout.dim[0].label='index'
    featurespub.publish(featuresmsg)

def command_callback(msg):
    ''' A callback to transmit a command to eris'''
    e.sendCommand(msg.data)

################################################################################
#Create an eris object
streams=['SINE','EMG','ETI']
streamsformat=[sineformat,emgformat,etiformat]
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

        emgdata=packetData['EMG']['emgdata']
        n=packetData['EMG']['len']
        for sample in emgdata: #all channels should have same lenght
            publishEMG(sample)

        sinedata=packetData['SINE']['sinedata']
        n=packetData['SINE']['len']
        for sample in sinedata: #all channels should have same lenght
            publishSine(sample)

        etidata=packetData['ETI']['etidata']
        n=packetData['ETI']['len']
        for sample in etidata: #all channels should have same lenght
            publishETI(sample)
     

def regression(data):
    ''' Read the features sent for regression and run a model '''
    #Make a local copy of the data
    dataMutex.acquire(1)
    local=copy.copy(data)
    dataMutex.release()

    #Only do it for the last packet as older info is not important
    lastpacket=local[-1]
    try:
        packetdata=featuresformat.parse(lastpacket)
    except Exception as ex:
        print('mierda')
        print(ex)
        print(lastpacket)
        return
        #TODO publish a missing data message under eris/errors
    publishFeatures(packetdata)



################################################################################

''' Main loop'''
dataMutex=Lock();
rospy.init_node('imunode', anonymous=True)
cmdsub = rospy.Subscriber('eris/command',String,command_callback)

ROSRATE=50 #Hz
rate = rospy.Rate(ROSRATE)
e.sendCommand('S_OFF')
e.sendCommand('S_T0') #Reset time to 0
t0=rospy.Time.now()
rate.sleep()

print('Inicio')

e.sendCommand('F_ON') #Enable features calculation and transmission
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
