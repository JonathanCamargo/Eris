#!/usr/bin/env python
'''Read imu data from Eris'''

from eris.eris import Eris
from eris.customtypes import FSR2CHSample_t , floatSample_t, uint8_tSample_t

import rospy
import threading

from custom_msgs.msg import FSR2CH,String,Float32,Uint8
from std_msgs.msg import Header
from threading import Thread,Lock

#Use construct to create a good c format
from construct import Array,Struct,Float32l,Int8ub,this
import numpy as np
import signal
import sys
import copy

fsrformat=Struct(
    "len" / Int8ub,
    "fsrdata" / Array(this.len,FSR2CHSample_t)
)

sineformat=Struct(
    "len" / Int8ub,
    "sinedata" / Array(this.len,floatSample_t)
)

syncformat=Struct(
    "len" / Int8ub,
    "syncdata" / Array(this.len,uint8_tSample_t)
)

if rospy.has_param('eris/port'):
    port=rospy.get_param('eris/port')
else:
    port='/dev/ttyACM0'

##################### ROS MESSAGES AND PUBLISHERS ##############################
fsrmsg=FSR2CH()
sinemsg=Float32()
syncmsg=Uint8()
textpub = rospy.Publisher('eris/print', String, queue_size=50)
sinepub = rospy.Publisher('/record/eris/sine', Float32, queue_size=50)
syncpub = rospy.Publisher('eris/sync', Uint8, queue_size=50)
fsrpub = rospy.Publisher('/record/eris/fsr', FSR2CH, queue_size=50)
T0=0 #absolute time

def publishFSR(sample):
    '''Publish data for FSR'''
    timestamp=sample['timestamp']
    fsrmsg.header=Header(stamp=T0+rospy.Duration(timestamp/1000.0))
    fsrmsg.ch0=sample['ch'][0]
    fsrmsg.ch1=sample['ch'][1]
    #fsrmsg.ch1=sample['ch1']
    #fsrmsg.ch2=sample['ch2']
    #fsrmsg.ch3=sample['ch3']
    fsrpub.publish(fsrmsg)

def publishSine(sample):
    '''Publish data for Sine'''
    timestamp=sample['timestamp']
    sinemsg.header=Header(stamp=T0+rospy.Duration(timestamp/1000.0))
    sinemsg.data=sample['value']
    sinepub.publish(sinemsg)

def publishSync(sample):
    '''Publish data for Sync'''
    timestamp=sample['timestamp']
    syncmsg.header=Header(stamp=T0+rospy.Duration(timestamp/1000.0))
    syncmsg.data=sample['value']
    syncpub.publish(syncmsg)

def publishText(data):
    textmsg.data = data[0];
    textmsg.header=Header(stamp=rospy.Time.now())
    textpub.publish(textmsg)

def command_callback(msg):
    ''' A callback to transmit a command to eris'''
    e.sendCommand(msg.data)

################################################################################
#Create an eris object
e=Eris(['FSR','SINE','SYNC'],[fsrformat,sineformat,syncformat],port)

######################## HELPER FUNCTIONS ######################################
def signal_handler(sig,frame):
    ''' Terminate the connection to eris and close the node'''
    print('Ctrl+c')
    e.sendCommand('S_OFF')
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
            #TODO publish a missing data message under eris/errors
            continue
	#FSR
        fsrdata=packetData['FSR']['fsrdata']
        n=packetData['FSR']['len']
        for sample in fsrdata: #all channels should have same lenght
            publishFSR(sample)
	#SINE
        sinedata=packetData['SINE']['sinedata']
        n=packetData['SINE']['len']
        for sample in sinedata: #all channels should have same lenght
            publishSine(sample)
	#SYNC
        syncdata=packetData['SYNC']['syncdata']
        n=packetData['SYNC']['len']
        for sample in syncdata: #all channels should have same lenght
            publishSync(sample)

################################################################################

''' Main loop'''
dataMutex=Lock();
rospy.init_node('fsrnode', anonymous=True)
cmdsub = rospy.Subscriber('eris/command',String,command_callback)

ROSRATE=50 #Hz
rate = rospy.Rate(ROSRATE)
e.sendCommand('S_OFF')
e.sendCommand('S_TIME') #Reset time to 0

rate.sleep()
e.sendCommand('S_ON') #Initilize automatic streaming of data
T0=rospy.Time.now()
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
            #This can be slow since it is data logging
            #t2=Thread(target=publishData,args=(d['D'],))
            #t2.start()
            publishData(d['D'])
    #rospy.loginfo_once("This message will print only once")
    rate.sleep()
e.sendCommand('S_OFF')
e.stop()
