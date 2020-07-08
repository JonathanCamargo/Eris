#!/usr/bin/env python
'''Read single imu data from Eris'''

from eris.eris import Eris
from eris.customtypes import IMUSample_t , floatSample_t

import rospy
import threading

from custom_msgs.msg import IMU,String,Float32
from std_msgs.msg import Header
from threading import Thread,Lock

#Use construct to create a good c format
from construct import Array,Struct,Float32l,Int8ub,this
import numpy as np
import signal
import sys
import copy

imuformat=Struct(
    "len" / Int8ub,
    "imudata" / Array(this.len,IMUSample_t)
)

sineformat=Struct(
    "len" / Int8ub,
    "sinedata" / Array(this.len,floatSample_t)
)

if rospy.has_param('eris/port'):
    port=rospy.get_param('eris/port')
else:
    port='/dev/ttyACM0'

##################### ROS MESSAGES AND PUBLISHERS ##############################
imumsg=IMU()
sinemsg=Float32()
textpub = rospy.Publisher('eris/print', String, queue_size=50)
sinepub = rospy.Publisher('eris/sine', Float32, queue_size=50)
imupub = rospy.Publisher('eris/imu', IMU, queue_size=50)
t0=0 #global variable to store time reference to linux time
def publishIMU(sample):
    '''Publish data for IMU'''
    timestamp=sample['timestamp']
    imumsg.header=Header(stamp=t0+rospy.Duration(timestamp)/1000.0)
    imumsg.ax=sample['ax']
    imumsg.ay=sample['ay']
    imumsg.az=sample['az']
    imumsg.wx=sample['wx']
    imumsg.wy=sample['wy']
    imumsg.wz=sample['wz']
    imupub.publish(imumsg)

def publishSine(sample):
    '''Publish data for IMU'''
    timestamp=sample['timestamp']
    sinemsg.header=Header(stamp=t0+rospy.Duration(timestamp)/1000.0)
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
e=Eris(['IMU_2','SINE'],[imuformat,sineformat],port)

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
        imudata=packetData['IMU_2']['imudata']
        n=packetData['IMU_2']['len']
        for sample in imudata: #all channels should have same lenght
            publishIMU(sample)
        sinedata=packetData['SINE']['sinedata']
        n=packetData['SINE']['len']
        for sample in sinedata: #all channels should have same lenght
            publishSine(sample)

################################################################################

''' Main loop'''
dataMutex=Lock();
rospy.init_node('imunode', anonymous=True)
cmdsub = rospy.Subscriber('eris/command',String,command_callback)

ROSRATE=50 #Hz
rate = rospy.Rate(ROSRATE)
e.sendCommand('S_OFF')
e.sendCommand('S_TIME') #Reset time to 0
t0=rospy.Time.now()
rate.sleep()
e.sendCommand('S_ON') #Initilize automatic streaming of data
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
