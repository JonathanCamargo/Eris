#!/usr/bin/env python
# READ EMG FEATURE DATA FROM ERIS AND SEND TO ROS TOPICS
# read EMG and ETI only

import rospy
from eris.eris import Eris
from eris.customtypes import Signal1CHSample_t, floatSample_t, uint8_tSample_t
from custom_msgs.msg import Signal1CH, String, Float32, Bool, Float32MultiArray, ETI
from std_msgs.msg import Header, MultiArrayDimension

from construct import Struct,Float32l,Int8ub
import numpy as np
import signal
import sys

tiSample_t = Struct(
    "timestamp" / Float32l,
    "temperature" / Float32l[1],
    "impedance" / Float32l[1]
    )


if rospy.has_param('eris/port'):
    port=rospy.get_param('eris/port')
else:
    port='/dev/ttyACM0'

##################### ROS MESSAGES AND PUBLISHERS ##############################
emgmsg=Signal1CH()
sinemsg=Float32()
etimsg=ETI()
fsrmsg=Signal1CH()

featuresmsg=Float32MultiArray()

textpub = rospy.Publisher('/print', String, queue_size=50)
sinepub = rospy.Publisher('/sine', Float32, queue_size=50)
etipub = rospy.Publisher('/ti', ETI, queue_size=50)
emgpub = rospy.Publisher('/emg', Signal1CH, queue_size=100)
fsrpub = rospy.Publisher('/fsr', Signal1CH, queue_size=50)
featurespub = rospy.Publisher('/features', Float32MultiArray, queue_size=1)

t0=0 #global variable to store time reference to linux time
def publishEMG(sample):
    '''Publish data for EMG'''
    timestamp=sample['timestamp']/1000.0
    #print(timestamp)
    emgmsg.header=Header(stamp=t0+rospy.Duration(timestamp))
    emgmsg.ch0=sample['ch'][0]
    #emgmsg.ch1=sample['ch'][1]
    #emgmsg.ch2=sample['ch'][2]
    #emgmsg.ch3=sample['ch'][3]
    #emgmsg.ch4=sample['ch'][4]
    #emgmsg.ch5=sample['ch'][5]
    #emgmsg.ch6=sample['ch'][6]
    #emgmsg.ch7=sample['ch'][7]
    emgpub.publish(emgmsg)
    #print(emgmsg)

def publishFSR(sample):
    '''Publish data for FSR'''
    timestamp=sample['timestamp']/1000.0
    #print(timestamp)
    fsrmsg.header=Header(stamp=t0+rospy.Duration(timestamp))
    fsrmsg.ch0=sample['ch'][0]
    #emgmsg.ch1=sample['ch'][1]
    #emgmsg.ch2=sample['ch'][2]
    #emgmsg.ch3=sample['ch'][3]
    #emgmsg.ch4=sample['ch'][4]
    #emgmsg.ch5=sample['ch'][5]
    #emgmsg.ch6=sample['ch'][6]
    #emgmsg.ch7=sample['ch'][7]
    fsrpub.publish(fsrmsg)
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
#What to read from Eris?
streams=['SINE','EMG','ETI','FSR']
#streams=['FSR']
streamsformat=[floatSample_t,Signal1CHSample_t,tiSample_t,Signal1CHSample_t]
e=Eris(streams,streamsformat,port)

######################## HELPER FUNCTIONS ######################################
def signal_handler(sig,frame):
    ''' Terminate the connection to eris and close the node'''
    print('Ctrl+c')
    e.stop()
    sys.exit(0)
signal.signal(signal.SIGINT,signal_handler)

################################################################################

''' Main loop'''
rospy.init_node('nextflexAnalog', anonymous=True)
cmdsub = rospy.Subscriber('eris/command',String,command_callback)

ROSRATE=50 #Hz
rate = rospy.Rate(ROSRATE)
e.sendCommand('S_T0') #Reset time to 0
t0=rospy.Time.now()
rate.sleep()

print('Inicio')

e.start()

while True:

    try:
    	out = e.read()
    except Exception as ex:
        print('Problem')
        print(ex)
        e.sendCommand('S_OFF')
        rate.sleep()
        e.sendCommand('S_ON')
        rate.sleep()
        continue

    for p in out['D']:
        for sample in p['SINE']:
            publishSine(sample)
        for sample in p['EMG']:
            publishEMG(sample)
        for sample in p['ETI']:
            publishETI(sample)
        for sample in p['FSR']:
            publishFSR(sample)
    #rospy.loginfo_once("This message will print only once")
    rate.sleep()
e.sendCommand('S_OFF')
e.stop()
