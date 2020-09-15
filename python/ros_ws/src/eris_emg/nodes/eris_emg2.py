#!/usr/bin/env python
# READ EMG FEATURE DATA FROM ERIS AND SEND TO ROS TOPICS
# read EMG from ETI Arrays and offer the option to modify the reference selection

import rospy
from eris.eris import Eris
from eris.customtypes import Signal1CHSample_t,Signal8CHSample_t, floatSample_t, uint8_tSample_t
from custom_msgs.msg import Signal1CH,Signal8CH, String, Float32, Bool, Float32MultiArray
from std_msgs.msg import Header, MultiArrayDimension
from std_msgs.msg import Float32 as stdmsgsFloat32

from construct import Struct,Float32l,Int8ub
import numpy as np
import signal
import sys

import threading

if rospy.has_param('eris_emg2/port'):
    port=rospy.get_param('eris_emg2/port')
else:
    port='/dev/ttyACM1'

namespace='eris_emg2'

##################### ROS MESSAGES AND PUBLISHERS ##############################
emgmsg=Signal8CH()
sinemsg=Float32()
fsrmsg=Signal1CH()

textpub = rospy.Publisher(namespace+'/print', String, queue_size=50)
emgpub = rospy.Publisher(namespace+'/emg', Signal8CH, queue_size=100)

t0=0 #global variable to store time reference to linux time
def publishEMG(sample):
    '''Publish data for EMG'''
    timestamp=sample['timestamp']/1000.0
    #print(timestamp)
    emgmsg.header=Header(stamp=t0+rospy.Duration(timestamp))
    emgmsg.ch0=sample['ch'][0]
    emgmsg.ch1=sample['ch'][1]
    emgmsg.ch2=sample['ch'][2]
    emgmsg.ch3=sample['ch'][3]
    emgmsg.ch4=sample['ch'][4]
    emgmsg.ch5=sample['ch'][5]
    emgmsg.ch6=sample['ch'][6]
    emgmsg.ch7=sample['ch'][7]
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

def publishText(data):
    textmsg.data = data[0];
    textmsg.header=Header(stamp=rospy.Time.now())
    textpub.publish(textmsg)

def command_callback(msg):
    ''' A callback to transmit a command to eris'''
    e.sendCommand(msg.data)


################################################################################
#Create an eris object
#What to read from Eris?
#streams=['SINE','EMG','FSR']
streams=['EMG']
#streamsformat=[floatSample_t,Signal8CHSample_t,Signal1CHSample_t]
streamsformat=[Signal8CHSample_t]
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
rospy.init_node('eris_emg', anonymous=True)
cmdsub = rospy.Subscriber(namespace+'eris/command',String,command_callback)

ROSRATE=50 #Hz
rate = rospy.Rate(ROSRATE)
e.sendCommand('S_TIME') #Reset time to 0
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
        #for sample in p['SINE']:
        #    publishSine(sample)
        for sample in p['EMG']:
            publishEMG(sample)
        #for sample in p['FSR']:
        #    publishFSR(sample)


    #rospy.loginfo_once("This message will print only once")
    rate.sleep()
e.sendCommand('S_OFF')
e.stop()
