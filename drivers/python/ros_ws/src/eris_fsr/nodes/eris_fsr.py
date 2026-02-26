#!/usr/bin/env python
'''Read imu data from Eris'''

from eris.eris import Eris
from eris.customtypes import Signal1CHSample_t , floatSample_t, uint8_tSample_t

import rospy

from custom_msgs.msg import Signal1CH,String,Float32,Uint8
import std_msgs
from std_msgs.msg import Header

import numpy as np
import signal
import sys

if rospy.has_param('eris/port'):
    port=rospy.get_param('eris/port')
else:
    port='/dev/ttyACM0'

##################### ROS MESSAGES AND PUBLISHERS ##############################
fsrmsg=Signal1CH()
sinemsg=Float32()
fsrfloatmsg=std_msgs.msg.Float32()

textpub = rospy.Publisher('/eris/print', String, queue_size=50)
sinepub = rospy.Publisher('/eris/sine', Float32, queue_size=50)
syncpub = rospy.Publisher('/eris/sync', Uint8, queue_size=50)
fsrpub = rospy.Publisher('/eris/fsr', Signal1CH, queue_size=50)
fsrfloatpub = rospy.Publisher('/float', std_msgs.msg.Float32, queue_size=50)

T0=0 #absolute time

def publishFSR(sample):
    '''Publish data for FSR'''
    timestamp=sample['timestamp']/1000
    fsrmsg.header=Header(stamp=T0+rospy.Duration(timestamp))
    fsrmsg.ch0=sample['ch'][0]
    #fsrmsg.ch1=sample['ch'][1]
    #fsrmsg.ch1=sample['ch1']
    #fsrmsg.ch2=sample['ch2']
    #fsrmsg.ch3=sample['ch3']
    fsrpub.publish(fsrmsg)
    fsrfloatmsg.data=sample['ch'][0]
    fsrfloatpub.publish(fsrfloatmsg)

def publishSine(sample):
    '''Publish data for Sine'''
    timestamp=sample['timestamp']
    sinemsg.header=Header(stamp=T0+rospy.Duration(timestamp))
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
e=Eris(['FSR','SINE'],[Signal1CHSample_t,floatSample_t],port)

######################## HELPER FUNCTIONS ######################################
def signal_handler(sig,frame):
    ''' Terminate the connection to eris and close the node'''
    print('Ctrl+c')
    e.sendCommand('S_OFF')
    sys.exit(0)
signal.signal(signal.SIGINT,signal_handler)

################################################################################

''' Main loop'''
rospy.init_node('fsrnode', anonymous=True)
cmdsub = rospy.Subscriber('eris/command',String,command_callback)

ROSRATE=50 #Hz
rate = rospy.Rate(ROSRATE)
e.sendCommand('S_T0') #Reset time to 0
T0=rospy.Time.now()
rate.sleep()

e.sendCommand('S_ON') #Reset time to 0
print('Inicio')
# Transmit a time stamp to eris to sync with pi time ??
e.start()

while True:
    #Get data from teensy
    try:
        out=e.read()
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
        for sample in p['FSR']:
            publishFSR(sample)

    #rospy.loginfo_once("This message will print only once")
    rate.sleep()
e.sendCommand('S_OFF')
e.stop()
