#!/usr/bin/env python
# READ IMU DATA FROM ERIS AND SEND TO ROS TOPICS

import rospy
from eris.eris import Eris
from eris.customtypes import  Signal2CHSample_t,floatSample_t
from custom_msgs.msg import Float32,Signal2CH,String
from std_msgs.msg import Header

import numpy as np
import signal
import sys

import threading
from time import sleep

if rospy.has_param('eris/port'):
    port=rospy.get_param('eris/port')
else:
    port='/dev/ttyACM0'


##################### ROS MESSAGES AND PUBLISHERS ##############################
analogmsg=Signal2CH()
sinemsg=Float32()

textpub = rospy.Publisher('print', String, queue_size=50)
analogpub = rospy.Publisher('analog', Signal2CH, queue_size=100)
sinepub = rospy.Publisher('sine', Float32, queue_size=100)

t0=0 #global variable to store time reference to linux time

def publishAnalog(sample):
    '''Publish data for IMU'''
    timestamp=sample['timestamp']/1000.0
    analogmsg.header=Header(stamp=t0+rospy.Duration(timestamp))
    analogmsg.ch0=sample['ch'][0]
    #analogmsg.ch1=sample['ch'][1]
    #analogmsg.ch2=sample['ch'][2]
    #analogmsg.ch3=sample['ch'][3]
    #analogmsg.ch4=sample['ch'][4]
    #analogmsg.ch5=sample['ch'][5]
    #analogmsg.ch6=sample['ch'][6]
    #analogmsg.ch7=sample['ch'][7]
    analogpub.publish(analogmsg)

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
streams=['Analog','SINE']
streamsformat=[Signal2CHSample_t,floatSample_t]
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
rospy.init_node('eris_analog', anonymous=True)
cmdsub = rospy.Subscriber('eris/command',String,command_callback)

ROSRATE=50 #Hz
rate = rospy.Rate(ROSRATE)
print("Waiting for initialization (5 seconds)")

counts=0
while (counts<5):
    sleep(1.0)
    counts=counts+1
    out = e.read()
    #Get error and shutdown if initialization failed
    print(out['T'])
    for er in out['E']:
        rospy.logerr(er)


print("Done init")
e.sendCommand('TIME0') #Reset time to 0
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
        for sample in p['Analog']:
            publishAnalog(sample)
    for p in out['E']:
    	print('SUPUTAMADREEEROR')
    	print(p)

    #rospy.loginfo_once("This message will print only once")
    rate.sleep()
e.sendCommand('S_OFF')
e.stop()
