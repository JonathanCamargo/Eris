#!/usr/bin/env python
# READ IMU DATA FROM ERIS AND SEND TO ROS TOPICS

import rospy
from eris.eris import Eris
from eris.customtypes import IMUSample_t, floatSample_t
from custom_msgs.msg import IMU,Float32,Signal1CH,String
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
imumsg=IMU()
sinemsg=Float32()
fsrmsg=Signal1CH()

textpub = rospy.Publisher('print', String, queue_size=50)
imu0pub = rospy.Publisher('imu0', IMU, queue_size=100)
imu1pub = rospy.Publisher('imu1', IMU, queue_size=100)
imu2pub = rospy.Publisher('imu2', IMU, queue_size=100)
imu3pub = rospy.Publisher('imu3', IMU, queue_size=100)
sinepub = rospy.Publisher('sine', Float32, queue_size=100)

t0=0 #global variable to store time reference to linux time

def publishIMU(sample,pub):
    '''Publish data for IMU'''
    timestamp=sample['timestamp']/1000.0
    imumsg.header=Header(stamp=t0+rospy.Duration(timestamp))
    imumsg.ax=sample['ax']
    imumsg.ay=sample['ay']
    imumsg.az=sample['az']
    imumsg.wx=sample['wx']
    imumsg.wy=sample['wy']
    imumsg.wz=sample['wz']
    pub.publish(imumsg)

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
streams=['IMU_0','IMU_1','IMU_2','IMU_3','SINE']
streamsformat=[IMUSample_t,IMUSample_t,IMUSample_t,IMUSample_t,floatSample_t]
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
rospy.init_node('eris_imu', anonymous=True)
cmdsub = rospy.Subscriber('eris/command',String,command_callback)

ROSRATE=50 #Hz
rate = rospy.Rate(ROSRATE)
e.sendCommand('INITIMU')
print("Waiting for IMU initialization (30 seconds)")

counts=0
while (counts<30):
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
        for sample in p['IMU_0']:
            publishIMU(sample,imu0pub)
        for sample in p['IMU_1']:
            publishIMU(sample,imu1pub)
        for sample in p['IMU_2']:
            publishIMU(sample,imu2pub)
        for sample in p['IMU_3']:
            publishIMU(sample,imu3pub)
    for p in out['E']:
    	print('SUPUTAMADREEEROR')
    	print(p)

    #rospy.loginfo_once("This message will print only once")
    rate.sleep()
e.sendCommand('S_OFF')
e.stop()
