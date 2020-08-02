#!/usr/bin/env python
# READ ONE floatSample_t DATA FROM ERIS AND SEND TO ROS TOPICS


import rospy
from custom_msgs.msg import Float32, String
from std_msgs.msg import Header
from eris.eris import Eris
from eris.customtypes import floatSample_t
import argparse

import numpy as np
import signal
import sys

if rospy.has_param('eris/port'):
    port=rospy.get_param('eris/port')
else:
    port='/dev/ttyACM1'

##################### ROS MESSAGES AND PUBLISHERS ##############################
msg=Float32()

topicname='/eris/pot/'

textpub = rospy.Publisher('/eris/print', String, queue_size=50)
pub = rospy.Publisher(topicname, Float32, queue_size=50)
sinepub = rospy.Publisher('/eris/sine', Float32, queue_size=50)

t0=0 #global variable to store time reference to linux time
def publishFloat(floatPublisher,sample):
    '''Publish data '''
    timestamp=sample['timestamp']/1000.0
    msg.header=Header(stamp=t0+rospy.Duration(timestamp))
    msg.data=sample['value']
    floatPublisher.publish(msg)

def publishText(data):
    textmsg.data = data[0];
    textmsg.header=Header(stamp=rospy.Time.now())
    textpub.publish(textmsg)

################################################################################
#Create an eris object
#What to read from Eris?
streams=['POT','SINE']
#streams=['FSR']
streamsformat=[floatSample_t,floatSample_t]
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
            publishFloat(sinepub,sample)
        for sample in p['POT']:
            publishFloat(pub,sample)

    #rospy.loginfo_once("This message will print only once")
    rate.sleep()
e.sendCommand('S_OFF')
e.stop()
