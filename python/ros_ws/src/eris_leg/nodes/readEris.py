#!/usr/bin/env python
'''Read joint and loadcell data from Eris'''
''' Send impedance parameters updates by serial command'''

from eris.eris import Eris
from eris.customtypes import JointStateSample_t , LoadcellSample_t, floatSample_t

import rospy
import threading

from custom_msgs.msg import Loadcell,String,Float32,JointState, ScaledParameters
from std_msgs.msg import Header
from threading import Thread,Lock

#Use construct to create a good c format
from construct import Array,Struct,Float32l,Int8ub,this
import numpy as np
import signal
import sys
import copy

jointformat=Struct(
    "len" / Int8ub,
    "jointdata" / Array(this.len,JointStateSample_t)
)

loadcellformat=Struct(
    "len" / Int8ub,
    "loadcelldata" / Array(this.len,LoadcellSample_t)
)

sineformat=Struct(
    "len" / Int8ub,
    "data" / Array(this.len,floatSample_t)
)
if rospy.has_param('eris/port'):
    port=rospy.get_param('eris/port')
else:
    port='/dev/ttyACM0'

##################### ROS MESSAGES AND PUBLISHERS ##############################

#Messages placeholders for ROS topics
jointmsg=JointState()
sinemsg=Float32()
textmsg=String()
loadcellmsg=Loadcell()

textpub = rospy.Publisher('eris/print', String, queue_size=50)
sinepub = rospy.Publisher('eris/sine', Float32, queue_size=50)
kneepub = rospy.Publisher('/fsm/knee/joint_state', JointState, queue_size=50)
anklepub = rospy.Publisher('/fsm/ankle/joint_state', JointState, queue_size=50)
loadcellpub = rospy.Publisher('/fsm/wrench', Loadcell, queue_size=50)

t0=0 #global variable to store time reference to linux time

def publishJoint(sample,publisher):
    '''Publish data for a joint''' 
    timestamp=sample['timestamp']
    jointmsg.header=Header(stamp=rospy.Duration(timestamp/1000.0)+t0)
    jointmsg.theta=sample['theta']
    jointmsg.theta_dot=sample['theta_dot']
    publisher.publish(jointmsg)
	
def publishLoadcell(sample):
    '''Publish data for the loadcell'''
    timestamp=sample['timestamp']
    loadcellmsg.header=Header(stamp=rospy.Duration(timestamp/1000.0)+t0)
    loadcellmsg.forceX=sample['forceX']
    loadcellmsg.forceY=sample['forceY']
    loadcellmsg.forceZ=sample['forceZ']
    loadcellpub.publish(loadcellmsg)
	
def publishSine(sample):
    '''Publish data for Sinewave'''
    timestamp=sample['timestamp']
    sinemsg.header=Header(stamp=t0+rospy.Duration(timestamp/1000.0))
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
e=Eris(['KNEE','ANKLE','LC','SINE'],[jointformat,jointformat,loadcellformat,sineformat],port)

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

        #KNEE       
        kneedata=packetData['KNEE']['jointdata']
        n=packetData['KNEE']['len']
        for sample in kneedata: #all channels should have same lenght
            publishJoint(sample,kneepub)
		
        #ANKLE	
        ankledata=packetData['ANKLE']['jointdata']
        n=packetData['ANKLE']['len']
        for sample in ankledata: #all channels should have same lenght
            publishJoint(sample,anklepub)
			
        #LC
        loadcelldata=packetData['LC']['loadcelldata']
        n=packetData['LC']['len']
        for sample in loadcelldata: #all channels should have same lenght
            publishLoadcell(sample)
	
        #SINE		
        sinedata=packetData['SINE']['data']
        n=packetData['SINE']['len']
        for sample in sinedata: #all channels should have same lenght
            publishSine(sample)

def setIPK_callback(msg):
    print(msg)
    erisMutex.acquire(1)
    tosend="IPK %1.2f %1.2f %1.2f\n" % (msg.k, msg.b, msg.theta) 
    e.sendCommand(tosend)
    erisMutex.release()

def setIPA_callback(msg):
    erisMutex.acquire(1)
    tosend="IPA %1.2f %1.2f %1.2f\n" % (msg.k, msg.b, msg.theta) 
    e.sendCommand(tosend)
    erisMutex.release()

################################################################################

''' Main loop'''
dataMutex=Lock();
erisMutex=Lock();
rospy.init_node('imunode', anonymous=True)

cmdsub = rospy.Subscriber('eris/command',String,command_callback)

kneeparamssub=rospy.Subscriber('/fsm/knee/scaled_params',ScaledParameters,setIPK_callback,queue_size=1)

ankleparamssub=rospy.Subscriber('/fsm/ankle/scaled_params',ScaledParameters,setIPA_callback,queue_size=1)


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
